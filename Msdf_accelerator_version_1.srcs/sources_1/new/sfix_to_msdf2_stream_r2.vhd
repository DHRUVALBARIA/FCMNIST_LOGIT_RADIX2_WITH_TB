library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

-- ============================================================================
-- File : sfix_to_msdf2_stream_r2.vhd  (ACCEL-safe version)
--
-- Fixed-point (w_fixed_t, scaled by 2^W_FRAC_BITS) -> radix-2 MSDF digit stream
-- digits d_k in {-1,0,+1}, MSD-first fractional:
--   x ≈ sum_{k=1..N} d_k * 2^(-k)
--
-- Contract:
-- . start loads x_in into the internal remainder and enters RUN
-- . step advances exactly one digit when RUN and ce=1
-- . step must only be pulsed when the downstream accepted the previous digit
-- . valid and done are 1-cycle pulses, cleared every clock edge (even if ce=0)
-- ============================================================================

entity sfix_to_msdf2_stream_r2 is
  generic (
    N_DIGITS_G : integer := N_DIGITS
  );
  port (
    clk     : in  std_logic;
    rst_n   : in  std_logic;  -- active-low synchronous reset
    ce      : in  std_logic;  -- clock enable

    start   : in  std_logic;  -- pulse: load x_in and go RUN (restart allowed)
    step    : in  std_logic;  -- pulse: emit one digit when '1'

    x_in    : in  w_fixed_t;

    digit_out   : out sd_digit_t;
    valid       : out std_logic; -- 1-cycle pulse when digit_out updates
    done        : out std_logic; -- 1-cycle pulse with the last digit
    busy        : out std_logic; -- high while RUN (even if step=0)
    digit_count : out integer range 0 to N_DIGITS_G
  );
end entity;

architecture rtl of sfix_to_msdf2_stream_r2 is

  -- Extend remainder width a bit for safe comparisons and shifts.
  constant EXT_W : integer := W_TOTAL_BITS + 2;
  subtype  ext_s is signed(EXT_W-1 downto 0);

  function pow2_s(k : natural) return ext_s is
    variable v : ext_s := (others => '0');
  begin
    v(0) := '1';
    return shift_left(v, integer(k));
  end function;

  constant ONE_C  : ext_s := pow2_s(natural(W_FRAC_BITS));      -- 1.0
  constant HALF_C : ext_s := pow2_s(natural(W_FRAC_BITS - 1));  -- 0.5
  constant LSB_C  : ext_s := pow2_s(0);                         -- 1 LSB

  constant CLAMP_LO : ext_s := -ONE_C + LSB_C;
  constant CLAMP_HI : ext_s :=  ONE_C - LSB_C;

  function clamp_s(x, lo, hi : ext_s) return ext_s is
  begin
    if x < lo then return lo;
    elsif x > hi then return hi;
    else return x;
    end if;
  end function;

  function i_to_sd(i : integer) return sd_digit_t is
  begin
    case i is
      when -1 => return SD_NEG1;
      when  0 => return SD_ZERO;
      when  1 => return SD_POS1;
      when others => return SD_ZERO;
    end case;
  end function;

  type state_t is (IDLE, RUN);
  signal state_r : state_t := IDLE;

  signal rem_r   : ext_s := (others => '0');
  signal cnt_r   : integer range 0 to N_DIGITS_G := 0;

  signal digit_r : sd_digit_t := SD_ZERO;
  signal valid_r : std_logic  := '0';
  signal done_r  : std_logic  := '0';
  signal busy_r  : std_logic  := '0';

begin
  digit_out   <= digit_r;
  valid       <= valid_r;
  done        <= done_r;
  busy        <= busy_r;
  digit_count <= cnt_r;

  assert W_FRAC_BITS >= 1
    report "sfix_to_msdf2_stream_r2: W_FRAC_BITS must be >= 1"
    severity failure;

  assert N_DIGITS_G >= 1
    report "sfix_to_msdf2_stream_r2: N_DIGITS_G must be >= 1"
    severity failure;

  process(clk)
    variable rem2  : ext_s;
    variable rem_n : ext_s;
    variable d_int : integer;
    variable xin_e : ext_s;
  begin
    if rising_edge(clk) then

      if rst_n = '0' then
        valid_r <= '0';
        done_r  <= '0';
        state_r <= IDLE;
        rem_r   <= (others => '0');
        cnt_r   <= 0;
        digit_r <= SD_ZERO;
        busy_r  <= '0';

      else
        -- Pulse-safe: clear pulses every clock edge, independent of ce
        valid_r <= '0';
        done_r  <= '0';

        if ce = '1' then
          case state_r is

            when IDLE =>
              busy_r  <= '0';
              digit_r <= SD_ZERO;
              cnt_r   <= 0;

              if start = '1' then
                xin_e   := resize(x_in, EXT_W);
                rem_r   <= clamp_s(xin_e, CLAMP_LO, CLAMP_HI);
                cnt_r   <= 0;
                state_r <= RUN;
                busy_r  <= '1';
              end if;

            when RUN =>
              busy_r <= '1';

              if start = '1' then
                -- restart mid-run
                xin_e   := resize(x_in, EXT_W);
                rem_r   <= clamp_s(xin_e, CLAMP_LO, CLAMP_HI);
                cnt_r   <= 0;
                digit_r <= SD_ZERO;

              elsif step = '1' then
                -- one accepted digit step
                rem2 := shift_left(rem_r, 1);

                if rem2 >= HALF_C then
                  d_int :=  1;
                  rem_n := rem2 - ONE_C;
                elsif rem2 <= -HALF_C then
                  d_int := -1;
                  rem_n := rem2 + ONE_C;
                else
                  d_int :=  0;
                  rem_n := rem2;
                end if;

                digit_r <= i_to_sd(d_int);
                valid_r <= '1';
                rem_r   <= rem_n;

                if cnt_r = N_DIGITS_G-1 then
                  done_r  <= '1';
                  state_r <= IDLE;
                  busy_r  <= '0';
                  cnt_r   <= 0;
                else
                  cnt_r <= cnt_r + 1;
                end if;
              end if;

          end case;
        end if; -- ce
      end if; -- rst
    end if; -- rising_edge
  end process;

end architecture;
