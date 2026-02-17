library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

entity tb_sfix_to_msdf2_stream_r2 is
end entity;

architecture tb of tb_sfix_to_msdf2_stream_r2 is

  constant CLK_PERIOD : time := 10 ns;

  signal clk   : std_logic := '0';
  signal rst_n : std_logic := '0';
  signal ce    : std_logic := '1';

  signal start : std_logic := '0';
  signal step  : std_logic := '0';
  signal x_in  : w_fixed_t := (others => '0');

  signal digit_out   : sd_digit_t;
  signal valid       : std_logic;
  signal done        : std_logic;
  signal busy        : std_logic;
  signal digit_count : integer range 0 to N_DIGITS;

  -- TB copies the effective digit count we want to test
  constant N_DIGITS_TB : integer := N_DIGITS;

  -- Local copy of the DUT’s math (same thresholds and clamp)
  constant EXT_W : integer := W_TOTAL_BITS + 2;
  subtype ext_s is signed(EXT_W-1 downto 0);

  function pow2_s(k : natural) return ext_s is
    variable v : ext_s := (others => '0');
  begin
    v(0) := '1';
    return shift_left(v, integer(k));
  end function;

  constant ONE_C  : ext_s := pow2_s(natural(W_FRAC_BITS));      -- 1.0
  constant HALF_C : ext_s := pow2_s(natural(W_FRAC_BITS - 1));  -- 0.5
  constant LSB_C  : ext_s := pow2_s(0);

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

begin

  clk <= not clk after CLK_PERIOD/2;

  dut : entity work.sfix_to_msdf2_stream_r2
    generic map (
      N_DIGITS_G => N_DIGITS_TB
    )
    port map (
      clk         => clk,
      rst_n       => rst_n,
      ce          => ce,
      start       => start,
      step        => step,
      x_in        => x_in,
      digit_out   => digit_out,
      valid       => valid,
      done        => done,
      busy        => busy,
      digit_count => digit_count
    );

  stim : process
    variable rem_ref : ext_s := (others => '0');

    procedure tick is
    begin
      wait until rising_edge(clk);
      wait for 1 ns;
    end procedure;

    procedure do_reset is
    begin
      rst_n <= '0';
      ce    <= '1';
      start <= '0';
      step  <= '0';
      x_in  <= (others => '0');
      tick;
      tick;
      rst_n <= '1';
      tick;
      rem_ref := (others => '0');
    end procedure;

    procedure do_start(constant xval : in integer) is
      variable xin_e : ext_s;
    begin
      x_in   <= to_signed(xval, W_TOTAL_BITS);
      start  <= '1';
      step   <= '0';
      tick;
      start <= '0';

      xin_e   := resize(to_signed(xval, W_TOTAL_BITS), EXT_W);
      rem_ref := clamp_s(xin_e, CLAMP_LO, CLAMP_HI);

      -- After start, DUT should be busy in RUN
      assert busy = '1'
        report "TB ERROR: busy should be 1 after start"
        severity failure;

      -- start does not emit a digit
      assert valid = '0'
        report "TB ERROR: valid should be 0 on start cycle"
        severity failure;
    end procedure;

    procedure step_check(constant expect_done : in boolean) is
      variable rem2  : ext_s;
      variable rem_n : ext_s;
      variable d_int : integer;
      variable d_exp : sd_digit_t;
      variable cnt_before : integer;
    begin
      cnt_before := digit_count;

      step <= '1';
      tick;
      step <= '0';

      -- If ce=0, nothing should happen
      if ce = '0' then
        assert valid = '0'
          report "TB ERROR: valid should stay 0 when ce=0"
          severity failure;
        assert done = '0'
          report "TB ERROR: done should stay 0 when ce=0"
          severity failure;
        assert digit_count = cnt_before
          report "TB ERROR: digit_count should not change when ce=0"
          severity failure;
        return;
      end if;

      -- Reference digit generation (matches DUT)
      rem2 := shift_left(rem_ref, 1);

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

      d_exp := i_to_sd(d_int);

      assert valid = '1'
        report "TB ERROR: valid not asserted on accepted step"
        severity failure;

      assert digit_out = d_exp
        report "TB ERROR: digit_out mismatch"
        severity failure;

      if expect_done then
        assert done = '1'
          report "TB ERROR: done not asserted on last digit"
          severity failure;
        assert busy = '0'
          report "TB ERROR: busy should go 0 on completion"
          severity failure;
      else
        assert done = '0'
          report "TB ERROR: done asserted early"
          severity failure;
        assert busy = '1'
          report "TB ERROR: busy should remain 1 mid-run"
          severity failure;
      end if;

      rem_ref := rem_n;
    end procedure;

    variable i : integer;

    constant HALF_X : integer := 2**natural(W_FRAC_BITS-1); -- +0.5
    constant QTR_X  : integer := 2**natural(W_FRAC_BITS-2); -- +0.25

  begin
    do_reset;

    -- Directed: +0.5 => [+1, 0, 0, ...]
    do_start(HALF_X);
    step_check(false);  -- +1
    for i in 2 to N_DIGITS_TB-1 loop
      step_check(false); -- zeros
    end loop;
    step_check(true);

    -- Directed: -0.5 => [-1, 0, 0, ...]
    do_start(-HALF_X);
    step_check(false);  -- -1
    for i in 2 to N_DIGITS_TB-1 loop
      step_check(false);
    end loop;
    step_check(true);

    -- Directed: +0.25 => [0, +1, 0, ...]
    do_start(QTR_X);
    step_check(false);  -- 0
    step_check(false);  -- +1
    for i in 3 to N_DIGITS_TB-1 loop
      step_check(false);
    end loop;
    step_check(true);

    -- ce gating: ce=0 prevents stepping
    do_start(QTR_X);
    ce   <= '0';
    step <= '1';
    tick;
    step <= '0';
    assert valid = '0' severity failure;
    assert done  = '0' severity failure;
    ce   <= '1';
    tick;

    -- Restart mid-run: reload to x=0
    do_start(HALF_X);
    step_check(false);
    do_start(0);
    for i in 1 to N_DIGITS_TB-1 loop
      step_check(false);
    end loop;
    step_check(true);

    report "TB PASSED: tb_sfix_to_msdf2_stream_r2" severity note;
    wait;
  end process;

end architecture;
