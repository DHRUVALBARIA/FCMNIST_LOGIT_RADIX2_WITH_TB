library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;

entity new_msdf_add_core_seq_r2_dbg_clean is
  generic (
    DELTA_G : natural := 2  -- online delay for radix-2 SD add
  );
  port (
    clk        : in  std_logic;
    rst_n      : in  std_logic;
    ce         : in  std_logic;

    x_digit_in : in  sd_digit_t;
    y_digit_in : in  sd_digit_t;
    in_valid   : in  std_logic;

    z_digit_out : out sd_digit_t;
    out_valid   : out std_logic;

    -- debug taps (observation only)
    dbg_init_mode : out std_logic;
    dbg_sum_xy    : out integer;
    dbg_p_reg     : out integer
  );
end entity;

architecture rtl of new_msdf_add_core_seq_r2_dbg_clean is

  -- init counter (counts accepted steps, saturates at DELTA_G)
  signal init_cnt_q : natural range 0 to DELTA_G := 0;
  signal init_mode  : std_logic := '1';

  -- internal state for δ=2 online add
  signal sum1_r : integer range -4 to 4   := 0;     -- x0+y0
  signal p_r    : integer range -64 to 63 := 0;     -- residual/carry-like state

  signal z_out_reg     : sd_digit_t := SD_ZERO;
  signal out_valid_reg : std_logic  := '0';

  -- Debug register so we do not touch input digits when not stepping
  signal dbg_sum_xy_r : integer := 0;

  -- ------------------------------------------------------------
  -- Strict digit decode used ONLY on accepted steps:
  --   - if digit is clean+legal => decode normally
  --   - if digit is invalid at t=0 => warn, return 0
  --   - if digit is invalid at t>0 => failure, return 0
  -- ------------------------------------------------------------
  function sd_to_int_strict_step(d : sd_digit_t) return integer is
    variable i : integer;
  begin
    if is_valid_sd_digit(d) then
      -- safe: this cannot warn because d is clean+legal
      i := sd_to_integer(d);
      -- clamp (defensive; should already be in [-1,1])
      if i > 1 then i := 1; end if;
      if i < -1 then i := -1; end if;
      return i;
    else
      -- synthesis translate_off
      if now = 0 ps then
        assert false
          report "new_msdf_add_core_seq_r2_dbg_clean: meta/illegal SD digit at t=0 on accepted step; treating as 0."
          severity warning;
      else
        assert false
          report "new_msdf_add_core_seq_r2_dbg_clean: meta/illegal SD digit after t=0 on accepted step (REAL BUG)."
          severity failure;
      end if;
      -- synthesis translate_on
      return 0;
    end if;
  end function;

  function int_to_sd_legal(i : integer) return sd_digit_t is
  begin
    if i >= 1 then
      return SD_POS1;
    elsif i <= -1 then
      return SD_NEG1;
    else
      return SD_ZERO;
    end if;
  end function;

begin

  init_mode <= '1' when (init_cnt_q < DELTA_G) else '0';

  process(clk)
    variable step_en : boolean;
    variable xi, yi  : integer;
    variable sxy     : integer;
    variable s       : integer;
    variable z_int   : integer;
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        init_cnt_q     <= 0;
        sum1_r         <= 0;
        p_r            <= 0;
        z_out_reg      <= SD_ZERO;
        out_valid_reg  <= '0';
        dbg_sum_xy_r   <= 0;

      else
        out_valid_reg <= '0';  -- pulse only on qualifying step

        step_en := (ce = '1') and (in_valid = '1');

        if step_en then
          -- STRICT decode only on accepted steps
          xi  := sd_to_int_strict_step(x_digit_in);
          yi  := sd_to_int_strict_step(y_digit_in);
          sxy := xi + yi;

          -- debug: only update when we actually consume inputs
          dbg_sum_xy_r <= sxy;

          -- init counter (saturating)
          if init_cnt_q < DELTA_G then
            init_cnt_q <= init_cnt_q + 1;
          else
            init_cnt_q <= init_cnt_q;
          end if;

          -- δ=2 pipeline:
          -- step 0: capture sum1
          -- step 1: initialize p
          -- step >=2: generate one output digit per step
          if init_cnt_q = 0 then
            sum1_r <= sxy;

          elsif init_cnt_q = 1 then
            p_r <= (2 * sum1_r) + sxy;

          else
            s := (2 * p_r) + sxy;

            if s >= 3 then
              z_int := 1;
            elsif s <= -3 then
              z_int := -1;
            else
              z_int := 0;
            end if;

            p_r <= s - (4 * z_int);

            z_out_reg     <= int_to_sd_legal(z_int);
            out_valid_reg <= '1';
          end if;
        end if;
      end if;
    end if;
  end process;

  z_digit_out <= z_out_reg;
  out_valid   <= out_valid_reg;

  dbg_init_mode <= init_mode;
  dbg_sum_xy    <= dbg_sum_xy_r;  -- safe (registered)
  dbg_p_reg     <= p_r;

end architecture;

