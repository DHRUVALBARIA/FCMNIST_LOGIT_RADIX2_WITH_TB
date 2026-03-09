library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ============================================================================
-- File  : msdf_tile_stepper.vhd
--
-- Purpose:
--   Generates a clean "accepted step" pulse (step_fire) for a tile.
--   Every digit-time action in the tile should key off step_fire.
--
-- Contract:
--   . tile_start : 1-cycle pulse to start (or restart) a tile
--   . in_ready   : downstream indicates it can accept the next step this cycle
--   . step_fire  : 1-cycle pulse when a step is actually accepted
--   . done_digits: pulses exactly when the N_DIGITS-th digit step is accepted
--   . done_tile  : pulses when (N_DIGITS + FLUSH) total accepted steps complete
--
-- Notes:
--   . FLUSH_G lets you include online delay flush cycles later without rewriting
--   . All pulses are cleared every clock edge (even if ce='0') to avoid sticking
-- ============================================================================

entity msdf_tile_stepper is
  generic (
    N_DIGITS_G : integer := 16;
    FLUSH_G    : integer := 0    -- extra accepted steps after digits (for pipeline flush)
  );
  port (
    clk        : in  std_logic;
    rst_n      : in  std_logic;  -- active-low synchronous reset
    ce         : in  std_logic;

    tile_start : in  std_logic;  -- pulse: start/restart tile
    in_ready   : in  std_logic;  -- downstream ready for next step

    active     : out std_logic;  -- high while tile is running
    step_fire  : out std_logic;  -- pulse when a step is accepted

    digit_idx  : out integer range 0 to N_DIGITS_G-1; -- current digit index (valid when active)
    step_count : out integer range 0 to N_DIGITS_G + FLUSH_G; -- accepted steps so far

    done_digits : out std_logic; -- pulse on last digit step
    done_tile   : out std_logic  -- pulse on final step (digits + flush)
  );
end entity;

architecture rtl of msdf_tile_stepper is

  constant TOTAL_STEPS_C : integer := N_DIGITS_G + FLUSH_G;

  type state_t is (IDLE, RUN);
  signal state_r : state_t := IDLE;

  signal cnt_r       : integer range 0 to TOTAL_STEPS_C := 0;
  signal active_r    : std_logic := '0';

  signal step_fire_r  : std_logic := '0';
  signal done_dig_r   : std_logic := '0';
  signal done_tile_r  : std_logic := '0';

  -- digit index is clamped after last digit (so it never goes out of range)
  function clamp_digit_idx(cnt : integer) return integer is
  begin
    if cnt < 0 then
      return 0;
    elsif cnt > (N_DIGITS_G-1) then
      return (N_DIGITS_G-1);
    else
      return cnt;
    end if;
  end function;

begin

  active     <= active_r;
  step_fire  <= step_fire_r;
  done_digits<= done_dig_r;
  done_tile  <= done_tile_r;

  digit_idx  <= clamp_digit_idx(cnt_r);
  step_count <= cnt_r;

  assert N_DIGITS_G >= 1
    report "msdf_tile_stepper: N_DIGITS_G must be >= 1"
    severity failure;

  assert FLUSH_G >= 0
    report "msdf_tile_stepper: FLUSH_G must be >= 0"
    severity failure;

  process(clk)
    variable fire_v : boolean;
  begin
    if rising_edge(clk) then

      -- pulse-safe: clear pulses every clock edge independent of ce
      step_fire_r <= '0';
      done_dig_r  <= '0';
      done_tile_r <= '0';

      if rst_n = '0' then
        state_r  <= IDLE;
        cnt_r    <= 0;
        active_r <= '0';

      else
        if ce = '1' then

          case state_r is

            when IDLE =>
              active_r <= '0';
              cnt_r    <= 0;

              if tile_start = '1' then
                state_r  <= RUN;
                active_r <= '1';
                cnt_r    <= 0;
              end if;

            when RUN =>
              active_r <= '1';

              -- restart mid-run if requested
              if tile_start = '1' then
                cnt_r <= 0;
              else
                fire_v := (in_ready = '1');

                if fire_v then
                  step_fire_r <= '1';

                  -- done_digits is the N_DIGITS-th accepted step (cnt == N_DIGITS-1)
                  if cnt_r = (N_DIGITS_G-1) then
                    done_dig_r <= '1';
                  end if;

                  -- done_tile is the TOTAL_STEPS-th accepted step (cnt == TOTAL_STEPS-1)
                  if (TOTAL_STEPS_C > 0) and (cnt_r = (TOTAL_STEPS_C-1)) then
                    done_tile_r <= '1';
                    state_r     <= IDLE;
                    active_r    <= '0';
                    cnt_r       <= 0;
                  else
                    cnt_r <= cnt_r + 1;
                  end if;
                end if;
              end if;

          end case;

        end if; -- ce
      end if; -- rst
    end if; -- rising_edge
  end process;

end architecture;
