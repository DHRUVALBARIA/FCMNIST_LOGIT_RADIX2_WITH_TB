library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

-- ============================================================================
-- File  : msdf_tile_sop.vhd
--
-- Phase 3.2 (Tile SOP Wrapper)
--
-- Purpose:
--   Compute one tile's online sum-of-products:
--     sum_{i=0..TILE_LEN-1} ( x[i] * w[i] )
--   as an MSDF digit stream (sd_digit_t + pulse-valid).
--
-- Core idea:
--   . A tile stepper defines "accepted steps" under stall/backpressure.
--   . Each lane digit-generator turns x_fixed -> MSDF digits.
--   . A multiplier lane array multiplies streamed x digits with static weights.
--   . An adder tree reduces TILE_LEN product streams to a single sum stream.
--
-- Handshake / stall rule:
--   . Stalling is done by pausing the accepted-step (in_ready=0).
--   . Do NOT rely on random ce gating for pulse-valid streams.
--
-- Restart / multi-tile rule:
--   . The online multiplier/adder cores are stateful.
--   . This wrapper applies a 1-cycle "soft reset" to mul+add datapath on tile_start,
--     so back-to-back tiles can run without global reset.
--   . Digit generators already support restart via start.
--
-- Output stream convention:
--   . This block does NOT reconstruct or apply SHIFT compensation.
--   . Reconstruction is done outside using msdf_tile_recon_r2 with SHIFT_LEFT_G=0
--     (matching your passing add-tree TB conclusion).
-- ============================================================================

entity msdf_tile_sop is
  generic (
    -- Extra flush steps beyond the computed pipeline drain. 
    -- Increase if you ever see: "tile sum recon did not complete" in the tile TB.
    FLUSH_EXTRA_G  : integer := 4;

    -- Soft-reset mul/add cores for 1 cycle on tile_start (recommended).
    SOFT_RESET_G   : boolean := true
  );
  port (
    clk   : in  std_logic;
    rst_n : in  std_logic;   -- active-low synchronous reset
    ce    : in  std_logic;

    -- Tile control
    tile_start : in  std_logic; -- 1-cycle pulse: start/restart tile
    in_ready   : in  std_logic; -- when 1, accept a step (stall when 0)

    -- Tile data
    x_tile_in  : in  w_fixed_vec_t; -- TILE_LEN activations (fixed-point)
    w_tile_in  : in  w_fixed_vec_t; -- TILE_LEN weights     (fixed-point)

    -- Status
    active     : out std_logic; -- high while tile is running (digits + flush)
    done_tile  : out std_logic; -- 1-cycle pulse when tile finished (after flush)

    -- Online sum stream output
    sum_digit_out : out sd_digit_t;
    sum_valid_out : out std_logic
  );
end entity;

architecture rtl of msdf_tile_sop is

  -- ilog2 for TILE_LEN in {1,2,4,8}
  function ilog2_pow2(n : positive) return natural is
    variable v : natural := 1;
    variable d : natural := 0;
  begin
    while v < natural(n) loop
      v := v * 2;
      d := d + 1;
    end loop;
    return d;
  end function;

  constant TREE_DEPTH_C : integer := integer(ilog2_pow2(TILE_LEN));

  -- Conservative drain estimate:
  -- . ONLINE_DELAY_MUL steps to drain multiplier online delay
  -- . ONLINE_DELAY_ADD * TREE_DEPTH_C steps to drain add-tree online delays
  -- . + FLUSH_EXTRA_G guard
  constant FLUSH_C : integer :=
      ONLINE_DELAY_MUL +
      (ONLINE_DELAY_ADD * TREE_DEPTH_C) +
      FLUSH_EXTRA_G;

  -- Stepper outputs
  signal active_s      : std_logic := '0';
  signal done_tile_s   : std_logic := '0';
  signal step_fire_s   : std_logic := '0'; -- post-edge pulse (debug/counting)

  -- Pre-edge "accepted step" enable (matches your integration TB style)
  signal step_en_pre   : std_logic := '0';

  -- 1-cycle delayed accepted-step enable for multiplier stepping
  signal step_en_d1    : std_logic := '0';

  -- Optional soft reset for datapath cores
  signal rst_dp_n      : std_logic := '1';

  -- Per-lane digit generator outputs (registered digits + pulse-valid)
  signal x_d  : sd_digit_vec_t := (others => SD_ZERO);
  signal x_v  : sl_vec_t       := (others => '0');
  signal x_dn : sl_vec_t       := (others => '0'); -- done pulses (unused)

  -- Multiplier array outputs
  signal p_d  : sd_digit_vec_t := (others => SD_ZERO);
  signal p_v  : sl_vec_t       := (others => '0');

  -- Add-tree output
  signal z_d  : sd_digit_t := SD_ZERO;
  signal z_v  : std_logic  := '0';

begin

  -- Outputs
  active        <= active_s;
  done_tile     <= done_tile_s;
  sum_digit_out <= z_d;
  sum_valid_out <= z_v;

  -- Synthesis-time sanity
  -- synthesis translate_off
  assert (TILE_LEN = 1) or (TILE_LEN = 2) or (TILE_LEN = 4) or (TILE_LEN = 8)
    report "msdf_tile_sop: TILE_LEN must be 1,2,4,8"
    severity failure;

  assert FLUSH_EXTRA_G >= 0
    report "msdf_tile_sop: FLUSH_EXTRA_G must be >= 0"
    severity failure;
  -- synthesis translate_on

  -- Soft reset only for mul/add datapath (not for generators)
  rst_dp_n <= rst_n when (SOFT_RESET_G = false) else (rst_n and (not tile_start));

  -- Pre-edge accepted-step enable:
  -- When active and in_ready, the step for THIS cycle is accepted at the upcoming edge.
  step_en_pre <= '1' when ((ce = '1') and (active_s = '1') and (in_ready = '1')) else '0';

  -- Delay accepted-step by 1 cycle for the multiplier array:
  -- The digit generators update digit_out at the edge when step_en_pre is 1.
  -- The multiplier must consume that digit on the NEXT edge, so we use step_en_d1.
  process(clk)
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        step_en_d1 <= '0';
      elsif ce = '1' then
        if tile_start = '1' then
          step_en_d1 <= '0';
        else
          step_en_d1 <= step_en_pre;
        end if;
      end if;
    end if;
  end process;

  -- =========================================================================
  -- Tile stepper (digits + flush)
  -- =========================================================================
  u_stepper : entity work.msdf_tile_stepper
    generic map (
      N_DIGITS_G => N_DIGITS,
      FLUSH_G    => FLUSH_C
    )
    port map (
      clk         => clk,
      rst_n       => rst_n,
      ce          => ce,

      tile_start  => tile_start,
      in_ready    => in_ready,

      active      => active_s,
      step_fire   => step_fire_s,  -- post-edge pulse
      digit_idx   => open,
      step_count  => open,

      done_digits => open,
      done_tile   => done_tile_s
    );

  -- =========================================================================
  -- TILE_LEN digit generators: x_fixed -> MSDF digits
  -- =========================================================================
  gen_gens : for i in 0 to TILE_LEN-1 generate
  begin
    u_gen : entity work.sfix_to_msdf2_stream_r2
      generic map (
        N_DIGITS_G => N_DIGITS
      )
      port map (
        clk         => clk,
        rst_n       => rst_n,
        ce          => ce,

        start       => tile_start,
        step        => step_en_pre,     -- accepted-step (pre-edge)

        x_in        => x_tile_in(i),

        digit_out   => x_d(i),
        valid       => x_v(i),
        done        => x_dn(i),
        busy        => open,
        digit_count => open
      );
  end generate;

  -- =========================================================================
  -- Multiplier lane array
  --
  -- Stepped by step_en_d1 so it consumes the digit produced by generators
  -- on the previous accepted step.
  -- =========================================================================
  u_mul_arr : entity work.msdf_mul_array_tile
    port map (
      clk          => clk,
      rst_n        => rst_dp_n,       -- soft-reset datapath on tile_start
      ce           => ce,

      step_en      => step_en_d1,
      x_digits_in  => x_d,            -- digits held stable in cycle after generation
      w_weights_in => w_tile_in,

      p_digits_out => p_d,
      p_valid_out  => p_v
    );

  -- =========================================================================
  -- Adder tree reducer (TILE_LEN -> 1 stream)
  -- =========================================================================
  u_add_tree : entity work.msdf_add_tree_r2
    port map (
      clk          => clk,
      rst_n        => rst_dp_n,      -- soft-reset datapath on tile_start
      ce           => ce,

      in_digits_in => p_d,
      in_valid_in  => p_v,

      z_digit_out  => z_d,
      z_valid_out  => z_v
    );

end architecture;
