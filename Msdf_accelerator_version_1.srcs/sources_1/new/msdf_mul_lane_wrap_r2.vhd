library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;

-- ============================================================================
-- File  : msdf_mul_lane_wrap_r2.vhd
--
-- Purpose:
--   Adapter wrapper for the verified online multiplier core:
--     new_msdf_core_seq_r2_dbg_clean
--
-- Why we need this in Phase 3:
--   Phase 3 will build arrays/trees. To keep integration clean, every block
--   should share the same "accepted-step" contract:
--
--     . step_en is a combinational enable that is TRUE *before* the rising edge
--     . when step_en='1' and ce='1', this lane consumes x_digit_in and updates
--     . p_valid_out pulses only when the core produces a real output digit
--
-- Contract:
--   . step_en should already include downstream acceptance (like in_ready),
--     meaning: step_en = (ce and active and in_ready ...) computed outside.
--
-- Notes:
--   . This wrapper ties off debug ports (open) so later Phase 3 modules are tidy.
-- ============================================================================

entity msdf_mul_lane_wrap_r2 is
  port (
    clk        : in  std_logic;
    rst_n      : in  std_logic;
    ce         : in  std_logic;

    step_en    : in  std_logic;   -- combinational "this step will be accepted"
    x_digit_in : in  sd_digit_t;  -- streamed MSDF digit input
    y_weight   : in  w_fixed_t;   -- static fixed-point weight for this lane

    p_digit_out : out sd_digit_t; -- product digit output stream
    p_valid_out : out std_logic   -- pulse when p_digit_out is valid
  );
end entity;

architecture rtl of msdf_mul_lane_wrap_r2 is
  signal in_valid_s : std_logic := '0';
  signal p_d_s      : sd_digit_t := SD_ZERO;
  signal p_v_s      : std_logic := '0';
begin

  -- Convert the Phase-3 contract to the core's in_valid.
  -- Core already gates with (ce and in_valid) internally.
  in_valid_s <= step_en;

  u_mul : entity work.new_msdf_core_seq_r2_dbg_clean
    port map (
      clk         => clk,
      rst_n       => rst_n,
      ce          => ce,

      x_digit_in  => x_digit_in,
      y_vector_in => y_weight,
      in_valid    => in_valid_s,

      p_digit_out => p_d_s,
      out_valid   => p_v_s,

      -- debug taps tied off in accelerator datapath
      dbg_w_reg     => open,
      dbg_w_next    => open,
      dbg_h1_term   => open,
      dbg_v_term    => open,
      dbg_v_hat     => open,
      dbg_p_raw     => open,
      dbg_init_mode => open
    );

  p_digit_out <= p_d_s;
  p_valid_out <= p_v_s;

end architecture;
