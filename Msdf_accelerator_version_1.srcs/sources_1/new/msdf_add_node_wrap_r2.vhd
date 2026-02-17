library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all; 

-- ============================================================================
-- File  : msdf_add_node_wrap_r2.vhd
--
-- Purpose:
--   Adapter wrapper for the verified online adder core:
--     new_msdf_add_core_seq_r2_dbg_clean
--
-- Why this exists in Phase 3:
--   Phase 3 will build an adder tree. To keep integration clean and avoid
--   repeating handshake logic, every node uses the same contract:
--
--   . step_en is a combinational enable that is TRUE *before* the rising edge
--   . when step_en='1' and ce='1', this node consumes x/y digits and updates
--   . z_valid_out pulses only when the core produces a real output digit
--
-- Contract:
--   . step_en should already include "accepted step" conditions (in_ready etc.)
--     step_en is NOT a registered pulse.
--
-- Notes:
--   . Debug ports are tied off (open) to keep datapath modules clean.
--   . DELTA_G is locked to ONLINE_DELAY_ADD from msdf_mul_pkg_r2_core.
-- ============================================================================

entity msdf_add_node_wrap_r2 is
  port (
    clk        : in  std_logic;
    rst_n      : in  std_logic;
    ce         : in  std_logic;

    step_en    : in  std_logic;   -- combinational "this step will be accepted"
    x_digit_in : in  sd_digit_t;  -- online digit stream A
    y_digit_in : in  sd_digit_t;  -- online digit stream B

    z_digit_out : out sd_digit_t; -- online sum digit stream
    z_valid_out : out std_logic   -- pulse when z_digit_out is valid
  );
end entity;

architecture rtl of msdf_add_node_wrap_r2 is
  signal in_valid_s : std_logic := '0';
  signal z_d_s      : sd_digit_t := SD_ZERO;
  signal z_v_s      : std_logic := '0';
begin

  -- Convert Phase-3 accepted-step contract to the core's in_valid.
  -- Core gates internally using (ce and in_valid).
  in_valid_s <= step_en;

  u_add : entity work.new_msdf_add_core_seq_r2_dbg_clean
    generic map (
      DELTA_G => ONLINE_DELAY_ADD
    )
    port map (
      clk        => clk,
      rst_n      => rst_n,
      ce         => ce,

      x_digit_in => x_digit_in,
      y_digit_in => y_digit_in,
      in_valid   => in_valid_s,

      z_digit_out => z_d_s,
      out_valid   => z_v_s,

      -- debug taps tied off in accelerator datapath
      dbg_init_mode => open,
      dbg_sum_xy    => open,
      dbg_p_reg     => open
    );

  z_digit_out <= z_d_s;
  z_valid_out <= z_v_s;

end architecture;
