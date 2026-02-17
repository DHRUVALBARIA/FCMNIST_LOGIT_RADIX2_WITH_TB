library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

-- ============================================================================
-- File  : msdf_mul_array_tile.vhd
--
-- Phase 3.1:
--   TILE_LEN parallel multiplier lanes (serial-parallel online multiply)
--
-- Uses fixed-size ports based on TILE_LEN in msdf_accel_pkg.
--
-- Contract:
--   . step_en is COMBINATIONAL and valid before the rising edge
--   . each lane wrapper converts step_en -> in_valid for the core
--   . p_valid_out(i) is a REGISTERED pulse from the core (post-edge)
-- ============================================================================

entity msdf_mul_array_tile is
  port (
    clk     : in  std_logic;
    rst_n   : in  std_logic;
    ce      : in  std_logic;

    step_en : in  std_logic;

    x_digits_in : in  sd_digit_vec_t;
    w_weights_in: in  w_fixed_vec_t;

    p_digits_out : out sd_digit_vec_t;
    p_valid_out  : out sl_vec_t
  );
end entity;

architecture rtl of msdf_mul_array_tile is
begin

  gen_lanes : for i in 0 to TILE_LEN-1 generate
  begin
    u_lane : entity work.msdf_mul_lane_wrap_r2
      port map (
        clk         => clk,
        rst_n       => rst_n,
        ce          => ce,

        step_en     => step_en,
        x_digit_in  => x_digits_in(i),
        y_weight    => w_weights_in(i),

        p_digit_out => p_digits_out(i),
        p_valid_out => p_valid_out(i)
      );
  end generate;

end architecture;
