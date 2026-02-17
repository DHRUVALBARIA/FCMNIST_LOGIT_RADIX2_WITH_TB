library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;

package msdf_accel_pkg is

  -- ==========================================================================
  -- Index sizing helper (synthesizable)
  -- returns smallest r such that 2^r >= n
  -- ==========================================================================
  function clog2_p(n : positive) return natural; 

  -- ==========================================================================
  -- Locked demo scope
  -- ==========================================================================
  constant VEC_LEN  : integer := 784;
  constant N_OUT    : integer := 10;

  -- ==========================================================================
  -- Locked precision knobs (Phase 1 decisions)
  -- ==========================================================================
  constant N_DIGITS : integer := 18;
  constant TILE_LEN : integer := 8;

  -- ==========================================================================
  -- Derived
  -- ==========================================================================
  constant N_TILES  : integer := VEC_LEN / TILE_LEN;

  -- Reconstruction internal prefix width
  constant CA_BITS  : integer := 32;

  -- Tile recon output width (bigger than w_fixed_t on purpose)
  constant TILE_RECON_BITS : integer := W_TOTAL_BITS + 10;
  subtype  tile_recon_t    is signed(TILE_RECON_BITS-1 downto 0);

  -- ==========================================================================
  -- Fixed-size accelerator vectors (Vivado/IP friendly)
  -- ==========================================================================
  type sd_digit_vec_t is array (0 to TILE_LEN-1) of sd_digit_t;
  type w_fixed_vec_t  is array (0 to TILE_LEN-1) of w_fixed_t;
  type sl_vec_t       is array (0 to TILE_LEN-1) of std_logic;

  -- ==========================================================================
  -- Canonical index widths and subtypes
  -- ==========================================================================
  constant NEUR_BITS_C : natural := clog2_p(N_OUT);
  constant TILE_BITS_C : natural := clog2_p(N_TILES);
  constant ADDR_BITS_C : natural := clog2_p(VEC_LEN);
  constant LANE_BITS_C : natural := clog2_p(TILE_LEN);

  subtype neur_idx_t is unsigned(NEUR_BITS_C-1 downto 0);
  subtype tile_idx_t is unsigned(TILE_BITS_C-1 downto 0);
  subtype addr_idx_t is unsigned(ADDR_BITS_C-1 downto 0);
  subtype lane_idx_t is unsigned(LANE_BITS_C-1 downto 0);

end package;


package body msdf_accel_pkg is

  function clog2_p(n : positive) return natural is
    variable v : natural := 1;
    variable r : natural := 0;
  begin
    while v < natural(n) loop
      v := v * 2;
      r := r + 1;
    end loop;
    return r;
  end function;

end package body;
