library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

-- ============================================================================
-- File  : msdf_add_tree_r2.vhd
--
-- Purpose:
--   Reduce TILE_LEN online digit streams into one stream using a balanced tree
--   of online adders (msdf_add_node_wrap_r2).
--
-- Contract:
--   . Inputs: in_digits_in(i) with valid pulses in_valid_in(i)
--   . Leaf valids must be aligned (same upstream stepping schedule)
--   . Each internal node steps only when BOTH children valid are '1'
--   . Output: (z_digit_out, z_valid_out) from the root
--
-- Notes:
--   . No buffering for misaligned streams (pure reducer)
--   . Meta-safe step generation: never uses std_logic AND directly
-- ============================================================================

entity msdf_add_tree_r2 is
  port (
    clk   : in  std_logic;
    rst_n : in  std_logic;
    ce    : in  std_logic;

    in_digits_in : in  sd_digit_vec_t;
    in_valid_in  : in  sl_vec_t;

    z_digit_out  : out sd_digit_t;
    z_valid_out  : out std_logic
  );
end entity;

architecture rtl of msdf_add_tree_r2 is

  constant TL : integer := TILE_LEN;

  type sd4_vec_t is array (0 to 3) of sd_digit_t;
  type sl4_vec_t is array (0 to 3) of std_logic;

  type sd2_vec_t is array (0 to 1) of sd_digit_t;
  type sl2_vec_t is array (0 to 1) of std_logic;

  signal s1_d : sd4_vec_t := (others => SD_ZERO);
  signal s1_v : sl4_vec_t := (others => '0');

  signal s2_d : sd2_vec_t := (others => SD_ZERO);
  signal s2_v : sl2_vec_t := (others => '0');

  signal root_d : sd_digit_t := SD_ZERO;
  signal root_v : std_logic  := '0';

  -- meta-safe step signals (computed as '0'/'1' only)
  signal step_s1 : sl4_vec_t := (others => '0');
  signal step_s2 : sl2_vec_t := (others => '0');
  signal step_rt : std_logic := '0';

begin

  z_digit_out <= root_d;
  z_valid_out <= root_v;

  -- synthesis translate_off
  assert (TL = 1) or (TL = 2) or (TL = 4) or (TL = 8)
    report "msdf_add_tree_r2: TILE_LEN must be 1,2,4,8 for this implementation"
    severity failure;
  -- synthesis translate_on

  -- =========================================================================
  -- TL = 1
  -- =========================================================================
  gen_tl1 : if TL = 1 generate
  begin
    root_d <= in_digits_in(0);
    root_v <= '1' when ((ce = '1') and (in_valid_in(0) = '1')) else '0';
  end generate;

  -- =========================================================================
  -- TL = 2
  -- =========================================================================
  gen_tl2 : if TL = 2 generate
  begin
    step_rt <= '1' when ((ce = '1') and (in_valid_in(0) = '1') and (in_valid_in(1) = '1')) else '0';

    u_root : entity work.msdf_add_node_wrap_r2
      port map (
        clk         => clk,
        rst_n       => rst_n,
        ce          => ce,
        step_en     => step_rt,
        x_digit_in  => in_digits_in(0),
        y_digit_in  => in_digits_in(1),
        z_digit_out => root_d,
        z_valid_out => root_v
      );
  end generate;

  -- =========================================================================
  -- TL = 4
  -- =========================================================================
  gen_tl4 : if TL = 4 generate
  begin
    step_s1(0) <= '1' when ((ce = '1') and (in_valid_in(0) = '1') and (in_valid_in(1) = '1')) else '0';
    step_s1(1) <= '1' when ((ce = '1') and (in_valid_in(2) = '1') and (in_valid_in(3) = '1')) else '0';
    step_rt    <= '1' when ((ce = '1') and (s1_v(0) = '1') and (s1_v(1) = '1')) else '0';

    u_s1_0 : entity work.msdf_add_node_wrap_r2
      port map (
        clk         => clk,
        rst_n       => rst_n,
        ce          => ce,
        step_en     => step_s1(0),
        x_digit_in  => in_digits_in(0),
        y_digit_in  => in_digits_in(1),
        z_digit_out => s1_d(0),
        z_valid_out => s1_v(0)
      );

    u_s1_1 : entity work.msdf_add_node_wrap_r2
      port map (
        clk         => clk,
        rst_n       => rst_n,
        ce          => ce,
        step_en     => step_s1(1),
        x_digit_in  => in_digits_in(2),
        y_digit_in  => in_digits_in(3),
        z_digit_out => s1_d(1),
        z_valid_out => s1_v(1)
      );

    u_root : entity work.msdf_add_node_wrap_r2
      port map (
        clk         => clk,
        rst_n       => rst_n,
        ce          => ce,
        step_en     => step_rt,
        x_digit_in  => s1_d(0),
        y_digit_in  => s1_d(1),
        z_digit_out => root_d,
        z_valid_out => root_v
      );
  end generate;

  -- =========================================================================
  -- TL = 8
  -- =========================================================================
  gen_tl8 : if TL = 8 generate
  begin
    -- Stage 1 steps (8 -> 4)
    gen_s1 : for k in 0 to 3 generate
    begin
      step_s1(k) <= '1'
        when ((ce = '1') and (in_valid_in(2*k) = '1') and (in_valid_in(2*k + 1) = '1'))
        else '0';

      u_s1 : entity work.msdf_add_node_wrap_r2
        port map (
          clk         => clk,
          rst_n       => rst_n,
          ce          => ce,
          step_en     => step_s1(k),
          x_digit_in  => in_digits_in(2*k),
          y_digit_in  => in_digits_in(2*k + 1),
          z_digit_out => s1_d(k),
          z_valid_out => s1_v(k)
        );
    end generate;

    -- Stage 2 steps (4 -> 2)
    step_s2(0) <= '1' when ((ce = '1') and (s1_v(0) = '1') and (s1_v(1) = '1')) else '0';
    step_s2(1) <= '1' when ((ce = '1') and (s1_v(2) = '1') and (s1_v(3) = '1')) else '0';

    u_s2_0 : entity work.msdf_add_node_wrap_r2
      port map (
        clk         => clk,
        rst_n       => rst_n,
        ce          => ce,
        step_en     => step_s2(0),
        x_digit_in  => s1_d(0),
        y_digit_in  => s1_d(1),
        z_digit_out => s2_d(0),
        z_valid_out => s2_v(0)
      );

    u_s2_1 : entity work.msdf_add_node_wrap_r2
      port map (
        clk         => clk,
        rst_n       => rst_n,
        ce          => ce,
        step_en     => step_s2(1),
        x_digit_in  => s1_d(2),
        y_digit_in  => s1_d(3),
        z_digit_out => s2_d(1),
        z_valid_out => s2_v(1)
      );

    -- Stage 3 step (2 -> 1)
    step_rt <= '1' when ((ce = '1') and (s2_v(0) = '1') and (s2_v(1) = '1')) else '0';

    u_root : entity work.msdf_add_node_wrap_r2
      port map (
        clk         => clk,
        rst_n       => rst_n,
        ce          => ce,
        step_en     => step_rt,
        x_digit_in  => s2_d(0),
        y_digit_in  => s2_d(1),
        z_digit_out => root_d,
        z_valid_out => root_v
      );
  end generate;

end architecture;
