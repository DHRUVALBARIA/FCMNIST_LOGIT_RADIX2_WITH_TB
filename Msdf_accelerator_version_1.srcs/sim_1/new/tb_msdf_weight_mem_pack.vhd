library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

entity tb_msdf_weight_mem_pack is
end entity;

architecture tb of tb_msdf_weight_mem_pack is

  constant CLK_PERIOD : time := 10 ns;

  function clog2(n : positive) return natural is
    variable v : natural := 1;
    variable r : natural := 0;
  begin
    while v < n loop
      v := v * 2;
      r := r + 1;
    end loop;
    return r;
  end function;

  constant NEUR_BITS_C : natural := clog2(N_OUT);
  constant TILE_BITS_C : natural := clog2(N_TILES);

  signal clk   : std_logic := '0';
  signal rst_n : std_logic := '0';
  signal ce    : std_logic := '1';

  signal rd_en     : std_logic := '0';
  signal rd_neuron : unsigned(NEUR_BITS_C-1 downto 0) := (others => '0');
  signal rd_tile   : unsigned(TILE_BITS_C-1 downto 0) := (others => '0');
  signal rd_valid  : std_logic;
  signal w_tile_o  : w_fixed_vec_t;

  signal wr_en     : std_logic := '0';
  signal wr_neuron : unsigned(NEUR_BITS_C-1 downto 0) := (others => '0');
  signal wr_tile   : unsigned(TILE_BITS_C-1 downto 0) := (others => '0');
  signal w_tile_i  : w_fixed_vec_t := (others => (others => '0'));

begin

  clk <= not clk after CLK_PERIOD/2;

  dut : entity work.msdf_weight_mem
    port map (
      clk       => clk,
      rst_n     => rst_n,
      ce        => ce,

      rd_en     => rd_en,
      rd_neuron => rd_neuron,
      rd_tile   => rd_tile,
      rd_valid  => rd_valid,
      w_tile_o  => w_tile_o,

      wr_en     => wr_en,
      wr_neuron => wr_neuron,
      wr_tile   => wr_tile,
      w_tile_i  => w_tile_i
    );

  stim : process
    procedure tick is
    begin
      wait until rising_edge(clk);
      wait for 1 ns;
    end procedure;

    variable exp : w_fixed_vec_t;
  begin
    -- Reset
    rst_n <= '0';
    rd_en <= '0';
    wr_en <= '0';
    tick;
    tick;
    rst_n <= '1';
    tick;

    -- Write one location (j=3, t=7)
    wr_neuron <= to_unsigned(3, NEUR_BITS_C);
    wr_tile   <= to_unsigned(7, TILE_BITS_C);

    for i in 0 to TILE_LEN-1 loop
      exp(i) := to_signed(1000 + i, W_TOTAL_BITS);
      w_tile_i(i) <= exp(i);
    end loop;

    wr_en <= '1';
    tick;               -- write occurs here
    wr_en <= '0';
    tick;

    -- Read back same location (synchronous read)
    rd_neuron <= to_unsigned(3, NEUR_BITS_C);
    rd_tile   <= to_unsigned(7, TILE_BITS_C);

    rd_en <= '1';
    tick;               -- read + rd_valid asserted here (after this edge)
    rd_en <= '0';

    -- Check in the correct cycle: immediately after the read edge
    assert rd_valid = '1'
      report "TB ERROR(weight_mem_pack): rd_valid did not assert in read cycle"
      severity failure;

    for i in 0 to TILE_LEN-1 loop
      assert w_tile_o(i) = exp(i)
        report "TB ERROR(weight_mem_pack): lane mismatch i=" & integer'image(i) &
               " got=" & integer'image(to_integer(w_tile_o(i))) &
               " exp=" & integer'image(to_integer(exp(i)))
        severity failure;
    end loop;

    -- Optional: confirm rd_valid drops next cycle when rd_en=0
    tick;
    assert rd_valid = '0'
      report "TB ERROR(weight_mem_pack): rd_valid did not deassert after read pulse"
      severity failure;

    report "TB PASSED: tb_msdf_weight_mem_pack" severity note;
    wait;
  end process;

end architecture;
