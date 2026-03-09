library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

entity tb_msdf_actv_buf_read is
end entity;

architecture tb of tb_msdf_actv_buf_read is

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

  constant ADDR_BITS_C : natural := clog2(VEC_LEN);

  signal clk   : std_logic := '0';
  signal rst_n : std_logic := '0';
  signal ce    : std_logic := '1';

  signal wr_en   : std_logic := '0';
  signal wr_addr : unsigned(ADDR_BITS_C-1 downto 0) := (others => '0');
  signal x_i     : w_fixed_t := (others => '0');

  signal rd_en    : std_logic := '0';
  signal rd_addr  : unsigned(ADDR_BITS_C-1 downto 0) := (others => '0');
  signal rd_valid : std_logic;
  signal x_o      : w_fixed_t;

begin

  clk <= not clk after CLK_PERIOD/2;

  dut : entity work.msdf_actv_buf
    port map (
      clk      => clk,
      rst_n    => rst_n,
      ce       => ce,

      wr_en    => wr_en,
      wr_addr  => wr_addr,
      x_i      => x_i,

      rd_en    => rd_en,
      rd_addr  => rd_addr,
      rd_valid => rd_valid,
      x_o      => x_o
    );

  stim : process
    procedure tick is
    begin
      wait until rising_edge(clk);
      wait for 1 ns;
    end procedure;

    procedure do_reset is
    begin
      rst_n <= '0';
      ce    <= '1';
      wr_en <= '0';
      rd_en <= '0';
      tick;
      tick;
      rst_n <= '1';
      tick;
    end procedure;

    procedure wr_one(addr_i : natural; val_i : integer) is
    begin
      wr_addr <= to_unsigned(addr_i, ADDR_BITS_C);
      x_i     <= to_signed(val_i, W_TOTAL_BITS);
      wr_en   <= '1';
      tick;              -- write happens here
      wr_en   <= '0';
      tick;
    end procedure;

    procedure rd_one_check(addr_i : natural; exp_i : integer) is
    begin
      rd_addr <= to_unsigned(addr_i, ADDR_BITS_C);
      rd_en   <= '1';
      tick;              -- read happens here, rd_valid asserted here
      rd_en   <= '0';

      assert rd_valid = '1'
        report "TB ERROR(actv_buf): rd_valid did not assert in read cycle. addr=" &
               integer'image(integer(addr_i))
        severity failure;

      assert x_o = to_signed(exp_i, W_TOTAL_BITS)
        report "TB ERROR(actv_buf): data mismatch. addr=" & integer'image(integer(addr_i)) &
               " got=" & integer'image(to_integer(x_o)) &
               " exp=" & integer'image(exp_i)
        severity failure;

      -- next cycle: rd_valid must drop if rd_en is 0
      tick;
      assert rd_valid = '0'
        report "TB ERROR(actv_buf): rd_valid did not deassert after read pulse. addr=" &
               integer'image(integer(addr_i))
        severity failure;
    end procedure;

    procedure tile_read8_check(base_addr : natural; exp_base : integer) is
      variable a : natural;
      variable e : integer;
    begin
      -- Mimic Phase 4.2: read 8 consecutive activations
      for k in 0 to TILE_LEN-1 loop
        a := base_addr + natural(k);
        e := exp_base + integer(k);
        rd_one_check(a, e);
      end loop;
    end procedure;

    constant LAST_ADDR_C : natural := natural(VEC_LEN - 1);
    constant TILE_BASE_C : natural := 16;  -- any base where base+7 < VEC_LEN

  begin
    do_reset;

    -- ------------------------------------------------------------
    -- Basic write/read at edges and a middle point
    -- ------------------------------------------------------------
    wr_one(0, 111);
    wr_one(7, -222);
    wr_one(LAST_ADDR_C, 333);

    rd_one_check(0, 111);
    rd_one_check(7, -222);
    rd_one_check(LAST_ADDR_C, 333);

    -- ------------------------------------------------------------
    -- Tile-style pattern: write 8 consecutive words and read back
    -- Lane k at address (base+k) = (1000 + k)
    -- ------------------------------------------------------------
    for k in 0 to TILE_LEN-1 loop
      wr_one(TILE_BASE_C + natural(k), 1000 + integer(k));
    end loop;

    tile_read8_check(TILE_BASE_C, 1000);

    -- ------------------------------------------------------------
    -- Optional: verify CE gating (no progress when ce=0)
    -- ------------------------------------------------------------
    ce <= '0';
    rd_addr <= to_unsigned(0, ADDR_BITS_C);
    rd_en   <= '1';
    tick;  -- ce=0, so rd_valid_r should NOT update
    rd_en   <= '0';

    assert rd_valid = '0'
      report "TB ERROR(actv_buf): rd_valid changed while ce=0"
      severity failure;

    ce <= '1';
    tick;

    report "TB PASSED: tb_msdf_actv_buf_read" severity note;
    wait;
  end process;

end architecture;
