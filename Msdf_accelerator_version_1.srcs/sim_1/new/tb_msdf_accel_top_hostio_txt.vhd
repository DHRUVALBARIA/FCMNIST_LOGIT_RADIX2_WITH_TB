-- ============================================================
-- File:    tb_msdf_accel_top_hostio_txt.vhd
-- Purpose: Pre-AXI Phase-5 top test (host emulation using txt vectors)
--
-- Clean stop behavior:
-- . Uses std.env.stop (VHDL-2008) so PASS does not appear as "Failure".
-- If your project is set to VHDL-2002, enable VHDL-2008 for simulation,
-- or replace std.env.stop with an assert severity failure.
-- ============================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use std.textio.all;

library std;
use std.env.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

entity tb_msdf_accel_top_hostio_txt is
  generic (
    -- Set absolute path for robustness
    G_VEC_DIR       : string  := "/home/bad32576/Serial_Parallel_Mul/Msdf_accelerator_version_1";
    G_N_CASES         : natural := 1000;
    G_BOUND_FINAL     : integer := 10;

    G_CHECK_CYCLES    : boolean := true;
    G_EXPECT_CYCLES   : natural := 62761;

    G_PROGRESS_EVERY  : natural := 50;

    G_CLK_PERIOD      : time    := 10 ns
  );
end entity;

architecture tb of tb_msdf_accel_top_hostio_txt is

  constant ACC_BITS_C  : positive := 40;
  constant BIAS_BITS_C : positive := 16;
  constant FLUSH_EXTRA_C : integer := 2;

  signal clk   : std_logic := '0';
  signal rst_n : std_logic := '0';
  signal ce    : std_logic := '1';

  signal start      : std_logic := '0';
  signal clear_done : std_logic := '0';

  signal busy : std_logic;
  signal done : std_logic;

  signal actv_wr_en   : std_logic := '0';
  signal actv_wr_addr : addr_idx_t := (others => '0');
  signal actv_wr_data : std_logic_vector(31 downto 0) := (others => '0');

  signal bias_wr_en   : std_logic := '0';
  signal bias_wr_addr : neur_idx_t := (others => '0');
  signal bias_wr_data : std_logic_vector(31 downto 0) := (others => '0');

  signal w_wr_en     : std_logic := '0';
  signal w_wr_neuron : neur_idx_t := (others => '0');
  signal w_wr_tile   : tile_idx_t := (others => '0');
  signal w_wr_tile_i : w_fixed_vec_t := (others => (others => '0'));

  signal cycles_total : unsigned(31 downto 0);
  signal logits_pack  : std_logic_vector(N_OUT*ACC_BITS_C-1 downto 0);

  subtype acc_t is signed(ACC_BITS_C-1 downto 0);
  type acc_vec_t is array (0 to N_OUT-1) of acc_t;

  function slv32_from_s16(i : integer) return std_logic_vector is
    variable s16 : signed(15 downto 0);
    variable s32 : signed(31 downto 0);
  begin
    s16 := to_signed(i, 16);
    s32 := resize(s16, 32);
    return std_logic_vector(s32);
  end function;

  function unpack_logits(p : std_logic_vector) return acc_vec_t is
    variable v  : acc_vec_t := (others => (others => '0'));
    variable lo : integer;
    variable hi : integer;
  begin
    for j in 0 to N_OUT-1 loop
      lo := j*ACC_BITS_C;
      hi := (j+1)*ACC_BITS_C - 1;
      v(j) := signed(p(hi downto lo));
    end loop;
    return v;
  end function;

begin

  clk <= not clk after G_CLK_PERIOD/2;

  dut : entity work.msdf_accel_top
    generic map (
      ACC_BITS_G    => ACC_BITS_C,
      BIAS_BITS_G   => BIAS_BITS_C,
      FLUSH_EXTRA_G => FLUSH_EXTRA_C,
      SOFT_RESET_G  => true
    )
    port map (
      clk   => clk,
      rst_n => rst_n,
      ce    => ce,

      start      => start,
      clear_done => clear_done,

      busy => busy,
      done => done,

      actv_wr_en   => actv_wr_en,
      actv_wr_addr => actv_wr_addr,
      actv_wr_data => actv_wr_data,

      bias_wr_en   => bias_wr_en,
      bias_wr_addr => bias_wr_addr,
      bias_wr_data => bias_wr_data,

      w_wr_en     => w_wr_en,
      w_wr_neuron => w_wr_neuron,
      w_wr_tile   => w_wr_tile,
      w_wr_tile_i => w_wr_tile_i,

      cycles_total => cycles_total,
      logits_pack  => logits_pack
    );

  stim : process
    file f_w  : text;
    file f_b  : text;
    file f_x  : text;
    file f_le : text;

    variable st : file_open_status;
    variable L  : line;

    variable w_i  : integer;
    variable b_i  : integer;
    variable x_i  : integer;
    variable le_i : integer;

    variable tilev      : w_fixed_vec_t;
    variable exp_logits : acc_vec_t;
    variable got_logits : acc_vec_t;

    variable err   : integer;
    variable abs_e : integer;

    variable k    : natural;
    variable j    : natural;
    variable t    : natural;
    variable lane : natural;
    variable idx  : natural;

    constant W_FILE  : string := G_VEC_DIR & "/w_fixed.txt";
    constant B_FILE  : string := G_VEC_DIR & "/b_fixed.txt";
    constant X_FILE  : string := G_VEC_DIR & "/x_fixed.txt";
    constant LE_FILE : string := G_VEC_DIR & "/logits_exp.txt";

    constant WATCHDOG_CYC : natural := 250000; -- comfortably > 62761
    variable wd : natural;
  begin
    rst_n <= '0';
    start <= '0';
    clear_done <= '0';
    actv_wr_en <= '0';
    bias_wr_en <= '0';
    w_wr_en <= '0';

    for i in 0 to 5 loop
      wait until rising_edge(clk);
    end loop;

    rst_n <= '1';
    wait until rising_edge(clk);

    report "TB: G_VEC_DIR = " & G_VEC_DIR severity note;
    report "TB: opening  " & W_FILE severity note;
    report "TB: opening  " & B_FILE severity note;
    report "TB: opening  " & X_FILE severity note;
    report "TB: opening  " & LE_FILE severity note;

    file_open(st, f_w,  W_FILE,  read_mode);
    assert st = open_ok report "TB FAIL: cannot open " & W_FILE severity failure;

    file_open(st, f_b,  B_FILE,  read_mode);
    assert st = open_ok report "TB FAIL: cannot open " & B_FILE severity failure;

    file_open(st, f_x,  X_FILE,  read_mode);
    assert st = open_ok report "TB FAIL: cannot open " & X_FILE severity failure;

    file_open(st, f_le, LE_FILE, read_mode);
    assert st = open_ok report "TB FAIL: cannot open " & LE_FILE severity failure;

    -- Load weights once: (j,t,lane)
    for j in 0 to N_OUT-1 loop
      for t in 0 to N_TILES-1 loop
        for lane in 0 to TILE_LEN-1 loop
          readline(f_w, L);
          read(L, w_i);
          tilev(lane) := to_signed(w_i, W_TOTAL_BITS);
        end loop;

        w_wr_neuron <= to_unsigned(j, NEUR_BITS_C);
        w_wr_tile   <= to_unsigned(t, TILE_BITS_C);
        w_wr_tile_i <= tilev;
        w_wr_en     <= '1';
        wait until rising_edge(clk);
        w_wr_en     <= '0';
        wait until rising_edge(clk);
      end loop;
    end loop;

    -- Load biases once: 10 lines
    for j in 0 to N_OUT-1 loop
      readline(f_b, L);
      read(L, b_i);

      bias_wr_addr <= to_unsigned(j, NEUR_BITS_C);
      bias_wr_data <= slv32_from_s16(b_i);
      bias_wr_en   <= '1';
      wait until rising_edge(clk);
      bias_wr_en   <= '0';
      wait until rising_edge(clk);
    end loop;

    for i in 0 to 3 loop
      wait until rising_edge(clk);
    end loop;

    for k in 0 to G_N_CASES-1 loop

      -- Load activations for this case
      for idx in 0 to VEC_LEN-1 loop
        readline(f_x, L);
        read(L, x_i);

        actv_wr_addr <= to_unsigned(idx, ADDR_BITS_C);
        actv_wr_data <= slv32_from_s16(x_i);
        actv_wr_en   <= '1';
        wait until rising_edge(clk);
        actv_wr_en   <= '0';
        wait until rising_edge(clk);
      end loop;

      -- Read expected logits
      for j in 0 to N_OUT-1 loop
        readline(f_le, L);
        read(L, le_i);
        exp_logits(j) := to_signed(le_i, ACC_BITS_C);
      end loop;

      -- Clear done
      clear_done <= '1';
      wait until rising_edge(clk);
      clear_done <= '0';
      wait until rising_edge(clk);

      -- Start
      start <= '1';
      wait until rising_edge(clk);
      start <= '0';

      -- Wait done with watchdog
      wd := 0;
      while done /= '1' loop
        wait until rising_edge(clk);
        wd := wd + 1;
        if wd = WATCHDOG_CYC then
          assert false report "TB FAIL: timeout waiting for done" severity failure;
        end if;
      end loop;

      -- Optional cycles check
      if G_CHECK_CYCLES then
        assert to_integer(cycles_total) = integer(G_EXPECT_CYCLES)
          report "TB FAIL: cycles_total mismatch. got=" &
                 integer'image(to_integer(cycles_total)) &
                 " exp=" & integer'image(integer(G_EXPECT_CYCLES))
          severity failure;
      end if;

      -- Compare logits
      got_logits := unpack_logits(logits_pack);

      for j in 0 to N_OUT-1 loop
        err := to_integer(got_logits(j)) - to_integer(exp_logits(j));
        abs_e := err;
        if abs_e < 0 then abs_e := -abs_e; end if;

        assert abs_e <= G_BOUND_FINAL
          report "TB FAIL: case=" & integer'image(integer(k)) &
                 " j=" & integer'image(integer(j)) &
                 " got=" & integer'image(to_integer(got_logits(j))) &
                 " exp=" & integer'image(to_integer(exp_logits(j))) &
                 " err=" & integer'image(err) &
                 " bound=" & integer'image(G_BOUND_FINAL)
          severity failure;
      end loop;

      if (G_PROGRESS_EVERY /= 0) and ((k mod G_PROGRESS_EVERY) = 0) then
        report "TB: case=" & integer'image(integer(k)) &
               " PASS. cycles=" & integer'image(to_integer(cycles_total))
          severity note;
      end if;

      for i in 0 to 5 loop
        wait until rising_edge(clk);
      end loop;

    end loop;

    file_close(f_w);
    file_close(f_b);
    file_close(f_x);
    file_close(f_le);

    report "TB PASSED: tb_msdf_accel_top_hostio_txt" severity note;
    std.env.stop;
    wait;
  end process;

end architecture;
