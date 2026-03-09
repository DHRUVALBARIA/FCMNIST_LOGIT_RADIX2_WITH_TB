-- ============================================================================
-- tb/tb_msdf_fc_784x10.vhd  (VHDL-2002 safe, Vivado/XSim friendly)
--
-- TextIO integer files:
--   w_fixed.txt    : N_OUT*VEC_LEN lines, order: j major, then i minor
--   b_fixed.txt    : N_OUT lines, order: j
--   x_fixed.txt    : N_CASES*VEC_LEN lines, order: case major, then i
--   logits_exp.txt : N_CASES*N_OUT lines, order: case major, then j
--
-- Important:
--   Running only "1000ns" is usually too short. Use "run all" or "run 5 ms".
-- Failed tb please refer the next one--- 
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

entity tb_msdf_fc_784x10 is
end entity;

architecture tb of tb_msdf_fc_784x10 is

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
  constant ADDR_BITS_C : natural := clog2(VEC_LEN);

  constant ACC_BITS_C  : positive := 40;
  constant BIAS_BITS_C : positive := 16;

  constant N_CASES_TB  : integer := 5;

  -- Watchdog must be large: FC run is long (tile engine + 8x activation reads per tile).
  constant WATCHDOG_CYCLES : integer := 20000000;

--  constant FILE_W  : string := "w_fixed.txt";
--  constant FILE_B  : string := "b_fixed.txt";
--  constant FILE_X  : string := "x_fixed.txt";
--  constant FILE_LE : string := "logits_exp.txt";
  
  -- Absolute paths to force XSim to find the files
  constant FILE_W  : string := "/home/bad32576/Serial_Parallel_Mul/Msdf_accelerator_version_1/w_fixed_2.txt";
  constant FILE_B  : string := "/home/bad32576/Serial_Parallel_Mul/Msdf_accelerator_version_1/b_fixed_2.txt";
  constant FILE_X  : string := "/home/bad32576/Serial_Parallel_Mul/Msdf_accelerator_version_1/x_fixed_2.txt";
  constant FILE_LE : string := "/home/bad32576/Serial_Parallel_Mul/Msdf_accelerator_version_1/logits_exp_2.txt";

  -- Portable integer array type (avoids integer_vector portability issues)
  type int_vec_t is array (natural range <>) of integer;

  -- Vivado sometimes rejects "(others=>'1')" directly in "=" comparisons.
  constant VMASK_ALL1_C : std_logic_vector(N_OUT-1 downto 0) := (others => '1');

  signal clk   : std_logic := '0';
  signal rst_n : std_logic := '0';
  signal ce    : std_logic := '1';

  -- Activation buffer
  signal actv_wr_en   : std_logic := '0';
  signal actv_wr_addr : unsigned(ADDR_BITS_C-1 downto 0) := (others => '0');
  signal actv_x_i     : w_fixed_t := (others => '0');

  signal actv_rd_en   : std_logic := '0';
  signal actv_rd_addr : unsigned(ADDR_BITS_C-1 downto 0) := (others => '0');
  signal actv_rd_v    : std_logic;
  signal actv_x_o     : w_fixed_t;

  -- Weight memory
  signal w_wr_en      : std_logic := '0';
  signal w_wr_neuron  : unsigned(NEUR_BITS_C-1 downto 0) := (others => '0');
  signal w_wr_tile    : unsigned(TILE_BITS_C-1 downto 0) := (others => '0');
  signal w_tile_i     : w_fixed_vec_t := (others => (others => '0'));

  signal w_rd_en      : std_logic := '0';
  signal w_rd_neuron  : unsigned(NEUR_BITS_C-1 downto 0) := (others => '0');
  signal w_rd_tile    : unsigned(TILE_BITS_C-1 downto 0) := (others => '0');
  signal w_rd_v       : std_logic;
  signal w_tile_o     : w_fixed_vec_t;

  -- Bias storage in TB, muxed by bias_sel
  type bias_arr_t is array (0 to N_OUT-1) of signed(BIAS_BITS_C-1 downto 0);
  signal bias_mem   : bias_arr_t := (others => (others => '0'));
  signal bias_sel_s : unsigned(NEUR_BITS_C-1 downto 0) := (others => '0');
  signal bias_in_s  : signed(BIAS_BITS_C-1 downto 0) := (others => '0');

  -- Controller
  signal start_s : std_logic := '0';
  signal busy_s  : std_logic;
  signal done_s  : std_logic;

  signal logit_we_s   : std_logic;
  signal logit_idx_s  : unsigned(NEUR_BITS_C-1 downto 0);
  signal logit_data_s : signed(ACC_BITS_C-1 downto 0);

  signal cycles_total_s : unsigned(31 downto 0);

  -- Logits regs
  signal regs_clear_s   : std_logic := '0';
  signal logits_pack_s  : signed((ACC_BITS_C*N_OUT)-1 downto 0);
  signal vmask_s        : std_logic_vector(N_OUT-1 downto 0);
  signal done_hold_s    : std_logic;
  signal cyc_lat_s      : unsigned(31 downto 0);

  function get_logit(p : signed; idx : integer) return signed is
    variable lo : integer;
    variable hi : integer;
  begin
    lo := idx * ACC_BITS_C;
    hi := (idx+1) * ACC_BITS_C - 1;
    return p(hi downto lo);
  end function;

begin

  clk <= not clk after CLK_PERIOD/2;

  bias_in_s <= bias_mem(to_integer(bias_sel_s));

  u_actv : entity work.msdf_actv_buf
    port map (
      clk      => clk,
      rst_n    => rst_n,
      ce       => ce,

      wr_en    => actv_wr_en,
      wr_addr  => actv_wr_addr,
      x_i      => actv_x_i,

      rd_en    => actv_rd_en,
      rd_addr  => actv_rd_addr,
      rd_valid => actv_rd_v,
      x_o      => actv_x_o
    );

  u_wmem : entity work.msdf_weight_mem
    port map (
      clk       => clk,
      rst_n     => rst_n,
      ce        => ce,

      rd_en     => w_rd_en,
      rd_neuron => w_rd_neuron,
      rd_tile   => w_rd_tile,
      rd_valid  => w_rd_v,
      w_tile_o  => w_tile_o,

      wr_en     => w_wr_en,
      wr_neuron => w_wr_neuron,
      wr_tile   => w_wr_tile,
      w_tile_i  => w_tile_i
    );

  u_ctrl : entity work.msdf_fc_ctrl_10
    generic map (
      ACC_BITS_G    => ACC_BITS_C,
      BIAS_BITS_G   => BIAS_BITS_C,
      FLUSH_EXTRA_G => 4,
      SOFT_RESET_G  => true
    )
    port map (
      clk   => clk,
      rst_n => rst_n,
      ce    => ce,

      start => start_s,
      busy  => busy_s,
      done  => done_s,

      w_rd_en     => w_rd_en,
      w_rd_neuron => w_rd_neuron,
      w_rd_tile   => w_rd_tile,
      w_rd_valid  => w_rd_v,
      w_tile_i    => w_tile_o,

      x_rd_en     => actv_rd_en,
      x_rd_addr   => actv_rd_addr,
      x_rd_valid  => actv_rd_v,
      x_i         => actv_x_o,

      bias_sel    => bias_sel_s,
      bias_in     => bias_in_s,

      logit_we    => logit_we_s,
      logit_idx   => logit_idx_s,
      logit_data  => logit_data_s,

      cycles_total => cycles_total_s
    );

  u_regs : entity work.msdf_logits_regs
    generic map (
      ACC_BITS_G => ACC_BITS_C,
      N_OUT_G    => N_OUT
    )
    port map (
      clk   => clk,
      rst_n => rst_n,
      ce    => ce,

      clear => regs_clear_s,

      logit_we   => logit_we_s,
      logit_idx  => logit_idx_s,
      logit_data => logit_data_s,

      done_pulse_in    => done_s,
      cycles_total_in  => cycles_total_s,

      logits_packed_o  => logits_pack_s,
      valid_mask_o     => vmask_s,
      done_hold_o      => done_hold_s,
      cycles_latched_o => cyc_lat_s
    );

  stim : process
    procedure tick is
    begin
      wait until rising_edge(clk);
      wait for 1 ns;
    end procedure;

    procedure read_int(file f : text; variable v : out integer) is
      variable l : line;
    begin
      if endfile(f) then
        assert false report "TB ERROR(fc): unexpected end of file" severity failure;
      end if;
      readline(f, l);
      read(l, v);
    end procedure;

    procedure load_weights(file fw : text) is
      variable val : integer;
      variable j   : integer;
      variable t   : integer;
      variable lane: integer;
      variable wv  : w_fixed_vec_t;
    begin
      for j in 0 to N_OUT-1 loop
        for t in 0 to N_TILES-1 loop
          for lane in 0 to TILE_LEN-1 loop
            read_int(fw, val);
            wv(lane) := to_signed(val, W_TOTAL_BITS);
          end loop;

          w_tile_i    <= wv;
          w_wr_neuron  <= to_unsigned(j, NEUR_BITS_C);
          w_wr_tile    <= to_unsigned(t, TILE_BITS_C);
          w_wr_en      <= '1';
          tick;
          w_wr_en      <= '0';
          tick;
        end loop;
      end loop;
    end procedure;

    procedure load_bias(file fb : text) is
      variable val : integer;
      variable j   : integer;
    begin
      for j in 0 to N_OUT-1 loop
        read_int(fb, val);
        bias_mem(j) <= to_signed(val, BIAS_BITS_C);
      end loop;
    end procedure;

    procedure load_one_x(file fx : text) is
      variable val : integer;
      variable i   : integer;
    begin
      for i in 0 to VEC_LEN-1 loop
        read_int(fx, val);
        actv_wr_addr <= to_unsigned(i, ADDR_BITS_C);
        actv_x_i     <= to_signed(val, W_TOTAL_BITS);
        actv_wr_en   <= '1';
        tick;
        actv_wr_en   <= '0';
        tick;
      end loop;
    end procedure;

    procedure read_expected_logits(file fl : text; variable exp : inout int_vec_t) is
      variable val : integer;
      variable j   : integer;
    begin
      for j in 0 to N_OUT-1 loop
        read_int(fl, val);
        exp(j) := val;
      end loop;
    end procedure;

    variable exp_logits : int_vec_t(0 to N_OUT-1);
    variable got        : signed(ACC_BITS_C-1 downto 0);
    variable exp_s      : signed(ACC_BITS_C-1 downto 0);

    file fw  : text;
    file fb  : text;
    file fx  : text;
    file fle : text;

    variable st_w  : file_open_status;
    variable st_b  : file_open_status;
    variable st_x  : file_open_status;
    variable st_le : file_open_status;

    variable k   : integer;
    variable cyc : integer;

  begin
    file_open(st_w,  fw,  FILE_W,  read_mode);
    file_open(st_b,  fb,  FILE_B,  read_mode);
    file_open(st_x,  fx,  FILE_X,  read_mode);
    file_open(st_le, fle, FILE_LE, read_mode);

    assert st_w  = open_ok report "TB ERROR(fc): cannot open " & FILE_W  severity failure;
    assert st_b  = open_ok report "TB ERROR(fc): cannot open " & FILE_B  severity failure;
    assert st_x  = open_ok report "TB ERROR(fc): cannot open " & FILE_X  severity failure;
    assert st_le = open_ok report "TB ERROR(fc): cannot open " & FILE_LE severity failure;

    -- Reset
    rst_n <= '0';
    regs_clear_s <= '1';
    tick;
    tick;
    rst_n <= '1';
    tick;
    regs_clear_s <= '0';
    tick;

    -- Load static weights and bias once
    load_weights(fw);
    load_bias(fb);

    -- Run cases
    for k in 0 to N_CASES_TB-1 loop
      load_one_x(fx);

      regs_clear_s <= '1';
      tick;
      regs_clear_s <= '0';
      tick;

      start_s <= '1';
      tick;
      start_s <= '0';

      for cyc in 0 to WATCHDOG_CYCLES loop
        tick;
        if done_s = '1' then
          exit;
        end if;
      end loop;

      assert done_s = '1'
        report "TB ERROR(fc): watchdog timeout waiting done. case=" & integer'image(k)
        severity failure;

      -- Fixed comparison constant avoids Vivado "others aggregate" error
      assert vmask_s = VMASK_ALL1_C
        report "TB ERROR(fc): logits_valid mask not complete. case=" & integer'image(k)
        severity failure;

      read_expected_logits(fle, exp_logits);

      for j in 0 to N_OUT-1 loop
        got   := get_logit(logits_pack_s, j);
        exp_s := to_signed(exp_logits(j), ACC_BITS_C);

        assert got = exp_s
          report "TB ERROR(fc): logit mismatch case=" & integer'image(k) &
                 " j=" & integer'image(j) &
                 " got=" & integer'image(to_integer(got)) &
                 " exp=" & integer'image(exp_logits(j))
          severity failure;
      end loop;

      report "TB OK(fc): case=" & integer'image(k) &
             " cycles=" & integer'image(to_integer(cyc_lat_s))
        severity note;
    end loop;

    report "TB PASSED: tb_msdf_fc_784x10" severity note;

    file_close(fw);
    file_close(fb);
    file_close(fx);
    file_close(fle);

    wait;
  end process;

end architecture;
