library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

entity tb_msdf_fc_784x10_bounded is
end entity;

architecture tb of tb_msdf_fc_784x10_bounded is

  constant CLK_PERIOD : time := 10 ns;

  constant ACC_BITS_C  : positive := 40;
  constant BIAS_BITS_C : positive := 16;

  constant FC_BOUND : integer := 16;
  constant N_CASES_TB : integer := 5;

  constant WATCHDOG_FC_CYCLES : integer := 20000000;

  constant FILE_W  : string := "/home/bad32576/Serial_Parallel_Mul/Msdf_accelerator_version_1/w_fixed.txt";
  constant FILE_B  : string := "/home/bad32576/Serial_Parallel_Mul/Msdf_accelerator_version_1/b_fixed.txt";
  constant FILE_X  : string := "/home/bad32576/Serial_Parallel_Mul/Msdf_accelerator_version_1/x_fixed.txt";
  constant FILE_LE : string := "/home/bad32576/Serial_Parallel_Mul/Msdf_accelerator_version_1/logits_exp.txt";

  constant VMASK_ALL1_C : std_logic_vector(N_OUT-1 downto 0) := (others => '1');

  subtype logit_t is signed(ACC_BITS_C-1 downto 0);

  type int_vec784_t is array (0 to VEC_LEN-1) of integer;
  type int_vec10_t  is array (0 to N_OUT-1) of integer;

  type x_cases_t    is array (0 to N_CASES_TB-1) of int_vec784_t;
  type w_neurons_t  is array (0 to N_OUT-1) of int_vec784_t;
  type le_cases_t   is array (0 to N_CASES_TB-1) of int_vec10_t;

  signal clk   : std_logic := '0';
  signal rst_n : std_logic := '0';
  signal ce    : std_logic := '1';

  -- Activation buffer
  signal actv_wr_en   : std_logic := '0';
  signal actv_wr_addr : addr_idx_t := (others => '0');
  signal actv_x_i     : w_fixed_t := (others => '0');

  signal actv_rd_en   : std_logic := '0';
  signal actv_rd_addr : addr_idx_t := (others => '0');
  signal actv_rd_v    : std_logic;
  signal actv_x_o     : w_fixed_t;

  -- Weight memory
  signal w_wr_en      : std_logic := '0';
  signal w_wr_neuron  : neur_idx_t := (others => '0');
  signal w_wr_tile    : tile_idx_t := (others => '0');
  signal w_tile_i     : w_fixed_vec_t := (others => (others => '0'));

  signal w_rd_en      : std_logic := '0';
  signal w_rd_neuron  : neur_idx_t := (others => '0');
  signal w_rd_tile    : tile_idx_t := (others => '0');
  signal w_rd_v       : std_logic;
  signal w_tile_o     : w_fixed_vec_t;

  -- Bias store in TB
  type bias_arr_t is array (0 to N_OUT-1) of signed(BIAS_BITS_C-1 downto 0);
  signal bias_mem   : bias_arr_t := (others => (others => '0'));
  signal bias_sel_s : neur_idx_t := (others => '0');
  signal bias_in_s  : signed(BIAS_BITS_C-1 downto 0) := (others => '0');

  -- Controller + regs
  signal start_s : std_logic := '0';
  signal busy_s  : std_logic;
  signal done_s  : std_logic;

  signal logit_we_s   : std_logic;
  signal logit_idx_s  : neur_idx_t;
  signal logit_data_s : logit_t;

  signal cycles_total_s : unsigned(31 downto 0);

  signal regs_clear_s   : std_logic := '0';
  signal logits_pack_s  : signed((ACC_BITS_C*N_OUT)-1 downto 0);
  signal vmask_s        : std_logic_vector(N_OUT-1 downto 0);
  signal done_hold_s    : std_logic;
  signal cyc_lat_s      : unsigned(31 downto 0);

  function get_logit(p : signed; idx : integer) return logit_t is
    variable lo : integer;
    variable hi : integer;
  begin
    lo := idx * ACC_BITS_C;
    hi := (idx+1) * ACC_BITS_C - 1;
    return logit_t(p(hi downto lo));
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
        assert false report "TB ERROR(bounded): unexpected EOF" severity failure;
      end if;
      readline(f, l);
      read(l, v);
    end procedure;

    function iabs(x : integer) return integer is
    begin
      if x < 0 then return -x; else return x; end if;
    end function;

    variable X   : x_cases_t;
    variable W   : w_neurons_t;
    variable B   : int_vec10_t;
    variable LE  : le_cases_t;

    file fw  : text;
    file fb  : text;
    file fx  : text;
    file fle : text;

    variable st_w  : file_open_status;
    variable st_b  : file_open_status;
    variable st_x  : file_open_status;
    variable st_le : file_open_status;

    variable val : integer;

    variable k : integer;
    variable j : integer;
    variable t : integer;
    variable i : integer;
    variable lane : integer;

    variable got_ctrl : int_vec10_t;

    variable err       : integer;
    variable max_abs_e : integer := 0;
    variable min_e     : integer := 0;
    variable max_e     : integer := 0;

    type int10_t is array (0 to N_OUT-1) of integer;
    variable per_neur_maxabs : int10_t := (others => 0);

    variable wv : w_fixed_vec_t;

    variable watchdog_hit : boolean;

  begin
    file_open(st_w,  fw,  FILE_W,  read_mode);
    file_open(st_b,  fb,  FILE_B,  read_mode);
    file_open(st_x,  fx,  FILE_X,  read_mode);
    file_open(st_le, fle, FILE_LE, read_mode);

    assert st_w  = open_ok report "TB ERROR(bounded): cannot open w_fixed.txt" severity failure;
    assert st_b  = open_ok report "TB ERROR(bounded): cannot open b_fixed.txt" severity failure;
    assert st_x  = open_ok report "TB ERROR(bounded): cannot open x_fixed.txt" severity failure;
    assert st_le = open_ok report "TB ERROR(bounded): cannot open logits_exp.txt" severity failure;

    for j in 0 to N_OUT-1 loop
      for i in 0 to VEC_LEN-1 loop
        read_int(fw, val);
        W(j)(i) := val;
      end loop;
    end loop;

    for j in 0 to N_OUT-1 loop
      read_int(fb, val);
      B(j) := val;
    end loop;

    for k in 0 to N_CASES_TB-1 loop
      for i in 0 to VEC_LEN-1 loop
        read_int(fx, val);
        X(k)(i) := val;
      end loop;
    end loop;

    for k in 0 to N_CASES_TB-1 loop
      for j in 0 to N_OUT-1 loop
        read_int(fle, val);
        LE(k)(j) := val;
      end loop;
    end loop;

    file_close(fw); file_close(fb); file_close(fx); file_close(fle);
    report "TB(bounded): loaded W,B,X,LE into TB RAM" severity note;

    rst_n <= '0';
    regs_clear_s <= '1';
    tick; tick;
    rst_n <= '1';
    tick;
    regs_clear_s <= '0';
    tick;

    for j in 0 to N_OUT-1 loop
      for t in 0 to N_TILES-1 loop
        for lane in 0 to TILE_LEN-1 loop
          wv(lane) := to_signed(W(j)(t*TILE_LEN + lane), W_TOTAL_BITS);
        end loop;
        w_tile_i    <= wv;
        w_wr_neuron <= to_unsigned(j, w_wr_neuron'length);
        w_wr_tile   <= to_unsigned(t, w_wr_tile'length);
        w_wr_en     <= '1';
        tick;
        w_wr_en     <= '0';
        tick;
      end loop;
    end loop;

    for j in 0 to N_OUT-1 loop
      bias_mem(j) <= to_signed(B(j), BIAS_BITS_C);
    end loop;
    tick;

    for k in 0 to N_CASES_TB-1 loop
      for i in 0 to VEC_LEN-1 loop
        actv_wr_addr <= to_unsigned(i, actv_wr_addr'length);
        actv_x_i     <= to_signed(X(k)(i), W_TOTAL_BITS);
        actv_wr_en   <= '1';
        tick;
        actv_wr_en   <= '0';
        tick;
      end loop;

      regs_clear_s <= '1';
      tick;
      regs_clear_s <= '0';
      tick;

      start_s <= '1';
      tick;
      start_s <= '0';

      watchdog_hit := true;
      for i in 0 to WATCHDOG_FC_CYCLES loop
        tick;
        if done_s = '1' then
          watchdog_hit := false;
          exit;
        end if;
      end loop;

      assert (not watchdog_hit)
        report "TB ERROR(bounded): FC watchdog timeout, case=" & integer'image(k)
        severity failure;

      assert vmask_s = VMASK_ALL1_C
        report "TB ERROR(bounded): logits_valid mask incomplete, case=" & integer'image(k)
        severity failure;

      for j in 0 to N_OUT-1 loop
        got_ctrl(j) := to_integer(get_logit(logits_pack_s, j));
      end loop;

      for j in 0 to N_OUT-1 loop
        err := got_ctrl(j) - LE(k)(j);

        if (k = 0) and (j = 0) then
          min_e := err;
          max_e := err;
        else
          if err < min_e then min_e := err; end if;
          if err > max_e then max_e := err; end if;
        end if;

        if iabs(err) > max_abs_e then
          max_abs_e := iabs(err);
        end if;

        if iabs(err) > per_neur_maxabs(j) then
          per_neur_maxabs(j) := iabs(err);
        end if;

        assert iabs(err) <= FC_BOUND
          report "TB ERROR(bounded): case=" & integer'image(k) &
                 " j=" & integer'image(j) &
                 " got=" & integer'image(got_ctrl(j)) &
                 " exp=" & integer'image(LE(k)(j)) &
                 " err=" & integer'image(err) &
                 " bound=" & integer'image(FC_BOUND)
          severity failure;
      end loop;

      report "TB(bounded): case=" & integer'image(k) &
             " cycles=" & integer'image(to_integer(cyc_lat_s)) &
             " max_abs_err_so_far=" & integer'image(max_abs_e)
        severity note;

    end loop;

    report "TB(bounded): DONE N_CASES=" & integer'image(N_CASES_TB) &
           " FC_BOUND=" & integer'image(FC_BOUND) &
           " max_abs_err=" & integer'image(max_abs_e) &
           " min_err=" & integer'image(min_e) &
           " max_err=" & integer'image(max_e)
      severity note;

    for j in 0 to N_OUT-1 loop
      report "TB(bounded): per_neuron_max_abs j=" & integer'image(j) &
             " max_abs=" & integer'image(per_neur_maxabs(j))
        severity note;
    end loop;

    report "TB PASSED: tb_msdf_fc_784x10_bounded" severity note;
    wait;
  end process;

end architecture;
