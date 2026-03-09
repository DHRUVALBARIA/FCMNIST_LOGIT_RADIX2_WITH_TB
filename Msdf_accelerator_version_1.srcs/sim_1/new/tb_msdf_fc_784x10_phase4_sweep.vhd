-- ============================================================================
-- tb/tb_msdf_fc_784x10_phase4_sweep.vhd
--
-- Phase 4.3: Full-dataset characterization sweep (EOF streaming)
--
-- Purpose:
--   . Run FC784x10 for every case in x_fixed.txt until EOF
--   . Compare logits vs logits_exp.txt
--   . Enforce a TEMPORARY loose bound (FC_BOUND_TMP) to catch gross mismatches
--   . Track global max_abs_err, min_err, max_err
--   . Track per-neuron max abs error
--   . Track cycle statistics (min/max/avg) using regs-latched cycles
--
-- Completion condition:
--   . Wait on done_hold_s (from msdf_logits_regs), NOT done_s (controller pulse)
--
-- Expected outputs:
--   . "DONE cases=N ... max_abs_err=..."
--   . per-neuron max abs
--   . cycles: min/max/avg
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

entity tb_msdf_fc_784x10_phase4_sweep is
end entity;

architecture tb of tb_msdf_fc_784x10_phase4_sweep is

  constant CLK_PERIOD : time := 10 ns;

  constant ACC_BITS_C  : positive := 40;
  constant BIAS_BITS_C : positive := 16;

  -- Temporary loose bound during characterization (do NOT treat as final contract)
  constant FC_BOUND_TMP : integer := 10;

  -- Watchdog per case
  constant WATCHDOG_FC_CYCLES : integer := 40000000;

  constant FILE_W  : string := "/home/bad32576/Serial_Parallel_Mul/Msdf_accelerator_version_1/w_fixed.txt";
  constant FILE_B  : string := "/home/bad32576/Serial_Parallel_Mul/Msdf_accelerator_version_1/b_fixed.txt";
  constant FILE_X  : string := "/home/bad32576/Serial_Parallel_Mul/Msdf_accelerator_version_1/x_fixed.txt";
  constant FILE_LE : string := "/home/bad32576/Serial_Parallel_Mul/Msdf_accelerator_version_1/logits_exp.txt";

  constant VMASK_ALL1_C : std_logic_vector(N_OUT-1 downto 0) := (others => '1');

  -- Clocks/resets
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
  signal logit_data_s : signed(ACC_BITS_C-1 downto 0);

  signal cycles_total_s : unsigned(31 downto 0);

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
      FLUSH_EXTRA_G => 2,
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
      logit_we         => logit_we_s,
      logit_idx        => logit_idx_s,
      logit_data       => logit_data_s,
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

    function iabs(x : integer) return integer is
    begin
      if x < 0 then return -x; else return x; end if;
    end function;

    procedure read_int(file f : text; variable v : out integer) is
      variable l : line;
    begin
      if endfile(f) then
        assert false report "TB ERROR(p4): unexpected EOF" severity failure;
      end if;
      readline(f, l);
      read(l, v);
    end procedure;

    procedure try_read_int(file f : text; variable v : out integer; variable ok : out boolean) is
      variable l : line;
    begin
      if endfile(f) then
        ok := false;
        v  := 0;
      else
        readline(f, l);
        read(l, v);
        ok := true;
      end if;
    end procedure;

    type int_vec10_t  is array (0 to N_OUT-1) of integer;
    type int_vec784_t is array (0 to VEC_LEN-1) of integer;

    file fw  : text;
    file fb  : text;
    file fx  : text;
    file fle : text;

    variable st_w  : file_open_status;
    variable st_b  : file_open_status;
    variable st_x  : file_open_status;
    variable st_le : file_open_status;

    variable val : integer;
    variable ok  : boolean;

    variable j    : integer;
    variable t    : integer;
    variable lane : integer;
    variable i    : integer;

    variable x_vec   : int_vec784_t;
    variable le_vec  : int_vec10_t;
    variable got_vec : int_vec10_t;

    variable err       : integer;
    variable max_abs_e : integer := 0;
    variable min_e     : integer := 0;
    variable max_e     : integer := 0;

    type int10_t is array (0 to N_OUT-1) of integer;
    variable per_neur_maxabs : int10_t := (others => 0);

    variable wv : w_fixed_vec_t;

    variable case_idx  : integer := 0;
    variable cyc       : integer;
    variable have_case : boolean;

    -- Cycle statistics (from regs-latched cycles)
    variable cyc_this  : integer := 0;
    variable cyc_min   : integer := 0;
    variable cyc_max   : integer := 0;
    variable cyc_sum   : integer := 0;  -- integer is safe unless you run *very* large datasets

    procedure load_one_case(variable have_case_o : out boolean) is
    begin
      for i in 0 to VEC_LEN-1 loop
        try_read_int(fx, val, ok);
        if not ok then
          have_case_o := false;
          return;
        end if;
        x_vec(i) := val;
      end loop;

      for j in 0 to N_OUT-1 loop
        try_read_int(fle, val, ok);
        if not ok then
          assert false report "TB ERROR(p4): logits_exp EOF before x_fixed EOF" severity failure;
        end if;
        le_vec(j) := val;
      end loop;

      have_case_o := true;
    end procedure;

  begin
    -- Open files
    file_open(st_w,  fw,  FILE_W,  read_mode);
    file_open(st_b,  fb,  FILE_B,  read_mode);
    file_open(st_x,  fx,  FILE_X,  read_mode);
    file_open(st_le, fle, FILE_LE, read_mode);

    assert st_w  = open_ok report "TB ERROR(p4): cannot open w_fixed.txt" severity failure;
    assert st_b  = open_ok report "TB ERROR(p4): cannot open b_fixed.txt" severity failure;
    assert st_x  = open_ok report "TB ERROR(p4): cannot open x_fixed.txt" severity failure;
    assert st_le = open_ok report "TB ERROR(p4): cannot open logits_exp.txt" severity failure;

    -- Reset
    rst_n <= '0';
    regs_clear_s <= '1';
    tick; tick;
    rst_n <= '1';
    tick;
    regs_clear_s <= '0';
    tick;

    -- Load weights into weight_mem
    for j in 0 to N_OUT-1 loop
      for t in 0 to N_TILES-1 loop
        for lane in 0 to TILE_LEN-1 loop
          read_int(fw, val);
          wv(lane) := to_signed(val, W_TOTAL_BITS);
        end loop;

        w_tile_i    <= wv;
        w_wr_neuron <= to_unsigned(j, NEUR_BITS_C);
        w_wr_tile   <= to_unsigned(t, TILE_BITS_C);
        w_wr_en     <= '1';
        tick;
        w_wr_en     <= '0';
        tick;
      end loop;
    end loop;

    -- Load biases
    for j in 0 to N_OUT-1 loop
      read_int(fb, val);
      bias_mem(j) <= to_signed(val, BIAS_BITS_C);
    end loop;
    tick;

    report "TB(p4): loaded W and B, start EOF sweep..." severity note;

    -- Sweep cases until EOF
    case_idx := 0;
    loop
      load_one_case(have_case);
      exit when not have_case;

      -- Load activation buffer
      for i in 0 to VEC_LEN-1 loop
        actv_wr_addr <= to_unsigned(i, ADDR_BITS_C);
        actv_x_i     <= to_signed(x_vec(i), W_TOTAL_BITS);
        actv_wr_en   <= '1';
        tick;
        actv_wr_en   <= '0';
        tick;
      end loop;

      -- Clear regs and start
      regs_clear_s <= '1';
      tick;
      regs_clear_s <= '0';
      tick;

      start_s <= '1';
      tick;
      start_s <= '0';

      -- Wait for regs-latched completion
      for cyc in 0 to WATCHDOG_FC_CYCLES loop
        tick;
        if done_hold_s = '1' then
          exit;
        end if;
      end loop;

      assert done_hold_s = '1'
        report "TB ERROR(p4): watchdog timeout, case=" & integer'image(case_idx)
        severity failure;

      assert vmask_s = VMASK_ALL1_C
        report "TB ERROR(p4): logits_valid mask incomplete, case=" & integer'image(case_idx)
        severity failure;

      assert to_integer(cyc_lat_s) > 0
        report "TB ERROR(p4): cycles_latched_o is 0 (controller hold or regs latch issue)"
        severity failure;

      -- Cycle stats update
      cyc_this := to_integer(cyc_lat_s);
      if case_idx = 0 then
        cyc_min := cyc_this;
        cyc_max := cyc_this;
        cyc_sum := cyc_this;
      else
        if cyc_this < cyc_min then cyc_min := cyc_this; end if;
        if cyc_this > cyc_max then cyc_max := cyc_this; end if;
        cyc_sum := cyc_sum + cyc_this;
      end if;

      -- Capture logits
      for j in 0 to N_OUT-1 loop
        got_vec(j) := to_integer(get_logit(logits_pack_s, j));
      end loop;

      -- Update error stats + enforce temporary bound
      for j in 0 to N_OUT-1 loop
        err := got_vec(j) - le_vec(j);

        if (case_idx = 0) and (j = 0) then
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

        assert iabs(err) <= FC_BOUND_TMP
          report "TB ERROR(p4): case=" & integer'image(case_idx) &
                 " j=" & integer'image(j) &
                 " got=" & integer'image(got_vec(j)) &
                 " exp=" & integer'image(le_vec(j)) &
                 " err=" & integer'image(err) &
                 " tmp_bound=" & integer'image(FC_BOUND_TMP)
          severity failure;
      end loop;

      -- Progress print
      if (case_idx mod 10) = 0 then
        report "TB(p4): case=" & integer'image(case_idx) &
               " cycles=" & integer'image(cyc_this) &
               " max_abs_err_so_far=" & integer'image(max_abs_e)
          severity note;
      end if;

      case_idx := case_idx + 1;
    end loop;

    -- Ensure logits file does not contain more cases than x_fixed
    try_read_int(fle, val, ok);
    assert not ok
      report "TB ERROR(p4): logits_exp has more cases than x_fixed"
      severity failure;

    file_close(fw); file_close(fb); file_close(fx); file_close(fle);

    -- Summary
    report "TB(p4): DONE cases=" & integer'image(case_idx) &
           " tmp_bound=" & integer'image(FC_BOUND_TMP) &
           " max_abs_err=" & integer'image(max_abs_e) &
           " min_err=" & integer'image(min_e) &
           " max_err=" & integer'image(max_e)
      severity note;

    for j in 0 to N_OUT-1 loop
      report "TB(p4): per_neuron_max_abs j=" & integer'image(j) &
             " max_abs=" & integer'image(per_neur_maxabs(j))
        severity note;
    end loop;

    if case_idx > 0 then
      report "TB(p4): cycles_stats min=" & integer'image(cyc_min) &
             " max=" & integer'image(cyc_max) &
             " avg=" & integer'image(cyc_sum / case_idx)
        severity note;
    end if;

    report "TB PASSED: tb_msdf_fc_784x10_phase4_sweep" severity note;
    wait;
  end process;

end architecture;
