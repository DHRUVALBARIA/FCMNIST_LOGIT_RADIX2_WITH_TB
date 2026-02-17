-- ============================================================================
-- tb/tb_msdf_fc_tile_delta_closure_all.vhd  (VHDL-2002 safe, XSim friendly)
--
-- Purpose:
--   For all k in [0..N_CASES_TB-1], all j in [0..N_OUT-1]:
--     . run FC controller to get got_logit(j)
--     . compare got vs exp (from logits_exp.txt) -> delta_fc
--     . independently scan tiles using a standalone msdf_tile_sop instance
--       and SW3a reference per tile -> sum_delta
--     . ASSERT: delta_fc == sum_delta
--
-- This proves FC mismatch is entirely explained by tile-level finite-digit error.
--
-- Run:
--   use "run all" (do NOT stop at 100us; this TB can take ms)
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

entity tb_msdf_fc_tile_delta_closure_all is
end entity;

architecture tb of tb_msdf_fc_tile_delta_closure_all is

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

  constant WATCHDOG_FC_CYCLES : integer := 20000000;
  constant WD_TILE_CYCLES     : integer := 50000;

  constant FILE_W  : string := "/home/bad32576/Serial_Parallel_Mul/Msdf_accelerator_version_1/w_fixed.txt";
  constant FILE_B  : string := "/home/bad32576/Serial_Parallel_Mul/Msdf_accelerator_version_1/b_fixed.txt";
  constant FILE_X  : string := "/home/bad32576/Serial_Parallel_Mul/Msdf_accelerator_version_1/x_fixed.txt";
  constant FILE_LE : string := "/home/bad32576/Serial_Parallel_Mul/Msdf_accelerator_version_1/logits_exp.txt";

  constant VMASK_ALL1_C : std_logic_vector(N_OUT-1 downto 0) := (others => '1');

  -- -------------------------
  -- Integer storage in TB
  -- -------------------------
  type int_vec784_t is array (0 to VEC_LEN-1) of integer;
  type int_vec10_t  is array (0 to N_OUT-1) of integer;

  type x_cases_t    is array (0 to N_CASES_TB-1) of int_vec784_t;
  type w_neurons_t  is array (0 to N_OUT-1) of int_vec784_t;
  type le_cases_t   is array (0 to N_CASES_TB-1) of int_vec10_t;

  signal dummy : std_logic := '0'; -- keeps some tools calm

  -- -------------------------
  -- DUT-side signals (FC)
  -- -------------------------
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

  -- -------------------------
  -- Standalone tile instance (for scanning)
  -- -------------------------
  signal tile_start2 : std_logic := '0';
  signal in_ready2   : std_logic := '1';
  signal x_tile2     : w_fixed_vec_t := (others => (others => '0'));
  signal w_tile2     : w_fixed_vec_t := (others => (others => '0'));
  signal active2     : std_logic;
  signal done_tile2  : std_logic;

  signal sum_d2 : sd_digit_t := SD_ZERO;
  signal sum_v2 : std_logic  := '0';

  signal tile_val2 : tile_recon_t := (others => '0');
  signal tile_vld2 : std_logic := '0';

  constant RAW_BITS_C : integer := (2 * integer(W_TOTAL_BITS)) + 6;
  subtype raw_t is signed(RAW_BITS_C-1 downto 0);

begin

  clk <= not clk after CLK_PERIOD/2;
  bias_in_s <= bias_mem(to_integer(bias_sel_s));

  -- FC DUT blocks
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

  -- Standalone tile for scanning
  u_tile2 : entity work.msdf_tile_sop
    generic map (
      FLUSH_EXTRA_G => 4,
      SOFT_RESET_G  => true
    )
    port map (
      clk           => clk,
      rst_n         => rst_n,
      ce            => ce,
      tile_start    => tile_start2,
      in_ready      => in_ready2,
      x_tile_in     => x_tile2,
      w_tile_in     => w_tile2,
      active        => active2,
      done_tile     => done_tile2,
      sum_digit_out => sum_d2,
      sum_valid_out => sum_v2
    );

  u_recon2 : entity work.msdf_tile_recon_r2
    generic map (
      N_DIGITS_G   => N_DIGITS,
      CA_BITS_G    => CA_BITS,
      OUT_BITS_G   => TILE_RECON_BITS,
      SHIFT_LEFT_G => 0
    )
    port map (
      clk            => clk,
      rst_n          => rst_n,
      ce             => '1',
      start          => tile_start2,
      step           => sum_v2,
      d_in           => sum_d2,
      tile_value_out => tile_val2,
      tile_valid     => tile_vld2,
      busy           => open,
      digit_count    => open
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
        assert false report "TB ERROR(closure): unexpected EOF" severity failure;
      end if;
      readline(f, l);
      read(l, v);
    end procedure;

    function iabs(x : integer) return integer is
    begin
      if x < 0 then return -x; else return x; end if;
    end function;

    -- TB storage
    variable X   : x_cases_t;
    variable W   : w_neurons_t;
    variable B   : int_vec10_t;
    variable LE  : le_cases_t;

    -- files
    file fw  : text;
    file fb  : text;
    file fx  : text;
    file fle : text;

    variable st_w  : file_open_status;
    variable st_b  : file_open_status;
    variable st_x  : file_open_status;
    variable st_le : file_open_status;

    variable val : integer;

    -- loop vars
    variable k    : integer;
    variable j    : integer;
    variable t    : integer;
    variable lane : integer;
    variable i    : integer;
    variable cyc  : integer;

    -- controller results
    variable got_ctrl : int_vec10_t;

    -- tile scan vars
    variable raw_sum : raw_t;
    variable p_raw   : signed((2*W_TOTAL_BITS)-1 downto 0);
    variable sw_tile : integer;
    variable hw_tile : integer;
    variable delta_t : integer;

    variable sum_delta      : integer;
    variable mismatch_count : integer;
    variable max_abs_delta  : integer;

    variable delta_fc : integer;

    -- latch tile pulses
    variable seen_vld  : std_logic;
    variable seen_done : std_logic;
    variable tile_lat  : tile_recon_t;

    variable wv : w_fixed_vec_t;

  begin
    -- -----------------------------
    -- Read all vectors into TB RAM
    -- -----------------------------
    file_open(st_w,  fw,  FILE_W,  read_mode);
    file_open(st_b,  fb,  FILE_B,  read_mode);
    file_open(st_x,  fx,  FILE_X,  read_mode);
    file_open(st_le, fle, FILE_LE, read_mode);

    assert st_w  = open_ok report "TB ERROR(closure): cannot open " & FILE_W  severity failure;
    assert st_b  = open_ok report "TB ERROR(closure): cannot open " & FILE_B  severity failure;
    assert st_x  = open_ok report "TB ERROR(closure): cannot open " & FILE_X  severity failure;
    assert st_le = open_ok report "TB ERROR(closure): cannot open " & FILE_LE severity failure;

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

    file_close(fw);
    file_close(fb);
    file_close(fx);
    file_close(fle);

    report "TB(closure): loaded W,B,X,LE into TB RAM" severity note;

    -- -----------------------------
    -- Reset
    -- -----------------------------
    rst_n <= '0';
    regs_clear_s <= '1';
    tick; tick;
    rst_n <= '1';
    tick;
    regs_clear_s <= '0';
    tick;

    -- -----------------------------
    -- Load weights into DUT weight_mem
    -- -----------------------------
    for j in 0 to N_OUT-1 loop
      for t in 0 to N_TILES-1 loop
        for lane in 0 to TILE_LEN-1 loop
          wv(lane) := to_signed(W(j)(t*TILE_LEN + lane), W_TOTAL_BITS);
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

    -- Load bias into TB bias_mem
    for j in 0 to N_OUT-1 loop
      bias_mem(j) <= to_signed(B(j), BIAS_BITS_C);
    end loop;

    tick;

    -- =============================
    -- Main loop: for each CASE k
    -- =============================
    for k in 0 to N_CASES_TB-1 loop

      -- load activation buffer for case k
      for i in 0 to VEC_LEN-1 loop
        actv_wr_addr <= to_unsigned(i, ADDR_BITS_C);
        actv_x_i     <= to_signed(X(k)(i), W_TOTAL_BITS);
        actv_wr_en   <= '1';
        tick;
        actv_wr_en   <= '0';
        tick;
      end loop;

      -- clear regs
      regs_clear_s <= '1';
      tick;
      regs_clear_s <= '0';
      tick;

      -- start controller
      start_s <= '1';
      tick;
      start_s <= '0';

      for cyc in 0 to WATCHDOG_FC_CYCLES loop
        tick;
        if done_s = '1' then
          exit;
        end if;
      end loop;

      assert done_s = '1'
        report "TB ERROR(closure): FC watchdog timeout, case=" & integer'image(k)
        severity failure;

      assert vmask_s = VMASK_ALL1_C
        report "TB ERROR(closure): logits_valid mask incomplete, case=" & integer'image(k)
        severity failure;

      -- capture controller logits
      for j in 0 to N_OUT-1 loop
        got_ctrl(j) := to_integer(get_logit(logits_pack_s, j));
      end loop;

      report "TB(closure): FC done case=" & integer'image(k) &
             " cycles=" & integer'image(to_integer(cyc_lat_s))
        severity note;

      -- ==========================================
      -- For each neuron j: compute sum_delta by scan
      -- ==========================================
      for j in 0 to N_OUT-1 loop
        sum_delta      := 0;
        mismatch_count := 0;
        max_abs_delta  := 0;

        for t in 0 to N_TILES-1 loop

          -- drive tile vectors
          for lane in 0 to TILE_LEN-1 loop
            x_tile2(lane) <= to_signed(X(k)(t*TILE_LEN + lane), W_TOTAL_BITS);
            w_tile2(lane) <= to_signed(W(j)(t*TILE_LEN + lane), W_TOTAL_BITS);
          end loop;

          tick; -- settle

          -- SW3a: sum raw products then arithmetic shift right W_FRAC_BITS
          raw_sum := (others => '0');
          for lane in 0 to TILE_LEN-1 loop
            p_raw   := resize(to_signed(X(k)(t*TILE_LEN + lane), W_TOTAL_BITS), W_TOTAL_BITS) *
                       resize(to_signed(W(j)(t*TILE_LEN + lane), W_TOTAL_BITS), W_TOTAL_BITS);
            raw_sum := raw_sum + resize(p_raw, RAW_BITS_C);
          end loop;
          sw_tile := to_integer(shift_right(raw_sum, W_FRAC_BITS));

          -- run HW tile
          tile_start2 <= '1';
          tick;
          tile_start2 <= '0';

          seen_vld  := '0';
          seen_done := '0';
          tile_lat  := (others => '0');

          for cyc in 0 to WD_TILE_CYCLES loop
            tick;

            if tile_vld2 = '1' then
              seen_vld := '1';
              tile_lat := tile_val2;
            end if;

            if done_tile2 = '1' then
              seen_done := '1';
            end if;

            if (seen_vld = '1') and (seen_done = '1') then
              exit;
            end if;
          end loop;

          assert (seen_vld = '1')
            report "TB ERROR(closure): tile watchdog tile_vld, case=" & integer'image(k) &
                   " j=" & integer'image(j) & " t=" & integer'image(t)
            severity failure;

          assert (seen_done = '1')
            report "TB ERROR(closure): tile watchdog done_tile, case=" & integer'image(k) &
                   " j=" & integer'image(j) & " t=" & integer'image(t)
            severity failure;

          hw_tile := to_integer(tile_lat);
          delta_t := hw_tile - sw_tile;

          sum_delta := sum_delta + delta_t;

          if delta_t /= 0 then
            mismatch_count := mismatch_count + 1;
          end if;

          if iabs(delta_t) > max_abs_delta then
            max_abs_delta := iabs(delta_t);
          end if;

        end loop; -- tiles

        delta_fc := got_ctrl(j) - LE(k)(j);

        report "TB(closure): case=" & integer'image(k) &
               " j=" & integer'image(j) &
               " got=" & integer'image(got_ctrl(j)) &
               " exp=" & integer'image(LE(k)(j)) &
               " delta_fc=" & integer'image(delta_fc) &
               " sum_delta=" & integer'image(sum_delta) &
               " mism_tiles=" & integer'image(mismatch_count) &
               " max_abs_tile=" & integer'image(max_abs_delta)
          severity note;

        assert delta_fc = sum_delta
          report "TB ERROR(closure): delta_fc != sum_delta, case=" & integer'image(k) &
                 " j=" & integer'image(j) &
                 " delta_fc=" & integer'image(delta_fc) &
                 " sum_delta=" & integer'image(sum_delta)
          severity failure;

      end loop; -- neurons

    end loop; -- cases

    report "TB PASSED: tb_msdf_fc_tile_delta_closure_all" severity note;
    wait;
  end process;

end architecture;
