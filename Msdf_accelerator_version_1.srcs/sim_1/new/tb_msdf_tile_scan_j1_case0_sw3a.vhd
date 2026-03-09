-- ============================================================================
-- tb_msdf_tile_scan_j1_case0_sw3a.vhd   (VHDL-2002, XSim friendly)
--
-- Purpose:
--   For case k=0 and neuron j=1, scan tiles t=0..N_TILES-1 and compare:
--     HW tile (msdf_tile_sop + msdf_tile_recon_r2)  vs  SW3a reference
--
-- SW3a reference:
--   raw_sum = sum_{lane=0..7} (x_i * w_i)   (raw Q24 style integer)
--   sw_tile = arithmetic_shift_right(raw_sum, W_FRAC_BITS)
--
-- Stops at first mismatch and prints details.
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

entity tb_msdf_tile_scan_j1_case0_sw3a is
end entity;

architecture tb of tb_msdf_tile_scan_j1_case0_sw3a is

  constant CLK_PERIOD : time := 10 ns;

  constant FILE_W : string := "/home/bad32576/Serial_Parallel_Mul/Msdf_accelerator_version_1/w_fixed_2.txt";
  constant FILE_X : string := "/home/bad32576/Serial_Parallel_Mul/Msdf_accelerator_version_1/x_fixed_2.txt";

  constant NEUR_J_C : integer := 1;
  constant CASE_K_C : integer := 0;

  constant WD_DONE_TILE_C : integer := 50000;

  signal clk   : std_logic := '0';
  signal rst_n : std_logic := '0';
  signal ce    : std_logic := '1';

  signal tile_start : std_logic := '0';
  signal in_ready   : std_logic := '1';

  signal x_tile : w_fixed_vec_t := (others => (others => '0'));
  signal w_tile : w_fixed_vec_t := (others => (others => '0'));

  signal active_s  : std_logic;
  signal done_tile : std_logic;

  signal sum_d : sd_digit_t := SD_ZERO;
  signal sum_v : std_logic  := '0';

  signal tile_val : tile_recon_t := (others => '0');
  signal tile_vld : std_logic := '0';

  file fw : text;
  file fx : text;

  type int_arr_t is array (0 to VEC_LEN-1) of integer;

  -- Raw-product accumulator width:
  -- product is 2*W_TOTAL_BITS, sum 8 terms adds ~3 bits, add margin.
  constant RAW_BITS_C : integer := (2 * integer(W_TOTAL_BITS)) + 6;
  subtype raw_t is signed(RAW_BITS_C-1 downto 0);

begin

  clk <= not clk after CLK_PERIOD/2;

  u_tile : entity work.msdf_tile_sop
    generic map (
      FLUSH_EXTRA_G => 4,
      SOFT_RESET_G  => true
    )
    port map (
      clk           => clk,
      rst_n         => rst_n,
      ce            => ce,

      tile_start    => tile_start,
      in_ready      => in_ready,

      x_tile_in     => x_tile,
      w_tile_in     => w_tile,

      active        => active_s,
      done_tile     => done_tile,

      sum_digit_out => sum_d,
      sum_valid_out => sum_v
    );

  u_recon : entity work.msdf_tile_recon_r2
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
      start          => tile_start,
      step           => sum_v,
      d_in           => sum_d,
      tile_value_out => tile_val,
      tile_valid     => tile_vld,
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
        assert false report "TB ERROR(tile_scan): unexpected EOF" severity failure;
      end if;
      readline(f, l);
      read(l, v);
    end procedure;

    procedure skip_lines(file f : text; constant n : integer) is
      variable dummy : integer;
      variable i     : integer;
    begin
      for i in 0 to n-1 loop
        read_int(f, dummy);
      end loop;
    end procedure;

    variable st_w : file_open_status;
    variable st_x : file_open_status;

    variable x_vec : int_arr_t;
    variable w_vec : int_arr_t;

    variable val   : integer;
    variable i     : integer;
    variable t     : integer;
    variable lane  : integer;

    variable raw_sum : raw_t;
    variable p_raw   : signed((2*W_TOTAL_BITS)-1 downto 0);
    variable sw_tile_i : integer;

    variable seen_vld  : std_logic;
    variable seen_done : std_logic;
    variable tile_lat  : tile_recon_t;

    variable hw_tile_i : integer;
    variable delta_i   : integer;

    variable acc_sw : integer := 0;
    variable acc_hw : integer := 0;

  begin
    file_open(st_w, fw, FILE_W, read_mode);
    file_open(st_x, fx, FILE_X, read_mode);

    assert st_w = open_ok report "TB ERROR(tile_scan): cannot open " & FILE_W severity failure;
    assert st_x = open_ok report "TB ERROR(tile_scan): cannot open " & FILE_X severity failure;

    -- Reset
    rst_n <= '0';
    tick;
    tick;
    rst_n <= '1';
    tick;

    -- Load neuron j weights into w_vec
    -- w_fixed.txt layout: j-major, total VEC_LEN lines per neuron
    skip_lines(fw, NEUR_J_C * VEC_LEN);
    for i in 0 to VEC_LEN-1 loop
      read_int(fw, val);
      w_vec(i) := val;
    end loop;

    -- Load case k activations into x_vec
    -- x_fixed.txt layout: case-major, VEC_LEN lines per case
    skip_lines(fx, CASE_K_C * VEC_LEN);
    for i in 0 to VEC_LEN-1 loop
      read_int(fx, val);
      x_vec(i) := val;
    end loop;

    report "TB(tile_scan): loaded x_vec and w_vec for case=" & integer'image(CASE_K_C) &
           " neuron=" & integer'image(NEUR_J_C)
      severity note;

    acc_sw := 0;
    acc_hw := 0;

    for t in 0 to N_TILES-1 loop

      -- Drive tile inputs
      for lane in 0 to TILE_LEN-1 loop
        i := (t * TILE_LEN) + lane;
        x_tile(lane) <= to_signed(x_vec(i), W_TOTAL_BITS);
        w_tile(lane) <= to_signed(w_vec(i), W_TOTAL_BITS);
      end loop;

      tick; -- settle

      -- SW3a compute: sum raw products then arithmetic shift right by W_FRAC_BITS
      raw_sum := (others => '0');
      for lane in 0 to TILE_LEN-1 loop
        i := (t * TILE_LEN) + lane;
        p_raw   := resize(to_signed(x_vec(i), W_TOTAL_BITS), W_TOTAL_BITS) *
                   resize(to_signed(w_vec(i), W_TOTAL_BITS), W_TOTAL_BITS);
        raw_sum := raw_sum + resize(p_raw, RAW_BITS_C);
      end loop;

      sw_tile_i := to_integer(shift_right(raw_sum, W_FRAC_BITS));
      acc_sw := acc_sw + sw_tile_i;

      -- Start HW tile
      tile_start <= '1';
      tick;
      tile_start <= '0';

      -- Latch pulses (tile_valid and done_tile are 1-cycle)
      seen_vld  := '0';
      seen_done := '0';
      tile_lat  := (others => '0');

      for i in 0 to WD_DONE_TILE_C loop
        tick;

        if tile_vld = '1' then
          seen_vld := '1';
          tile_lat := tile_val;
        end if;

        if done_tile = '1' then
          seen_done := '1';
        end if;

        if (seen_vld = '1') and (seen_done = '1') then
          exit;
        end if;
      end loop;

      assert (seen_vld = '1') report "TB ERROR(tile_scan): watchdog waiting tile_vld, t=" & integer'image(t) severity failure;
      assert (seen_done = '1') report "TB ERROR(tile_scan): watchdog waiting done_tile, t=" & integer'image(t) severity failure;

      hw_tile_i := to_integer(tile_lat);
      acc_hw := acc_hw + hw_tile_i;

      delta_i := hw_tile_i - sw_tile_i;

      report "TB(tile_scan): t=" & integer'image(t) &
             " sw=" & integer'image(sw_tile_i) &
             " hw=" & integer'image(hw_tile_i) &
             " delta=" & integer'image(delta_i) &
             " acc_sw=" & integer'image(acc_sw) &
             " acc_hw=" & integer'image(acc_hw)
        severity note;

      if delta_i /= 0 then
        report "TB(tile_scan): FIRST MISMATCH at tile t=" & integer'image(t) severity note;

        -- Print lane details for the first mismatch tile
        for lane in 0 to TILE_LEN-1 loop
          i := (t * TILE_LEN) + lane;
          report "TB(tile_scan): lane=" & integer'image(lane) &
                 " x=" & integer'image(x_vec(i)) &
                 " w=" & integer'image(w_vec(i)) &
                 " xw=" & integer'image(x_vec(i) * w_vec(i))
            severity note;
        end loop;

        assert false report "TB(tile_scan): stopping at first mismatch" severity failure;
      end if;

    end loop;

    report "TB(tile_scan): ALL TILES MATCH SW3a for case0 neuron1" severity note;

    file_close(fw);
    file_close(fx);
    wait;
  end process;

end architecture;
