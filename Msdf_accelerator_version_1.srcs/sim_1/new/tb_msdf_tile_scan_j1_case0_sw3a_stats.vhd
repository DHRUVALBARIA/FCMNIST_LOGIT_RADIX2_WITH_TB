-- ============================================================================
-- tb/tb_msdf_tile_scan_j1_case0_sw3a_stats.vhd  (VHDL-2002 safe, XSim friendly)
--
-- Purpose:
--   Scan ALL tiles t=0..N_TILES-1 for one (case, neuron) pair and compare
--   HW tile output vs SW3a (sumRaw then floor/ashr once).
--
--   Collect stats:
--     . sum_delta      = Σ(hw - sw) across all tiles
--     . mismatch_count = number of tiles where hw != sw
--     . max_abs_delta  = max |hw - sw|
--
--   Also prints the first few mismatches (configurable) with lane breakdown.
--
-- Inputs (text files):
--   w_fixed.txt : N_OUT*VEC_LEN lines, TB expects j-major then tile then lane
--   x_fixed.txt : N_CASES*VEC_LEN lines, TB expects case-major then i
--
-- NOTE:
--   This TB assumes FILE_W/FILE_X point to the *server-side* copies used by XSim.
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

entity tb_msdf_tile_scan_j1_case0_sw3a_stats is
end entity;

architecture tb of tb_msdf_tile_scan_j1_case0_sw3a_stats is

  constant CLK_PERIOD : time := 10 ns;

  -- --------- EDIT THESE IF NEEDED ----------
  constant FILE_W : string := "/home/bad32576/Serial_Parallel_Mul/Msdf_accelerator_version_1/w_fixed.txt";
  constant FILE_X : string := "/home/bad32576/Serial_Parallel_Mul/Msdf_accelerator_version_1/x_fixed.txt";

  constant CASE_K_C  : integer := 0;
  constant NEUR_J_C  : integer := 1;

  -- limit verbose mismatch prints
  constant MAX_MISMATCH_PRINT_C : integer := 5;

  -- watchdogs (cycles) for one tile run
  constant WD_TILE_VLD_C  : integer := 50000;
  constant WD_DONE_TILE_C : integer := 50000;
  -- -----------------------------------------

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

  -- product is 2*W_TOTAL_BITS; summing 8 terms needs +3 bits; add margin.
  constant RAW_BITS_C : integer := (2 * integer(W_TOTAL_BITS)) + 6;
  subtype raw_t is signed(RAW_BITS_C-1 downto 0);

  function iabs(x : integer) return integer is
  begin
    if x < 0 then return -x; else return x; end if;
  end function;

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
        assert false report "TB ERROR(tile_scan_stats): unexpected EOF" severity failure;
      end if;
      readline(f, l);
      read(l, v);
    end procedure;

    -- Read and discard N integers from a file (for skipping)
    procedure skip_ints(file f : text; constant n : integer) is
      variable tmp : integer;
      variable k   : integer;
    begin
      for k in 0 to n-1 loop
        read_int(f, tmp);
      end loop;
    end procedure;

    -- Load x vector for CASE_K_C into a local full array (784)
    type int_arr_t is array (0 to VEC_LEN-1) of integer;
    type int2d_arr_t is array (0 to N_OUT-1, 0 to VEC_LEN-1) of integer;

    variable x_vec : int_arr_t;
    variable w_vec : int2d_arr_t;

    -- stats
    variable sum_delta      : integer := 0;
    variable mismatch_count : integer := 0;
    variable max_abs_delta  : integer := 0;
    variable mism_printed   : integer := 0;

    -- per tile
    variable t     : integer;
    variable lane  : integer;
    variable val   : integer;

    variable raw_sum : raw_t;
    variable sw_tile : integer;
    variable hw_tile : integer;
    variable delta   : integer;

    variable p_raw : signed((2*W_TOTAL_BITS)-1 downto 0);

    -- pulse latch
    variable seen_vld  : std_logic;
    variable seen_done : std_logic;
    variable tile_lat  : tile_recon_t;

    variable cyc : integer;

    variable st_w : file_open_status;
    variable st_x : file_open_status;

    -- helpers to load whole arrays
    variable j : integer;
    variable i : integer;

  begin
    file_open(st_w, fw, FILE_W, read_mode);
    file_open(st_x, fx, FILE_X, read_mode);

    assert st_w = open_ok report "TB ERROR(tile_scan_stats): cannot open " & FILE_W severity failure;
    assert st_x = open_ok report "TB ERROR(tile_scan_stats): cannot open " & FILE_X severity failure;

    -- Reset
    rst_n <= '0';
    tick; tick;
    rst_n <= '1';
    tick;

    -- -----------------------------
    -- Load full x_vec for CASE_K_C
    -- x_fixed.txt order: case-major then i
    -- -----------------------------
    if CASE_K_C > 0 then
      skip_ints(fx, CASE_K_C * integer(VEC_LEN));
    end if;

    for i in 0 to VEC_LEN-1 loop
      read_int(fx, val);
      x_vec(i) := val;
    end loop;

    -- -----------------------------
    -- Load full w_vec for ALL j
    -- w_fixed.txt order produced by your dump script:
    --   j major, then tile t, then lane
    --   i = t*TILE_LEN + lane
    -- -----------------------------
    for j in 0 to N_OUT-1 loop
      for t in 0 to N_TILES-1 loop
        for lane in 0 to TILE_LEN-1 loop
          read_int(fw, val);
          w_vec(j, (t*TILE_LEN) + lane) := val;
        end loop;
      end loop;
    end loop;

    report "TB(tile_scan_stats): loaded x_vec and w_vec for case=" &
           integer'image(CASE_K_C) & " neuron=" & integer'image(NEUR_J_C)
      severity note;

    -- -----------------------------
    -- Scan all tiles
    -- -----------------------------
    sum_delta      := 0;
    mismatch_count := 0;
    max_abs_delta  := 0;
    mism_printed   := 0;

    for t in 0 to N_TILES-1 loop

      -- drive tile inputs
      for lane in 0 to TILE_LEN-1 loop
        x_tile(lane) <= to_signed(x_vec((t*TILE_LEN)+lane), W_TOTAL_BITS);
        w_tile(lane) <= to_signed(w_vec(NEUR_J_C, (t*TILE_LEN)+lane), W_TOTAL_BITS);
      end loop;

      tick; -- settle

      -- SW3a compute: sum raw products then arithmetic shift right once
      raw_sum := (others => '0');
      for lane in 0 to TILE_LEN-1 loop
        p_raw   := resize(to_signed(x_vec((t*TILE_LEN)+lane), W_TOTAL_BITS), W_TOTAL_BITS) *
                   resize(to_signed(w_vec(NEUR_J_C, (t*TILE_LEN)+lane), W_TOTAL_BITS), W_TOTAL_BITS);
        raw_sum := raw_sum + resize(p_raw, RAW_BITS_C);
      end loop;

      sw_tile := to_integer(shift_right(raw_sum, W_FRAC_BITS));

      -- run HW tile
      tile_start <= '1';
      tick;
      tile_start <= '0';

      seen_vld  := '0';
      seen_done := '0';
      tile_lat  := (others => '0');

      for cyc in 0 to WD_DONE_TILE_C loop
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

      assert (seen_vld = '1')
        report "TB ERROR(tile_scan_stats): watchdog waiting tile_vld at t=" & integer'image(t)
        severity failure;
      assert (seen_done = '1')
        report "TB ERROR(tile_scan_stats): watchdog waiting done_tile at t=" & integer'image(t)
        severity failure;

      hw_tile := to_integer(tile_lat);
      delta   := hw_tile - sw_tile;

      -- stats
      sum_delta := sum_delta + delta;

      if delta /= 0 then
        mismatch_count := mismatch_count + 1;
      end if;

      if iabs(delta) > max_abs_delta then
        max_abs_delta := iabs(delta);
      end if;

      -- print a few mismatches with lane breakdown
      if (delta /= 0) and (mism_printed < MAX_MISMATCH_PRINT_C) then
        report "TB(tile_scan_stats): MISMATCH t=" & integer'image(t) &
               " sw=" & integer'image(sw_tile) &
               " hw=" & integer'image(hw_tile) &
               " delta=" & integer'image(delta)
          severity note;

        for lane in 0 to TILE_LEN-1 loop
          report "TB(tile_scan_stats):   lane=" & integer'image(lane) &
                 " x=" & integer'image(x_vec((t*TILE_LEN)+lane)) &
                 " w=" & integer'image(w_vec(NEUR_J_C,(t*TILE_LEN)+lane)) &
                 " xw=" & integer'image(x_vec((t*TILE_LEN)+lane) * w_vec(NEUR_J_C,(t*TILE_LEN)+lane))
            severity note;
        end loop;

        mism_printed := mism_printed + 1;
      end if;

      -- optional progress prints (comment out if noisy)
      if (t mod 16) = 0 then
        report "TB(tile_scan_stats): progress t=" & integer'image(t) &
               " sum_delta=" & integer'image(sum_delta) &
               " mism=" & integer'image(mismatch_count) &
               " max_abs=" & integer'image(max_abs_delta)
          severity note;
      end if;

    end loop;

    -- Final summary
    report "TB(tile_scan_stats): DONE case=" & integer'image(CASE_K_C) &
           " neuron=" & integer'image(NEUR_J_C) &
           " sum_delta=" & integer'image(sum_delta) &
           " mismatch_count=" & integer'image(mismatch_count) &
           " max_abs_delta=" & integer'image(max_abs_delta)
      severity note;

    -- If you want a PASS/FAIL gate for the “sum_delta should equal FC delta” hypothesis,
    -- uncomment and set expected value here.
    -- assert (sum_delta = 7)
    --   report "TB ERROR(tile_scan_stats): sum_delta does not match expected FC delta"
    --   severity failure;

    report "TB PASSED: tb_msdf_tile_scan_j1_case0_sw3a_stats" severity note;

    file_close(fw);
    file_close(fx);

    wait;
  end process;

end architecture;
