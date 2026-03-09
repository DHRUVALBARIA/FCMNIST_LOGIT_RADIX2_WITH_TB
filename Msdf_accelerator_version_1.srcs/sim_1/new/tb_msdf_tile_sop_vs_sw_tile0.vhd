-- ============================================================================
-- tb_msdf_tile_sop_vs_sw_tile0_refs.vhd  (VHDL-2002 safe, XSim-friendly)
--
-- Purpose:
--   For (case0, neuron0, tile0), compare HW tile output against multiple SW
--   fixed-point reference models to identify the actual numeric contract.
--
-- SW models:
--   SW1  per-product floor  : sum_i floor((x_i*w_i)/2^W_FRAC_BITS)
--   SW2  per-product trunc0 : sum_i trunc0((x_i*w_i)/2^W_FRAC_BITS)
--   SW3a sumRaw then floor  : floor((sum_i (x_i*w_i))/2^W_FRAC_BITS)
--   SW3b sumRaw then trunc0 : trunc0((sum_i (x_i*w_i))/2^W_FRAC_BITS)
--
-- Files:
--   w_fixed.txt: j-major then i-minor, one integer per line (N_OUT*VEC_LEN)
--   x_fixed.txt: case-major then i-minor, one integer per line (N_CASES*VEC_LEN)
--
-- Expected:
--   Exactly one of the SW models often matches HW. If none match, HW is doing
--   something else (online truncation/approx), and FC checking must use a bound.
-- ============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

entity tb_msdf_tile_sop_vs_sw_tile0_refs is
end entity;

architecture tb of tb_msdf_tile_sop_vs_sw_tile0_refs is

  constant CLK_PERIOD : time := 10 ns;

  -- Set these to the actual server paths
  constant FILE_W : string := "/home/bad32576/Serial_Parallel_Mul/Msdf_accelerator_version_1/w_fixed.txt";
  constant FILE_X : string := "/home/bad32576/Serial_Parallel_Mul/Msdf_accelerator_version_1/x_fixed.txt";

  -- Watchdogs (in cycles)
  constant WD_TILE_VLD_C  : integer := 20000;
  constant WD_DONE_TILE_C : integer := 20000;

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

  -- Raw-product accumulator width:
  -- product is 2*W_TOTAL_BITS; sum 8 terms -> +3 bits worst-case; add margin.
  constant RAW_BITS_C : integer := (2 * integer(W_TOTAL_BITS)) + 6;
  subtype raw_t is signed(RAW_BITS_C-1 downto 0);

  -- ---------------------------
  -- Software reference helpers
  -- ---------------------------

  function mul_floor_q(a : w_fixed_t; b : w_fixed_t) return integer is
    variable p  : signed((2*W_TOTAL_BITS)-1 downto 0);
    variable ps : signed((2*W_TOTAL_BITS)-1 downto 0);
  begin
    p  := resize(a, W_TOTAL_BITS) * resize(b, W_TOTAL_BITS);
    ps := shift_right(p, W_FRAC_BITS);     -- arithmetic shift => floor for negatives
    return to_integer(ps);
  end function;

  function mul_trunc0_q(a : w_fixed_t; b : w_fixed_t) return integer is
    variable p  : signed((2*W_TOTAL_BITS)-1 downto 0);
    variable ps : signed((2*W_TOTAL_BITS)-1 downto 0);
  begin
    p  := resize(a, W_TOTAL_BITS) * resize(b, W_TOTAL_BITS);
    ps := div_pow2_trunc0_any(p, natural(W_FRAC_BITS)); -- trunc0 toward 0
    return to_integer(ps);
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
        assert false report "TB ERROR(tile0): unexpected EOF" severity failure;
      end if;
      readline(f, l);
      read(l, v);
    end procedure;

    variable st_w : file_open_status;
    variable st_x : file_open_status;

    variable val  : integer;
    variable lane : integer;

    variable sw1_floor_pp   : integer := 0;
    variable sw2_trunc0_pp  : integer := 0;

    variable raw_sum        : raw_t := (others => '0');
    variable sw3a_floor_sum : integer := 0;
    variable sw3b_tr0_sum   : integer := 0;

    variable p_floor  : integer;
    variable p_tr0    : integer;

    variable p_raw    : signed((2*W_TOTAL_BITS)-1 downto 0);
    variable raw_div  : raw_t;

    variable hw_sum   : integer := 0;

    variable seen_vld  : std_logic := '0';
    variable seen_done : std_logic := '0';
    variable tile_lat  : tile_recon_t := (others => '0');

    variable cyc : integer;

    function iabs(x : integer) return integer is
    begin
      if x < 0 then return -x; else return x; end if;
    end function;

  begin
    file_open(st_w, fw, FILE_W, read_mode);
    file_open(st_x, fx, FILE_X, read_mode);

    assert st_w = open_ok report "TB ERROR(tile0): cannot open " & FILE_W severity failure;
    assert st_x = open_ok report "TB ERROR(tile0): cannot open " & FILE_X severity failure;

    -- Reset
    rst_n <= '0';
    tick;
    tick;
    rst_n <= '1';
    tick;

    -- Load weights: (j=0, tile=0) => first 8 lines
    for lane in 0 to TILE_LEN-1 loop
      read_int(fw, val);
      w_tile(lane) <= to_signed(val, W_TOTAL_BITS);
    end loop;

    -- Load activations: (case0, tile=0) => first 8 lines
    for lane in 0 to TILE_LEN-1 loop
      read_int(fx, val);
      x_tile(lane) <= to_signed(val, W_TOTAL_BITS);
    end loop;

    tick; -- settle

    -- Compute SW references
    sw1_floor_pp  := 0;
    sw2_trunc0_pp := 0;

    raw_sum := (others => '0');

    for lane in 0 to TILE_LEN-1 loop
      p_floor := mul_floor_q(x_tile(lane), w_tile(lane));
      p_tr0   := mul_trunc0_q(x_tile(lane), w_tile(lane));

      sw1_floor_pp  := sw1_floor_pp  + p_floor;
      sw2_trunc0_pp := sw2_trunc0_pp + p_tr0;

      -- raw product for "sum then shift once"
      p_raw   := resize(x_tile(lane), W_TOTAL_BITS) * resize(w_tile(lane), W_TOTAL_BITS);
      raw_sum := raw_sum + resize(p_raw, RAW_BITS_C);

      report "TB(tile0): lane=" & integer'image(lane) &
             " x=" & integer'image(to_integer(x_tile(lane))) &
             " w=" & integer'image(to_integer(w_tile(lane))) &
             " pp_floor=" & integer'image(p_floor) &
             " pp_trunc0=" & integer'image(p_tr0) &
             " d(tr0-floor)=" & integer'image(p_tr0 - p_floor)
        severity note;
    end loop;

    -- SW3a: floor after summing raw products
    sw3a_floor_sum := to_integer(shift_right(raw_sum, W_FRAC_BITS));

    -- SW3b: trunc0 after summing raw products
    raw_div := div_pow2_trunc0_any(raw_sum, natural(W_FRAC_BITS));
    sw3b_tr0_sum := to_integer(raw_div);

    report "TB(tile0): SW1 per-prod floor   = " & integer'image(sw1_floor_pp)  severity note;
    report "TB(tile0): SW2 per-prod trunc0  = " & integer'image(sw2_trunc0_pp) severity note;
    report "TB(tile0): SW3a sumRaw  floor    = " & integer'image(sw3a_floor_sum) severity note;
    report "TB(tile0): SW3b sumRaw  trunc0   = " & integer'image(sw3b_tr0_sum)  severity note;

    -- Start tile
    tile_start <= '1';
    tick;
    tile_start <= '0';

    -- Latch pulses like your controller does (cannot miss a 1-cycle pulse)
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
      report "TB ERROR(tile0): watchdog waiting tile_vld" severity failure;
    assert (seen_done = '1')
      report "TB ERROR(tile0): watchdog waiting done_tile" severity failure;

    hw_sum := to_integer(tile_lat);
    report "TB(tile0): HW tile sum          = " & integer'image(hw_sum) severity note;

    -- Print deltas to identify matching contract
    report "TB(tile0): delta vs SW1  = " & integer'image(hw_sum - sw1_floor_pp)  severity note;
    report "TB(tile0): delta vs SW2  = " & integer'image(hw_sum - sw2_trunc0_pp) severity note;
    report "TB(tile0): delta vs SW3a = " & integer'image(hw_sum - sw3a_floor_sum) severity note;
    report "TB(tile0): delta vs SW3b = " & integer'image(hw_sum - sw3b_tr0_sum)  severity note;

    -- Decide which model matches (if any)
    if hw_sum = sw2_trunc0_pp then
      report "TB(tile0): MATCH = per-product trunc0 (SW2)" severity note;
    elsif hw_sum = sw3b_tr0_sum then
      report "TB(tile0): MATCH = sumRaw then trunc0 (SW3b)" severity note;
    elsif hw_sum = sw3a_floor_sum then
      report "TB(tile0): MATCH = sumRaw then floor (SW3a)" severity note;
    elsif hw_sum = sw1_floor_pp then
      report "TB(tile0): MATCH = per-product floor (SW1)" severity note;
    else
      assert false
        report "TB ERROR(tile0): HW matches none of SW models. " &
               "This implies online/truncation effects beyond simple shift placement."
        severity failure;
    end if;

    report "TB PASSED: tb_msdf_tile_sop_vs_sw_tile0_refs" severity note;

    file_close(fw);
    file_close(fx);
    wait;
  end process;

end architecture;
