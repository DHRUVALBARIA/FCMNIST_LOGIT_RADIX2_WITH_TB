library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

-- ============================================================================
-- TB Variant 2: tile_start robustness tests for msdf_tile_sop
--
-- Tests:
--   A) Restart during active:
--      . Start tile with tile_start
--      . While active=1, assert tile_start again (restart)
--      . Expect: clean restart and correct final reconstructed sum
--
--   B) Double-pulse start:
--      . Assert tile_start for two consecutive cycles
--      . Expect: behaves like a restart; still finishes and matches golden
--
-- Stalls:
--   Use a mild random in_ready pattern (not the long burst torture; Variant 1 did that)
-- ============================================================================

entity tb_msdf_tile_sop_start_robust is
end entity;

architecture tb of tb_msdf_tile_sop_start_robust is

  constant CLK_PERIOD : time := 10 ns;

  constant WATCHDOG_CYCLES : integer := 250000;
  constant FLUSH_EXTRA_TB  : integer := 4;

  constant W_SCALE_I       : integer := 2 ** W_FRAC_BITS;

  signal clk   : std_logic := '0';
  signal rst_n : std_logic := '0';
  signal ce    : std_logic := '1';

  signal tile_start : std_logic := '0';
  signal in_ready   : std_logic := '0';

  signal x_tile_in  : w_fixed_vec_t := (others => (others => '0'));
  signal w_tile_in  : w_fixed_vec_t := (others => (others => '0'));

  signal active    : std_logic;
  signal done_tile : std_logic;

  signal sum_d : sd_digit_t := SD_ZERO;
  signal sum_v : std_logic  := '0';

  signal sum_rec_out : tile_recon_t := (others => '0');
  signal sum_rec_vld : std_logic    := '0';

  -- ------------------------------------------------------------
  -- PRNG (deterministic)
  -- ------------------------------------------------------------
  function next_seed(s : integer) return integer is
  begin
    return (s * 1103515245 + 12345) mod 2147483647;
  end function;

  -- ------------------------------------------------------------
  -- Helpers
  -- ------------------------------------------------------------
  function pow2_i(k : natural) return integer is
    variable v : integer := 1;
  begin
    for j in 1 to k loop
      v := v * 2;
    end loop;
    return v;
  end function;

  function abs_i(x : integer) return integer is
  begin
    if x < 0 then return -x; else return x; end if;
  end function;

  function ashr_i(x : integer; sh : natural) return integer is
    variable d : integer;
  begin
    if sh = 0 then
      return x;
    end if;

    d := pow2_i(sh);

    if x >= 0 then
      return x / d;
    else
      return -(((-x) + d - 1) / d);
    end if;
  end function;

  function mul_fixed_ashr(xi : integer; wi : integer) return integer is
    variable p : integer;
  begin
    p := xi * wi;
    return ashr_i(p, natural(W_FRAC_BITS));
  end function;

  function golden_tile_sum_ashr(xv : w_fixed_vec_t; wv : w_fixed_vec_t) return integer is
    variable acc : integer := 0;
  begin
    for i in 0 to TILE_LEN-1 loop
      acc := acc + mul_fixed_ashr(to_integer(xv(i)), to_integer(wv(i)));
    end loop;
    return acc;
  end function;

begin

  clk <= not clk after CLK_PERIOD/2;

  -- ------------------------------------------------------------
  -- DUT
  -- ------------------------------------------------------------
  dut : entity work.msdf_tile_sop
    generic map (
      FLUSH_EXTRA_G => FLUSH_EXTRA_TB,
      SOFT_RESET_G  => true
    )
    port map (
      clk           => clk,
      rst_n         => rst_n,
      ce            => ce,

      tile_start    => tile_start,
      in_ready      => in_ready,

      x_tile_in     => x_tile_in,
      w_tile_in     => w_tile_in,

      active        => active,
      done_tile     => done_tile,

      sum_digit_out => sum_d,
      sum_valid_out => sum_v
    );

  -- ------------------------------------------------------------
  -- Reconstruction (restarts on tile_start)
  -- ------------------------------------------------------------
  u_sum_rec : entity work.msdf_tile_recon_r2
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
      tile_value_out => sum_rec_out,
      tile_valid     => sum_rec_vld,
      busy           => open,
      digit_count    => open
    );

  -- ------------------------------------------------------------
  -- TB process
  -- ------------------------------------------------------------
  stim : process

    procedure tick is
    begin
      wait until rising_edge(clk);
      wait for 1 ns;
    end procedure;

    procedure do_reset is
    begin
      rst_n      <= '0';
      ce         <= '1';
      tile_start <= '0';
      in_ready   <= '0';

      for i in 0 to TILE_LEN-1 loop
        x_tile_in(i) <= (others => '0');
        w_tile_in(i) <= (others => '0');
      end loop;

      tick;
      tick;
      rst_n <= '1';
      tick;
    end procedure;

    procedure drive_tile(
      variable xv : in w_fixed_vec_t;
      variable wv : in w_fixed_vec_t
    ) is
    begin
      for i in 0 to TILE_LEN-1 loop
        x_tile_in(i) <= xv(i);
        w_tile_in(i) <= wv(i);
      end loop;
    end procedure;

    procedure make_all_const(
      variable xv : out w_fixed_vec_t;
      variable wv : out w_fixed_vec_t;
      constant xval : in integer;
      constant wval : in integer
    ) is
    begin
      for i in 0 to TILE_LEN-1 loop
        xv(i) := to_signed(xval, W_TOTAL_BITS);
        wv(i) := to_signed(wval, W_TOTAL_BITS);
      end loop;
    end procedure;

    -- Mild stall pattern (keeps things realistic but not torture-level)
    procedure drive_ready_mild(variable seed : inout integer) is
    begin
      seed := next_seed(seed);
      if (seed mod 5) = 0 then
        in_ready <= '0';
      else
        in_ready <= '1';
      end if;
    end procedure;

    -- Wait until active becomes 1 (with watchdog)
    procedure wait_active_high(constant case_id : in integer) is
      variable k : integer := 0;
    begin
      for k in 0 to 10000 loop
        tick;
        exit when active = '1';
      end loop;

      assert active = '1'
        report "TB ERROR(tile_sop_start): active did not go high. case_id=" & integer'image(case_id)
        severity failure;
    end procedure;

    -- Core checker: run one tile but optionally inject extra tile_start pulses
    procedure run_tile_case_with_start_injection(
      variable xv      : in w_fixed_vec_t;
      variable wv      : in w_fixed_vec_t;
      constant seed_in : in integer;
      constant case_id : in integer;
      constant mode    : in integer   -- 0=normal, 1=restart while active, 2=double-pulse start
    ) is
      variable seed         : integer := seed_in;

      variable exp_sum      : integer := 0;
      variable got_sum      : integer := 0;
      variable tol          : integer := 0;

      variable got_done     : boolean := false;
      variable got_recon    : boolean := false;

      variable sum_cnt      : integer := 0;
      variable cyc          : integer := 0;

      variable in_contract  : boolean := true;

      -- For mode 1/2 we want to observe the restart behavior clearly
      variable injected     : boolean := false;
    begin
      exp_sum := golden_tile_sum_ashr(xv, wv);

      if abs_i(exp_sum) > W_SCALE_I then
        in_contract := false;
        report "TB NOTE(tile_sop_start): case_id=" & integer'image(case_id) &
               " OUT-OF-CONTRACT. exp_sum=" & integer'image(exp_sum)
          severity note;
      end if;

      if W_FRAC_BITS > N_DIGITS then
        tol := pow2_i(natural(W_FRAC_BITS - N_DIGITS)) + 12;
      else
        tol := 12;
      end if;

      drive_tile(xv, wv);

      -- --------------------------------------------------------
      -- Start behavior modes
      -- --------------------------------------------------------
      if mode = 2 then
        -- Double pulse: tile_start high for two consecutive cycles
        tile_start <= '1';
        in_ready   <= '0';
        tick;
        tile_start <= '1';
        tick;
        tile_start <= '0';
      else
        -- Normal single pulse start
        tile_start <= '1';
        in_ready   <= '0';
        tick;
        tile_start <= '0';
      end if;

      got_done  := false;
      got_recon := false;
      sum_cnt   := 0;
      injected  := false;

      -- If mode 1: wait for active=1 then inject a restart tile_start pulse
      if mode = 1 then
        wait_active_high(case_id);

        -- let it run a few cycles (so we are truly mid-tile)
        for k in 0 to 5 loop
          drive_ready_mild(seed);
          tick;
        end loop;

        -- Inject restart pulse while active is 1
        report "TB NOTE(tile_sop_start): injecting RESTART tile_start while active=1 case_id=" &
               integer'image(case_id) severity note;

        tile_start <= '1';
        tick;
        tile_start <= '0';

        injected := true;
      end if;

      -- Main run loop
      for cyc in 0 to WATCHDOG_CYCLES loop
        drive_ready_mild(seed);
        tick;

        if sum_v = '1' then
          sum_cnt := sum_cnt + 1;
        end if;

        if done_tile = '1' then
          got_done := true;
        end if;

        if sum_rec_vld = '1' then
          got_recon := true;
          got_sum := to_integer(sum_rec_out);
        end if;

        if got_done and got_recon then
          exit;
        end if;
      end loop;

      in_ready <= '0';
      tick;

      assert got_done
        report "TB ERROR(tile_sop_start): done_tile did not assert. case_id=" & integer'image(case_id)
        severity failure;

      assert got_recon
        report "TB ERROR(tile_sop_start): sum reconstruction did not complete. case_id=" & integer'image(case_id)
        severity failure;

      assert sum_cnt >= N_DIGITS
        report "TB ERROR(tile_sop_start): too few sum valid digits. cnt=" & integer'image(sum_cnt) &
               " N_DIGITS=" & integer'image(N_DIGITS) &
               " case_id=" & integer'image(case_id)
        severity failure;

      if in_contract then
        assert abs_i(got_sum - exp_sum) <= tol
          report "TB ERROR(tile_sop_start): mismatch. case_id=" & integer'image(case_id) &
                 " got=" & integer'image(got_sum) &
                 " exp=" & integer'image(exp_sum) &
                 " diff=" & integer'image(got_sum - exp_sum) &
                 " tol=" & integer'image(tol)
          severity failure;

        report "TB OK(tile_sop_start): case_id=" & integer'image(case_id) &
               " mode=" & integer'image(mode) &
               " got=" & integer'image(got_sum) &
               " exp=" & integer'image(exp_sum) &
               " diff=" & integer'image(got_sum - exp_sum) &
               " tol=" & integer'image(tol)
          severity note;
      else
        report "TB NOTE(tile_sop_start): out-of-contract observed. case_id=" & integer'image(case_id) &
               " mode=" & integer'image(mode) &
               " got=" & integer'image(got_sum) &
               " exp=" & integer'image(exp_sum)
          severity note;
      end if;

    end procedure;

    variable xv_v : w_fixed_vec_t;
    variable wv_v : w_fixed_vec_t;

    constant HALF_X     : integer := pow2_i(natural(W_FRAC_BITS-1)); -- +0.5
    constant ONE_W      : integer := pow2_i(natural(W_FRAC_BITS));   -- +1.0
    constant ONE_W_DIV4 : integer := ONE_W / 4;                      -- +0.25 (in-contract)

  begin
    do_reset;

    -- Base directed in-contract tile (near bound)
    make_all_const(xv_v, wv_v, HALF_X, ONE_W_DIV4);

    -- Mode 0: normal start
    run_tile_case_with_start_injection(xv_v, wv_v, 1001, 10, 0);

    -- Mode 2: double pulse at start
    run_tile_case_with_start_injection(xv_v, wv_v, 2002, 11, 2);

    -- Mode 1: restart while active
    run_tile_case_with_start_injection(xv_v, wv_v, 3003, 12, 1);

    report "TB PASSED: tb_msdf_tile_sop_start_robust" severity note;
    wait;
  end process;

end architecture;
