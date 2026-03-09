library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

-- ============================================================================
-- TB Variant 1: Long stall-burst torture for msdf_tile_sop
--
-- Goal:
--   Prove that long backpressure bursts (in_ready held low for 50..200 cycles)
--   do NOT cause:
--     . deadlock (done_tile / recon never arrives)
--     . loss of valid pulses (sum stream too short)
--     . numeric divergence (in-contract cases must still match golden)
--
-- This TB is intentionally similar to your main tb_msdf_tile_sop, but with
-- a different stall generator.
-- ============================================================================

entity tb_msdf_tile_sop_stall_torture is
end entity;

architecture tb of tb_msdf_tile_sop_stall_torture is

  constant CLK_PERIOD : time := 10 ns;

  -- Keep modest: we are stressing stalls, not volume
  constant N_CASES_RAND    : integer := 15;
  constant WATCHDOG_CYCLES : integer := 400000;

  constant FLUSH_EXTRA_TB  : integer := 4;

  -- Contract bound for tile sum magnitude (in Q(W_FRAC_BITS))
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
  -- Helpers (integer math)
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

  -- Arithmetic shift-right for integer: floor(x/2^sh) for negatives
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

  -- Phase-1 "MUL_SCALE_MODE=ashr":
  -- prod = ashr(x*w, W_FRAC_BITS)
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
  -- Reconstruct tile sum stream (SHIFT_LEFT_G=0 per your add-tree TB)
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

    procedure make_random_tile(
      constant seed0 : in integer;
      variable xv : out w_fixed_vec_t;
      variable wv : out w_fixed_vec_t
    ) is
      variable s  : integer := seed0;
      variable xi : integer;
      variable wi : integer;
    begin
      -- mimic your Phase-1 ranges:
      -- x about [-3686..3686], w about [-512..512]
      for i in 0 to TILE_LEN-1 loop
        s  := next_seed(s);
        xi := (s mod 7373) - 3686;

        s  := next_seed(s);
        wi := (s mod 1025) - 512;

        xv(i) := to_signed(xi, W_TOTAL_BITS);
        wv(i) := to_signed(wi, W_TOTAL_BITS);
      end loop;
    end procedure;

    procedure run_tile_case(
      variable xv      : in w_fixed_vec_t;
      variable wv      : in w_fixed_vec_t;
      constant seed_in : in integer;
      constant case_id : in integer
    ) is
      variable seed         : integer := seed_in;

      variable got_done     : boolean := false;
      variable got_recon    : boolean := false;

      variable exp_sum      : integer := 0;
      variable got_sum      : integer := 0;
      variable tol          : integer := 0;

      variable sum_cnt      : integer := 0;
      variable cyc          : integer := 0;

      variable in_contract  : boolean := true;

      -- Long stall-burst controller
      variable stall_rem    : integer := 0;
      variable burst_len    : integer := 0;
    begin
      exp_sum := golden_tile_sum_ashr(xv, wv);

      if abs_i(exp_sum) > W_SCALE_I then
        in_contract := false;
        report "TB NOTE(tile_sop_stall): case_id=" & integer'image(case_id) &
               " OUT-OF-CONTRACT. exp_sum=" & integer'image(exp_sum)
          severity note;
      end if;

      if W_FRAC_BITS > N_DIGITS then
        tol := pow2_i(natural(W_FRAC_BITS - N_DIGITS)) + 12;
      else
        tol := 12;
      end if;

      drive_tile(xv, wv);

      -- Start tile
      tile_start <= '1';
      in_ready   <= '0';
      tick;
      tile_start <= '0';

      sum_cnt   := 0;
      got_done  := false;
      got_recon := false;

      stall_rem := 0;

      for cyc in 0 to WATCHDOG_CYCLES loop

        -- --------------------------------------------------------
        -- Stall torture:
        --   Occasionally start a long burst: in_ready=0 for 50..200 cycles.
        --   Otherwise keep in_ready=1 (with rare single-cycle hiccups).
        -- --------------------------------------------------------
        if stall_rem > 0 then
          in_ready  <= '0';
          stall_rem := stall_rem - 1;
        else
          seed := next_seed(seed);

          -- 1/16 chance to start a long stall burst
          if (seed mod 16) = 0 then
            seed := next_seed(seed);
            burst_len := (seed mod 151) + 50;  -- 50..200
            stall_rem := burst_len;
            in_ready  <= '0';

            report "TB NOTE(tile_sop_stall): case_id=" & integer'image(case_id) &
                   " starting stall_burst len=" & integer'image(burst_len)
              severity note;
          else
            -- mostly ready, with a rare 1-cycle hiccup (1/32)
            seed := next_seed(seed);
            if (seed mod 32) = 0 then
              in_ready <= '0';
            else
              in_ready <= '1';
            end if;
          end if;
        end if;

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
        report "TB ERROR(tile_sop_stall): done_tile did not assert. case_id=" & integer'image(case_id)
        severity failure;

      assert got_recon
        report "TB ERROR(tile_sop_stall): sum reconstruction did not complete. case_id=" & integer'image(case_id)
        severity failure;

      assert sum_cnt >= N_DIGITS
        report "TB ERROR(tile_sop_stall): too few sum valid digits. cnt=" & integer'image(sum_cnt) &
               " N_DIGITS=" & integer'image(N_DIGITS) &
               " case_id=" & integer'image(case_id)
        severity failure;

      if in_contract then
        assert abs_i(got_sum - exp_sum) <= tol
          report "TB ERROR(tile_sop_stall): mismatch. case_id=" & integer'image(case_id) &
                 " got=" & integer'image(got_sum) &
                 " exp=" & integer'image(exp_sum) &
                 " diff=" & integer'image(got_sum - exp_sum) &
                 " tol=" & integer'image(tol)
          severity failure;

        report "TB OK(tile_sop_stall): case_id=" & integer'image(case_id) &
               " got=" & integer'image(got_sum) &
               " exp=" & integer'image(exp_sum) &
               " diff=" & integer'image(got_sum - exp_sum) &
               " tol=" & integer'image(tol)
          severity note;
      else
        report "TB NOTE(tile_sop_stall): out-of-contract observed. case_id=" & integer'image(case_id) &
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

    variable k : integer;

  begin
    do_reset;

    -- Directed sanity
    make_all_const(xv_v, wv_v, 0, 123);
    run_tile_case(xv_v, wv_v, 11, 0);

    -- In-contract near-bound
    make_all_const(xv_v, wv_v, HALF_X, ONE_W_DIV4);
    run_tile_case(xv_v, wv_v, 22, 1);

    -- Mixed signs
    for i in 0 to TILE_LEN-1 loop
      if (i mod 2) = 0 then
        xv_v(i) := to_signed( 3000, W_TOTAL_BITS);
        wv_v(i) := to_signed(-400,  W_TOTAL_BITS);
      else
        xv_v(i) := to_signed(-2000, W_TOTAL_BITS);
        wv_v(i) := to_signed( 500,  W_TOTAL_BITS);
      end if;
    end loop;
    run_tile_case(xv_v, wv_v, 33, 2);

    -- Random tiles with stall torture
    for k in 0 to N_CASES_RAND-1 loop
      make_random_tile(5000 + 97*k, xv_v, wv_v);
      run_tile_case(xv_v, wv_v, 7000 + 31*k, 100 + k);
    end loop;

    -- Back-to-back without global reset
    make_random_tile(7777, xv_v, wv_v);
    run_tile_case(xv_v, wv_v, 8888, 9000);

    make_random_tile(9999, xv_v, wv_v);
    run_tile_case(xv_v, wv_v, 1111, 9001);

    report "TB PASSED: tb_msdf_tile_sop_stall_torture" severity note;
    wait;
  end process;

end architecture;
