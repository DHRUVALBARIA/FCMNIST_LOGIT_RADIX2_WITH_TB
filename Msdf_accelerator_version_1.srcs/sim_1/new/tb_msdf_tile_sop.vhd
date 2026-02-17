library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

entity tb_msdf_tile_sop is
end entity;

architecture tb of tb_msdf_tile_sop is

  constant CLK_PERIOD : time := 10 ns;

  -- TB controls
  constant N_CASES_RAND    : integer := 30;
  constant WATCHDOG_CYCLES : integer := 250000;

  -- Must match your DUT generic default unless you changed it
  constant FLUSH_EXTRA_TB  : integer := 4;

  -- Contract bound for tile sum magnitude (in Q(W_FRAC_BITS)):
  -- For your Phase-1 plan, we target |tile_sum| <= 1.0 (about W_SCALE).
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

  -- Arithmetic shift-right for integer: floor(x/2^sh) for negatives.
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
      -- -ceil(|x|/d)
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

      -- contract flags
      variable in_contract  : boolean := true;
    begin
      exp_sum := golden_tile_sum_ashr(xv, wv);

      -- Contract check: tile sum should be bounded near +/-1.0 (Q12 ~ 4096)
      -- If you intentionally want to test saturation, you can flip this later.
      if abs_i(exp_sum) > W_SCALE_I then
        in_contract := false;
        report "TB NOTE(tile_sop): case_id=" & integer'image(case_id) &
               " is OUT-OF-CONTRACT (|exp_sum| > 1.0). exp_sum=" &
               integer'image(exp_sum) severity note;
      end if;

      -- tolerance (bring-up)
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

      for cyc in 0 to WATCHDOG_CYCLES loop

        -- Stall only by in_ready (ce stays 1)
        seed := next_seed(seed);
        if (seed mod 4) = 0 then
          in_ready <= '0';
        else
          in_ready <= '1';
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
        report "TB ERROR(tile_sop): done_tile did not assert. case_id=" & integer'image(case_id)
        severity failure;

      assert got_recon
        report "TB ERROR(tile_sop): sum reconstruction did not complete. case_id=" & integer'image(case_id)
        severity failure;

      assert sum_cnt >= N_DIGITS
        report "TB ERROR(tile_sop): too few sum valid digits. cnt=" & integer'image(sum_cnt) &
               " N_DIGITS=" & integer'image(N_DIGITS) &
               " case_id=" & integer'image(case_id)
        severity failure;

      -- Only enforce strict numeric match when the stimulus is in contract.
      if in_contract then
        assert abs_i(got_sum - exp_sum) <= tol
          report "TB ERROR(tile_sop): mismatch. case_id=" & integer'image(case_id) &
                 " got=" & integer'image(got_sum) &
                 " exp=" & integer'image(exp_sum) &
                 " diff=" & integer'image(got_sum - exp_sum) &
                 " tol=" & integer'image(tol)
          severity failure;

        report "TB OK(tile_sop): case_id=" & integer'image(case_id) &
               " got=" & integer'image(got_sum) &
               " exp=" & integer'image(exp_sum) &
               " diff=" & integer'image(got_sum - exp_sum) &
               " tol=" & integer'image(tol)
          severity note;
      else
        -- Out-of-contract: report only (you can add a saturation check later if desired)
        report "TB NOTE(tile_sop): out-of-contract observed. case_id=" & integer'image(case_id) &
               " got=" & integer'image(got_sum) &
               " exp=" & integer'image(exp_sum)
          severity note;
      end if;

    end procedure;

    variable xv_v : w_fixed_vec_t;
    variable wv_v : w_fixed_vec_t;

    constant HALF_X : integer := pow2_i(natural(W_FRAC_BITS-1)); -- +0.5
    constant ONE_W  : integer := pow2_i(natural(W_FRAC_BITS));   -- +1.0

    -- In-contract scaled weight for WEIGHT_RSHIFT=2 equivalent:
    constant ONE_W_DIV4 : integer := ONE_W / 4;                  -- +0.25

    variable k : integer;

  begin
    do_reset;

    -- ------------------------------------------------------------
    -- Directed cases
    -- ------------------------------------------------------------

    -- D0: all x=0 => sum=0
    make_all_const(xv_v, wv_v, 0, 123);
    run_tile_case(xv_v, wv_v, 11, 0);

    -- D1 (FIXED): x=0.5, w=0.25  => tile sum ~= 1.0 (in-contract)
    -- This matches your Phase-1 bounded tile sum idea.
    make_all_const(xv_v, wv_v, HALF_X, ONE_W_DIV4);
    run_tile_case(xv_v, wv_v, 22, 1);

    -- D2: alternating signs (forces negative products)
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

    -- ------------------------------------------------------------
    -- Random tiles (random stalls)
    -- ------------------------------------------------------------
    for k in 0 to N_CASES_RAND-1 loop
      make_random_tile(1000 + 97*k, xv_v, wv_v);
      run_tile_case(xv_v, wv_v, 2000 + 31*k, 100 + k);
    end loop;

    -- ------------------------------------------------------------
    -- Back-to-back tiles without global reset (SOFT_RESET proof)
    -- ------------------------------------------------------------
    make_random_tile(7777, xv_v, wv_v);
    run_tile_case(xv_v, wv_v, 8888, 9000);

    make_random_tile(9999, xv_v, wv_v);
    run_tile_case(xv_v, wv_v, 1111, 9001);

    report "TB PASSED: tb_msdf_tile_sop" severity note;
    wait;
  end process;

end architecture;
