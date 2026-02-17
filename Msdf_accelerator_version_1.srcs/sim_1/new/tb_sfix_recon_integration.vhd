library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

entity tb_sfix_recon_integration is
end entity;

architecture tb of tb_sfix_recon_integration is
  constant CLK_PERIOD : time := 10 ns;
  constant N_DIGITS_TB : integer := N_DIGITS;

  subtype hat_t   is signed(CA_BITS-1 downto 0);
  subtype recon_t is signed(TILE_RECON_BITS-1 downto 0);

  signal clk   : std_logic := '0';
  signal rst_n : std_logic := '0';
  signal ce    : std_logic := '1';

  -- Stepper
  signal tile_start : std_logic := '0';
  signal in_ready   : std_logic := '0';
  signal active     : std_logic;
  signal step_fire  : std_logic;

  -- Accepted step enable (combinational)
  signal step_en  : std_logic;

  -- One-cycle delayed step for recon alignment
  signal step_d1  : std_logic := '0';

  -- Generator
  signal x_in      : w_fixed_t := (others => '0');
  signal gen_start : std_logic := '0';
  signal d_gen     : sd_digit_t;
  signal v_gen     : std_logic;
  signal done_gen  : std_logic;
  signal busy_gen  : std_logic;

  -- Reconstructor
  signal recon_out : recon_t;
  signal recon_vld : std_logic;
  signal busy_rec  : std_logic;

  function pow2_i(k : natural) return integer is
    variable v : integer := 1;
    variable j : natural;
  begin
    for j in 1 to k loop
      v := v * 2;
    end loop;
    return v;
  end function;

  -- Must match msdf_tile_recon_r2 behavior (arith shift for negatives)
  function hat_to_out_tb(hat : hat_t; len : integer) return recon_t is
    variable x  : recon_t;
    variable sh : integer;
  begin
    if len <= 0 then
      return (others => '0');
    end if;

    x := resize(hat, TILE_RECON_BITS);

    if len <= W_FRAC_BITS then
      sh := W_FRAC_BITS - len;
      return shift_left(x, sh);
    else
      sh := len - W_FRAC_BITS;
      return shift_right(x, sh);
    end if;
  end function;

begin
  clk <= not clk after CLK_PERIOD/2;

  -- Accepted step enable for synchronous blocks must be true BEFORE the edge
  step_en <= '1' when (ce = '1' and active = '1' and in_ready = '1') else '0';

  -- Delay step_en by 1 cycle for recon, so recon consumes the digit generated on the previous step
  process(clk)
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        step_d1 <= '0';
      else
        step_d1 <= step_en;
      end if;
    end if;
  end process;

  u_stepper : entity work.msdf_tile_stepper
    generic map (
      N_DIGITS_G => N_DIGITS_TB,
      FLUSH_G    => 0
    )
    port map (
      clk         => clk,
      rst_n       => rst_n,
      ce          => ce,
      tile_start  => tile_start,
      in_ready    => in_ready,
      active      => active,
      step_fire   => step_fire,
      digit_idx   => open,
      step_count  => open,
      done_digits => open,
      done_tile   => open
    );

  u_gen : entity work.sfix_to_msdf2_stream_r2
    generic map (
      N_DIGITS_G => N_DIGITS_TB
    )
    port map (
      clk         => clk,
      rst_n       => rst_n,
      ce          => ce,
      start       => gen_start,
      step        => step_en,     -- generator steps on step_en
      x_in        => x_in,
      digit_out   => d_gen,
      valid       => v_gen,
      done        => done_gen,
      busy        => busy_gen,
      digit_count => open
    );

  u_rec : entity work.msdf_tile_recon_r2
    generic map (
      N_DIGITS_G => N_DIGITS_TB,
      CA_BITS_G  => CA_BITS,
      OUT_BITS_G => TILE_RECON_BITS
    )
    port map (
      clk            => clk,
      rst_n          => rst_n,
      ce             => ce,
      start          => tile_start,
      step           => step_d1,   -- recon steps one cycle later
      d_in           => d_gen,
      tile_value_out => recon_out,
      tile_valid     => recon_vld,
      busy           => busy_rec,
      digit_count    => open
    );

  stim : process
    procedure tick is
    begin
      wait until rising_edge(clk);
      wait for 1 ns;
    end procedure;

    procedure do_reset is
    begin
      rst_n <= '0';
      ce <= '1';
      tile_start <= '0';
      gen_start  <= '0';
      in_ready   <= '0';
      x_in       <= (others => '0');
      tick; tick;
      rst_n <= '1';
      tick;
    end procedure;

    procedure run_case(constant xval : in integer) is
      variable hat_ref : hat_t := (others => '0');
      variable len_ref : integer := 0;
      variable exp_out : recon_t := (others => '0');

      variable seed  : integer := 19;
      variable cyc   : integer;

      variable tol     : integer;
      variable xin_i   : integer;
      variable recon_i : integer;
      variable diff    : integer;

      -- Hold pre-edge values so reference uses the same digit recon consumes
      variable step_d1_hold : std_logic;
      variable d_hold       : sd_digit_t;
    begin
      x_in <= to_signed(xval, W_TOTAL_BITS);

      -- Start stepper + generator + recon together
      gen_start  <= '1';
      tile_start <= '1';
      in_ready   <= '0';
      tick;

      gen_start  <= '0';
      tile_start <= '0';

      for cyc in 0 to 8000 loop
        seed := (seed * 17 + 5) mod 65521;
        if (seed mod 4) = 0 then
          in_ready <= '0';
        else
          in_ready <= '1';
        end if;

        -- Capture what recon will see at the upcoming edge
        step_d1_hold := step_d1;
        d_hold       := d_gen;

        tick;

        -- When step_d1 was high BEFORE the edge, recon consumed d_hold on that edge
        if step_d1_hold = '1' then
          hat_ref := shift_left(hat_ref, 1) + to_signed(sd_to_integer(d_hold), CA_BITS);
          len_ref := len_ref + 1;

          if len_ref = N_DIGITS_TB then
            exp_out := hat_to_out_tb(hat_ref, len_ref);
          end if;
        end if;

        if recon_vld = '1' then
          assert len_ref = N_DIGITS_TB
            report "TB ERROR(integration): tile_valid asserted early/late"
            severity failure;

          assert recon_out = exp_out
            report "TB ERROR(integration): recon_out mismatch vs reference"
            severity failure;

          -- Sanity: recon should be close to x in fixed domain (expected trunc error)
          xin_i   := xval;
          recon_i := to_integer(recon_out);
          diff := xin_i - recon_i;
          if diff < 0 then diff := -diff; end if;

          if W_FRAC_BITS > N_DIGITS_TB then
            tol := pow2_i(natural(W_FRAC_BITS - N_DIGITS_TB)) + 1;
          else
            tol := 2;
          end if;

          assert diff <= tol
            report "TB ERROR(integration): recon not close to x (diff too large)"
            severity failure;

          exit;
        end if;
      end loop;

      assert recon_vld = '1'
        report "TB ERROR(integration): did not complete within cycle budget"
        severity failure;
    end procedure;

    constant HALF_X : integer := pow2_i(natural(W_FRAC_BITS-1));
    constant QTR_X  : integer := pow2_i(natural(W_FRAC_BITS-2));
  begin
    do_reset;

    run_case( HALF_X );
    run_case(-HALF_X );
    run_case( QTR_X  );
    run_case( 1234   );
    run_case(-987    );

    report "TB PASSED: tb_sfix_recon_integration" severity note;
    wait;
  end process;

end architecture;
