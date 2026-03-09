library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

entity tb_msdf_add_tree_r2 is
end entity;

architecture tb of tb_msdf_add_tree_r2 is

  constant CLK_PERIOD  : time := 10 ns;
  constant N_DIGITS_TB : integer := N_DIGITS;

  function pow2_i(k : natural) return integer is
    variable v : integer := 1;
    variable j : natural;
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

  function ilog2_pow2(n : positive) return natural is
    variable v : natural := 1;
    variable d : natural := 0;
  begin
    while v < natural(n) loop
      v := v * 2;
      d := d + 1;
    end loop;
    return d;
  end function;

  constant TREE_DEPTH_C : integer := integer(ilog2_pow2(TILE_LEN));

  -- Drain latency for depth-D online adder tree (accepted steps)
  constant FLUSH_TB     : integer := (ONLINE_DELAY_ADD * TREE_DEPTH_C) + 2;

  -- Keep this printed as info (do NOT apply unless you prove you need it)
  constant SHIFT_SUM_C  : natural := natural(ONLINE_DELAY_ADD * TREE_DEPTH_C);

  signal clk   : std_logic := '0';
  signal rst_n : std_logic := '0';
  signal ce    : std_logic := '1';

  -- Generator control
  signal gen_start : std_logic := '0';
  signal step_gen  : std_logic := '0';
  signal x_in      : w_fixed_t := (others => '0');

  signal d_gen    : sd_digit_t := SD_ZERO;
  signal v_gen    : std_logic  := '0';
  signal done_gen : std_logic  := '0';
  signal busy_gen : std_logic  := '0';

  -- Flush control
  signal flush_left : integer range 0 to 256 := 0;

  -- Tree inputs (fanout)
  signal in_digits : sd_digit_vec_t := (others => SD_ZERO);
  signal in_valids : sl_vec_t       := (others => '0');

  -- Tree output
  signal z_d : sd_digit_t := SD_ZERO;
  signal z_v : std_logic  := '0';

  -- Reconstruct x
  signal x_rec_out : tile_recon_t := (others => '0');
  signal x_rec_vld : std_logic    := '0';

  -- Reconstruct sum
  signal s_rec_out : tile_recon_t := (others => '0');
  signal s_rec_vld : std_logic    := '0';

  -- Simple LFSR for random STEP stalls (NOT ce stalls)
  signal lfsr_q : unsigned(7 downto 0) := x"A7";

begin

  clk <= not clk after CLK_PERIOD/2;

  -- Banner: print numeric contract once
  p_banner : process
  begin
    wait for 0 ns;
    report "TB CONST: ONLINE_DELAY_ADD=" & integer'image(ONLINE_DELAY_ADD) &
           " W_FRAC_BITS=" & integer'image(W_FRAC_BITS) &
           " N_DIGITS_TB=" & integer'image(N_DIGITS_TB) &
           " TILE_LEN=" & integer'image(TILE_LEN) &
           " TREE_DEPTH=" & integer'image(TREE_DEPTH_C) &
           " FLUSH_TB=" & integer'image(FLUSH_TB) &
           " SHIFT_SUM_C=" & integer'image(integer(SHIFT_SUM_C))
           severity note;

    report "TB CAL: add-tree output stream expects NO recon compensation (latency only). SHIFT_LEFT_G=0"
           severity note;
    wait;
  end process;

  -- LFSR update (always clocks)
  process(clk)
    variable fb   : std_logic;
    variable fb_u : unsigned(0 downto 0);
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        lfsr_q <= x"A7";
      else
        fb := std_logic(lfsr_q(7) xor lfsr_q(5) xor lfsr_q(4) xor lfsr_q(3));
        fb_u(0) := fb;
        lfsr_q <= lfsr_q(6 downto 0) & fb_u;
      end if;
    end if;
  end process;

  -- Flush counter: after done_gen, keep stepping tree for FLUSH_TB cycles
  process(clk)
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        flush_left <= 0;
      else
        if gen_start = '1' then
          flush_left <= 0;
        elsif done_gen = '1' then
          flush_left <= FLUSH_TB;
        elsif flush_left > 0 then
          flush_left <= flush_left - 1;
        end if;
      end if;
    end if;
  end process;

  -- Feed digits/valids into tree
  gen_fanout : for i in 0 to TILE_LEN-1 generate
  begin
    in_digits(i) <= d_gen when (v_gen = '1') else SD_ZERO;
    in_valids(i) <= '1' when ((v_gen = '1') or (flush_left > 0)) else '0';
  end generate;

  -- Generator
  u_gen : entity work.sfix_to_msdf2_stream_r2
    generic map (
      N_DIGITS_G => N_DIGITS_TB
    )
    port map (
      clk         => clk,
      rst_n       => rst_n,
      ce          => ce,
      start       => gen_start,
      step        => step_gen,
      x_in        => x_in,
      digit_out   => d_gen,
      valid       => v_gen,
      done        => done_gen,
      busy        => busy_gen,
      digit_count => open
    );

  -- Add tree
  u_tree : entity work.msdf_add_tree_r2
    port map (
      clk          => clk,
      rst_n        => rst_n,
      ce           => ce,
      in_digits_in => in_digits,
      in_valid_in  => in_valids,
      z_digit_out  => z_d,
      z_valid_out  => z_v
    );

  -- Reconstruct x from generator stream (ce forced to '1')
  u_xrec : entity work.msdf_tile_recon_r2
    generic map (
      N_DIGITS_G    => N_DIGITS_TB,
      CA_BITS_G     => CA_BITS,
      OUT_BITS_G    => TILE_RECON_BITS,
      SHIFT_LEFT_G  => 0
    )
    port map (
      clk            => clk,
      rst_n          => rst_n,
      ce             => '1',
      start          => gen_start,
      step           => v_gen,
      d_in           => d_gen,
      tile_value_out => x_rec_out,
      tile_valid     => x_rec_vld,
      busy           => open,
      digit_count    => open
    );

  -- Reconstruct sum from tree output stream
  -- IMPORTANT: no compensation here
  u_srec : entity work.msdf_tile_recon_r2
    generic map (
      N_DIGITS_G    => N_DIGITS_TB,
      CA_BITS_G     => CA_BITS,
      OUT_BITS_G    => TILE_RECON_BITS,
      SHIFT_LEFT_G  => 0
    )
    port map (
      clk            => clk,
      rst_n          => rst_n,
      ce             => '1',
      start          => gen_start,
      step           => z_v,
      d_in           => z_d,
      tile_value_out => s_rec_out,
      tile_valid     => s_rec_vld,
      busy           => open,
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
      rst_n     <= '0';
      ce        <= '1';
      gen_start <= '0';
      step_gen  <= '0';
      x_in      <= (others => '0');
      tick; tick;
      rst_n <= '1';
      tick;
    end procedure;

    procedure run_case(constant xval : in integer) is
      variable got_x   : boolean := false;
      variable got_sum : boolean := false;

      variable x_cap   : integer := 0;
      variable s_cap   : integer := 0;

      variable expi    : integer := 0;
      variable diff    : integer := 0;
      variable tol     : integer := 0;

      variable cyc     : integer;
    begin
      -- Reset between cases: tree adders are stateful
      do_reset;

      if W_FRAC_BITS > N_DIGITS_TB then
        tol := pow2_i(natural(W_FRAC_BITS - N_DIGITS_TB)) + 10;
      else
        tol := 10;
      end if;

      x_in      <= to_signed(xval, W_TOTAL_BITS);
      gen_start <= '1';
      step_gen  <= '0';
      ce        <= '1';
      tick;

      gen_start <= '0';

      for cyc in 0 to 50000 loop
        ce <= '1';

        -- During flush: do not stall (must drain)
        if flush_left > 0 then
          step_gen <= '1';
        else
          step_gen <= std_logic(lfsr_q(0));
        end if;

        tick;

        if x_rec_vld = '1' then
          got_x := true;
          x_cap := to_integer(x_rec_out);
        end if;

        if s_rec_vld = '1' then
          got_sum := true;
          s_cap := to_integer(s_rec_out);
        end if;

        if got_x and got_sum then
          exit;
        end if;
      end loop;

      step_gen <= '0';
      ce       <= '1';
      tick;

      assert got_x
        report "TB ERROR(add_tree): x reconstruction did not complete"
        severity failure;

      assert got_sum
        report "TB ERROR(add_tree): sum reconstruction did not complete"
        severity failure;

      expi := x_cap * TILE_LEN;
      diff := s_cap - expi;

      assert abs_i(diff) <= tol
        report "TB ERROR(add_tree): mismatch. x=" & integer'image(x_cap) &
               " sum=" & integer'image(s_cap) &
               " exp=" & integer'image(expi) &
               " diff=" & integer'image(diff)
        severity failure;
    end procedure;

    constant X1 : integer := pow2_i(natural(W_FRAC_BITS - 5));   -- +1/32
    constant X2 : integer := -pow2_i(natural(W_FRAC_BITS - 6));  -- -1/64
  begin
    -- Each run_case does its own reset (on purpose)
    run_case(X1);
    run_case(X2);

    report "TB PASSED: tb_msdf_add_tree_r2" severity note;
    wait;
  end process;

end architecture;
