library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

entity tb_msdf_mul_array_tile is
end entity;

architecture tb of tb_msdf_mul_array_tile is
  constant CLK_PERIOD  : time := 10 ns;
  constant N_DIGITS_TB : integer := N_DIGITS;

  constant FLUSH_TB : integer := ONLINE_DELAY_MUL + 2;

  type tile_recon_vec_t is array (0 to TILE_LEN-1) of tile_recon_t;
  type bool_vec_t      is array (0 to TILE_LEN-1) of boolean;

  signal clk   : std_logic := '0';
  signal rst_n : std_logic := '0';
  signal ce    : std_logic := '1';

  signal step_gen   : std_logic := '0';
  signal step_mul   : std_logic := '0';
  signal flush_left : integer range 0 to 64 := 0;

  signal gen_start : std_logic := '0';
  signal x_in      : w_fixed_t := (others => '0');
  signal d_gen     : sd_digit_t := SD_ZERO;
  signal v_gen     : std_logic := '0';
  signal done_gen  : std_logic := '0';
  signal busy_gen  : std_logic := '0';

  signal d_to_mul : sd_digit_t := SD_ZERO;

  signal x_digits_in  : sd_digit_vec_t := (others => SD_ZERO);
  signal w_weights_in : w_fixed_vec_t  := (others => (others => '0'));
  signal p_digits_out : sd_digit_vec_t := (others => SD_ZERO);
  signal p_valid_out  : sl_vec_t       := (others => '0');

  signal x_rec_out : tile_recon_t := (others => '0');
  signal x_rec_vld : std_logic := '0';

  signal p_rec_out : tile_recon_vec_t := (others => (others => '0'));
  signal p_rec_vld : sl_vec_t := (others => '0');

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

  function shl_i(x : integer; k : natural) return integer is
  begin
    return x * pow2_i(k);
  end function;

  -- shift right with trunc toward 0 (not floor)
  function shr_trunc0_i(x : integer; k : natural) return integer is
    variable d : integer := pow2_i(k);
  begin
    if k = 0 then
      return x;
    end if;

    if x >= 0 then
      return x / d;
    else
      return -((-x) / d);
    end if;
  end function;

  type comp_mode_t is (COMP_NONE, COMP_SHL, COMP_SHR);

begin
  clk <= not clk after CLK_PERIOD/2;

  -- Print once (confirms exactly what constants this sim is using)
  p_banner : process
  begin
    wait for 0 ns;
    report "TB CONST: ONLINE_DELAY_MUL=" & integer'image(ONLINE_DELAY_MUL) &
           " W_FRAC_BITS=" & integer'image(W_FRAC_BITS) &
           " N_DIGITS_TB=" & integer'image(N_DIGITS_TB)
      severity note;
    wait;
  end process;

  gen_fanout : for i in 0 to TILE_LEN-1 generate
  begin
    x_digits_in(i) <= d_to_mul;
  end generate;

  process(clk)
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        flush_left <= 0;
      elsif ce = '1' then
        if gen_start = '1' then
          flush_left <= 0;
        elsif done_gen = '1' then
          flush_left <= FLUSH_TB;
        elsif (flush_left > 0) then
          flush_left <= flush_left - 1;
        end if;
      end if;
    end if;
  end process;

  step_mul <= '1' when ((v_gen = '1') or (flush_left > 0)) else '0';
  d_to_mul <= d_gen when (v_gen = '1') else SD_ZERO;

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

  u_arr : entity work.msdf_mul_array_tile
    port map (
      clk          => clk,
      rst_n        => rst_n,
      ce           => ce,
      step_en      => step_mul,
      x_digits_in  => x_digits_in,
      w_weights_in => w_weights_in,
      p_digits_out => p_digits_out,
      p_valid_out  => p_valid_out
    );

  -- x recon: SHIFT_LEFT_G = 0
  u_xrec : entity work.msdf_tile_recon_r2
    generic map (
      N_DIGITS_G   => N_DIGITS_TB,
      CA_BITS_G    => CA_BITS,
      OUT_BITS_G   => TILE_RECON_BITS,
      SHIFT_LEFT_G => 0
    )
    port map (
      clk            => clk,
      rst_n          => rst_n,
      ce             => ce,
      start          => gen_start,
      step           => v_gen,
      d_in           => d_gen,
      tile_value_out => x_rec_out,
      tile_valid     => x_rec_vld,
      busy           => open,
      digit_count    => open
    );

  -- product recon for every lane: SHIFT_LEFT_G = 0 (RAW stream)
  gen_prec : for i in 0 to TILE_LEN-1 generate
  begin
    u_prec : entity work.msdf_tile_recon_r2
      generic map (
        N_DIGITS_G   => N_DIGITS_TB,
        CA_BITS_G    => CA_BITS,
        OUT_BITS_G   => TILE_RECON_BITS,
        SHIFT_LEFT_G => 0
      )
      port map (
        clk            => clk,
        rst_n          => rst_n,
        ce             => ce,
        start          => gen_start,
        step           => p_valid_out(i),
        d_in           => p_digits_out(i),
        tile_value_out => p_rec_out(i),
        tile_valid     => p_rec_vld(i),
        busy           => open,
        digit_count    => open
      );
  end generate;

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
      step_gen  <= '0';
      gen_start <= '0';
      x_in      <= (others => '0');
      tick; tick;
      rst_n <= '1';
      tick;
    end procedure;

    procedure set_weights_basic is
      constant ONE_W  : w_fixed_t := to_signed( W_SCALE, W_TOTAL_BITS);
      constant ZERO_W : w_fixed_t := (others => '0');
      constant NEG_W  : w_fixed_t := to_signed(-W_SCALE, W_TOTAL_BITS);
    begin
      w_weights_in(0) <= ONE_W;
      w_weights_in(1) <= ZERO_W;
      w_weights_in(2) <= NEG_W;

      for i in 3 to TILE_LEN-1 loop
        w_weights_in(i) <= ONE_W;
      end loop;
    end procedure;

    procedure run_case_nostall(constant xval : in integer) is
      variable tol : integer;

      variable got_x    : boolean := false;
      variable got_lane : bool_vec_t := (others => false);
      variable got_cnt  : integer := 0;

      variable x_cap : tile_recon_t := (others => '0');
      variable p_cap : tile_recon_vec_t := (others => (others => '0'));

      variable xi   : integer;
      variable p0   : integer;

      variable mode : comp_mode_t := COMP_NONE;

      variable cyc  : integer;
      variable i    : integer;

      variable pi_raw : integer;
      variable pi_cmp : integer;
      variable expi   : integer;
      variable diff   : integer;
    begin
      -- Reset before each case (prevents multiplier state leaking between cases)
      do_reset;
      set_weights_basic;

      if W_FRAC_BITS > N_DIGITS_TB then
        tol := pow2_i(natural(W_FRAC_BITS - N_DIGITS_TB)) + 6;
      else
        tol := 6;
      end if;

      x_in      <= to_signed(xval, W_TOTAL_BITS);
      gen_start <= '1';
      step_gen  <= '0';
      tick;

      gen_start <= '0';
      step_gen  <= '1';

      for cyc in 0 to 25000 loop
        tick;

        if x_rec_vld = '1' then
          got_x := true;
          x_cap := x_rec_out;
        end if;

        for i in 0 to TILE_LEN-1 loop
          if p_rec_vld(i) = '1' then
            if not got_lane(i) then
              got_lane(i) := true;
              got_cnt := got_cnt + 1;
              p_cap(i) := p_rec_out(i);
            end if;
          end if;
        end loop;

        if got_x and (got_cnt = TILE_LEN) then
          exit;
        end if;
      end loop;

      step_gen <= '0';
      tick;

      assert got_x
        report "TB ERROR(mul_array): input recon did not complete"
        severity failure;

      assert got_cnt = TILE_LEN
        report "TB ERROR(mul_array): not all lane reconstructions completed"
        severity failure;

      xi := to_integer(x_cap);
      p0 := to_integer(p_cap(0));

      -- Auto-detect which mapping your multiplier stream actually uses
      if abs_i(p0 - xi) <= tol then
        mode := COMP_NONE;
      elsif abs_i(shl_i(p0, natural(ONLINE_DELAY_MUL)) - xi) <= tol then
        mode := COMP_SHL;
      elsif abs_i(shr_trunc0_i(p0, natural(ONLINE_DELAY_MUL)) - xi) <= tol then
        mode := COMP_SHR;
      else
        report "TB ERROR(mul_array): cannot calibrate DELTA mapping. " &
               "x=" & integer'image(xi) &
               " p0_raw=" & integer'image(p0) &
               " p0<<DELTA=" & integer'image(shl_i(p0, natural(ONLINE_DELAY_MUL))) &
               " p0>>DELTA=" & integer'image(shr_trunc0_i(p0, natural(ONLINE_DELAY_MUL)))
          severity failure;
      end if;

      if mode = COMP_NONE then
        report "TB CAL: product stream needs NO compensation" severity note;
      elsif mode = COMP_SHL then
        report "TB CAL: product stream needs SHIFT_LEFT(DELTA)" severity note;
      else
        report "TB CAL: product stream needs SHIFT_RIGHT(DELTA)" severity note;
      end if;

      -- Now check all lanes using the detected mapping
      for i in 0 to TILE_LEN-1 loop
        pi_raw := to_integer(p_cap(i));

        if mode = COMP_NONE then
          pi_cmp := pi_raw;
        elsif mode = COMP_SHL then
          pi_cmp := shl_i(pi_raw, natural(ONLINE_DELAY_MUL));
        else
          pi_cmp := shr_trunc0_i(pi_raw, natural(ONLINE_DELAY_MUL));
        end if;

        if i = 0 then
          expi := xi;
        elsif i = 1 then
          expi := 0;
        elsif i = 2 then
          expi := -xi;
        else
          expi := xi;
        end if;

        diff := pi_cmp - expi;

        assert abs_i(diff) <= tol
          report "TB ERROR(mul_array): lane" & integer'image(i) &
                 " mismatch. x=" & integer'image(xi) &
                 " p_raw=" & integer'image(pi_raw) &
                 " p_cmp=" & integer'image(pi_cmp) &
                 " exp=" & integer'image(expi) &
                 " diff=" & integer'image(diff)
          severity failure;
      end loop;
    end procedure;

    constant HALF_X : integer := pow2_i(natural(W_FRAC_BITS-1)); -- +0.5
    constant QTR_X  : integer := pow2_i(natural(W_FRAC_BITS-2)); -- +0.25
  begin
    run_case_nostall(HALF_X);
    run_case_nostall(-QTR_X);

    report "TB PASSED: tb_msdf_mul_array_tile" severity note;
    wait;
  end process;

end architecture;
