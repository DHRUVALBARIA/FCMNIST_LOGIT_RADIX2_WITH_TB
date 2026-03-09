library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

entity tb_msdf_tile_recon_r2 is
end entity;

architecture tb of tb_msdf_tile_recon_r2 is

  constant CLK_PERIOD : time := 10 ns;

  signal clk   : std_logic := '0';
  signal rst_n : std_logic := '0';
  signal ce    : std_logic := '1';

  signal start : std_logic := '0';
  signal step  : std_logic := '0';
  signal d_in  : sd_digit_t := SD_ZERO;

  signal tile_value : signed(TILE_RECON_BITS-1 downto 0);
  signal tile_valid : std_logic;
  signal busy       : std_logic;
  signal cnt        : integer range 0 to N_DIGITS;

  subtype ca_hat_t is signed(CA_BITS-1 downto 0);
  subtype out_t    is signed(TILE_RECON_BITS-1 downto 0);

  function hat_to_out_tb(hat : ca_hat_t; len : integer) return out_t is
    variable x  : out_t;
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

  dut : entity work.msdf_tile_recon_r2
    generic map (
      N_DIGITS_G => N_DIGITS,
      CA_BITS_G  => CA_BITS,
      OUT_BITS_G => TILE_RECON_BITS
    )
    port map (
      clk            => clk,
      rst_n          => rst_n,
      ce             => ce,
      start          => start,
      step           => step,
      d_in           => d_in,
      tile_value_out => tile_value,
      tile_valid     => tile_valid,
      busy           => busy,
      digit_count    => cnt
    );

  stim : process
    variable hat_ref  : ca_hat_t := (others => '0');
    variable cnt_ref  : integer := 0;
    variable busy_ref : std_logic := '0';

    procedure tick is
    begin
      wait until rising_edge(clk);
      wait for 1 ns;
    end procedure;

  begin
    rst_n <= '0';
    tick; tick;
    rst_n <= '1';
    tick;

    -- +0.5: digits [+1, 0, 0, ...]
    start <= '1'; step <= '0'; d_in <= SD_ZERO;
    tick;
    start <= '0';

    busy_ref := '1';
    hat_ref  := (others => '0');
    cnt_ref  := 0;

    d_in <= SD_POS1; step <= '1'; tick; step <= '0';
    hat_ref := shift_left(hat_ref,1) + to_signed(1, CA_BITS);
    cnt_ref := cnt_ref + 1;

    for k in 1 to (N_DIGITS-1) loop
      d_in <= SD_ZERO; step <= '1'; tick; step <= '0';
      hat_ref := shift_left(hat_ref,1) + to_signed(0, CA_BITS);
      cnt_ref := cnt_ref + 1;
    end loop;

    assert tile_valid = '1'
      report "TB ERROR: expected tile_valid at end of N_DIGITS"
      severity failure;

    assert tile_value = hat_to_out_tb(hat_ref, cnt_ref)
      report "TB ERROR: tile_value mismatch for +0.5"
      severity failure;

    report "TB PASSED: tb_msdf_tile_recon_r2"
      severity note;

    wait;
  end process;

end architecture;
