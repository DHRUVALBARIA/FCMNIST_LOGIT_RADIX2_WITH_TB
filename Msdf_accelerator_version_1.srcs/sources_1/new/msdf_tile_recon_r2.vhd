library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

entity msdf_tile_recon_r2 is
  generic (
    N_DIGITS_G    : integer := N_DIGITS;
    CA_BITS_G     : integer := CA_BITS;
    OUT_BITS_G    : integer := TILE_RECON_BITS;

    -- Key addition:
    -- Shift the final reconstructed fixed-point value left by this many bits.
    -- Use SHIFT_LEFT_G = ONLINE_DELAY_MUL when reconstructing multiplier outputs.
    SHIFT_LEFT_G  : natural := 0
  );
  port (
    clk   : in  std_logic;
    rst_n : in  std_logic;   -- active-low synchronous reset
    ce    : in  std_logic;

    start : in  std_logic;   -- pulse: clear state and begin new tile
    step  : in  std_logic;   -- accept exactly one digit when '1'

    d_in  : in  sd_digit_t;

    tile_value_out : out signed(OUT_BITS_G-1 downto 0);
    tile_valid     : out std_logic;   -- 1-cycle pulse when tile_value_out updates
    busy           : out std_logic;   -- high while collecting digits
    digit_count    : out integer range 0 to N_DIGITS_G
  );
end entity;

architecture rtl of msdf_tile_recon_r2 is

  subtype ca_hat_t is signed(CA_BITS_G-1 downto 0);
  subtype out_t    is signed(OUT_BITS_G-1 downto 0);

  signal hat_r   : ca_hat_t := (others => '0');
  signal cnt_r   : integer range 0 to N_DIGITS_G := 0;
  signal busy_r  : std_logic := '0';

  signal out_r   : out_t := (others => '0');
  signal vld_r   : std_logic := '0';

  function hat_to_out(hat : ca_hat_t; len : integer) return out_t is
    variable x  : out_t;
    variable sh : integer;
    variable y  : out_t;
  begin
    if len <= 0 then
      return (others => '0');
    end if;

    x := resize(hat, OUT_BITS_G);

    -- recon = (hat / 2^len) * 2^W_FRAC_BITS = hat * 2^(W_FRAC_BITS - len)
    if len <= W_FRAC_BITS then
      sh := W_FRAC_BITS - len;
      y  := shift_left(x, sh);
    else
      sh := len - W_FRAC_BITS;
      y  := shift_right(x, sh);
    end if;

    -- Optional compensation for online-delay digit-index shift
    if SHIFT_LEFT_G = 0 then
      return y;
    else
      return shift_left(y, integer(SHIFT_LEFT_G));
    end if;
  end function;

begin

  tile_value_out <= out_r;
  tile_valid     <= vld_r;
  busy           <= busy_r;
  digit_count    <= cnt_r;

  assert N_DIGITS_G >= 1
    report "msdf_tile_recon_r2: N_DIGITS_G must be >= 1"
    severity failure;

  process(clk)
    variable di    : integer;
    variable hat_n : ca_hat_t;
    variable cnt_n : integer;
  begin
    if rising_edge(clk) then

      -- pulse-safe (never sticks, even if ce=0)
      vld_r <= '0';

      if rst_n = '0' then
        hat_r  <= (others => '0');
        cnt_r  <= 0;
        busy_r <= '0';
        out_r  <= (others => '0');
        vld_r  <= '0';

      elsif ce = '1' then

        if start = '1' then
          hat_r  <= (others => '0');
          cnt_r  <= 0;
          busy_r <= '1';
          out_r  <= (others => '0');
          vld_r  <= '0';

        elsif (busy_r = '1') and (step = '1') then

          -- synthesis translate_off
          assert is_valid_sd_digit(d_in)
            report "msdf_tile_recon_r2: illegal sd digit encoding"
            severity failure;
          -- synthesis translate_on

          di    := sd_to_integer(d_in);
          hat_n := shift_left(hat_r, 1) + to_signed(di, CA_BITS_G);
          cnt_n := cnt_r + 1;

          hat_r <= hat_n;
          cnt_r <= cnt_n;

          if cnt_r = (N_DIGITS_G - 1) then
            out_r  <= hat_to_out(hat_n, cnt_n);
            vld_r  <= '1';
            busy_r <= '0';
          end if;

        end if;
      end if;
    end if;
  end process;

end architecture;
