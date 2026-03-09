library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;

entity msdf_mul_sp_term_gen is
  port (
    x_digit_in  : in  sd_digit_t;  -- serial digit
    y_vector_in : in  w_fixed_t;   -- static weight in base precision
    E_out       : out w_ext_t      -- EXT precision term
  );
end entity;

architecture rtl of msdf_mul_sp_term_gen is
  constant DELTA : natural := ONLINE_DELAY_MUL;
begin
  process(x_digit_in, y_vector_in)
    variable y_ext      : w_ext_t;
    variable term_ext   : w_ext_t;
    variable x_clean    : boolean;
    variable x_i        : integer;
  begin
    y_ext    := w_to_ext(y_vector_in);
    term_ext := W_EXT_ZERO;

    -- Avoid calling strict sd_to_integer unless digit is clean+legal
    x_clean := is_valid_sd_digit(x_digit_in);

    -- synthesis translate_off
    if (now > 0 ps) and (not x_clean) then
      assert false
        report "msdf_mul_sp_term_gen: x_digit_in is meta/illegal after t=0."
        severity failure;
    end if;
    -- synthesis translate_on

    if x_clean then
      x_i := sd_to_integer(x_digit_in);
      case x_i is
        when  1 => term_ext :=  y_ext;
        when -1 => term_ext := -y_ext;
        when others => term_ext := W_EXT_ZERO;
      end case;
    else
      term_ext := W_EXT_ZERO;
    end if;

    -- Apply r^-delta as numeric division; keep EXT scale.
    E_out <= w_ext_t(div_pow2_trunc0_any(term_ext, DELTA));
  end process;
end architecture;
