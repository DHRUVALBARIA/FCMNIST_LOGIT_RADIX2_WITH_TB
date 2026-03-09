library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;

entity msdf_mul_state_update_r2_sp_prec is
  port (
    W_in_ext  : in  w_ext_t;
    E_in_ext  : in  w_ext_t;
    init_mode : in  std_logic;

    W_out_ext : out w_ext_t;

    V_out_ext : out v_ext_t;  -- for debug
    v_hat     : out v_hat_t;
    p_digit   : out sd_digit_t
  );
end entity;

architecture rtl of msdf_mul_state_update_r2_sp_prec is
  signal v_ext   : v_ext_t   := W_EXT_ZERO;
  signal v_hat_s : v_hat_t   := (others => '0');
  signal p_raw   : sd_digit_t := SD_ZERO;
begin

  -- v_hat drives existing SELM digit select
  v_hat <= v_hat_s;

  u_digit_select : entity work.msdf_mul_digit_select_r2
    port map (
      v_hat   => v_hat_s,
      p_digit => p_raw
    );

  comb : process(W_in_ext, E_in_ext, init_mode, p_raw)
    variable v_wide   : signed(EXT_TOTAL_BITS downto 0);
    variable v_calc   : v_ext_t;
    variable p_use    : sd_digit_t;
    variable p_ext    : w_ext_t;
    variable w_wide   : signed(EXT_TOTAL_BITS downto 0);
    variable w_next   : w_ext_t;

    variable init_clean : boolean;
    variable init_zero  : boolean;
    variable p_clean    : boolean;
  begin
    -- compute v = 2*w + E in EXT
    v_wide := shift_left(resize(W_in_ext, EXT_TOTAL_BITS+1), 1) +
              resize(E_in_ext, EXT_TOTAL_BITS+1);
    v_calc := resize(v_wide, EXT_TOTAL_BITS);

    -- synthesis translate_off
    assert v_wide(EXT_TOTAL_BITS) = v_calc(EXT_TOTAL_BITS-1)
      report "msdf_mul_state_update_r2_sp_prec: OVERFLOW in v=2*w+E (EXT)."
      severity failure;
    -- synthesis translate_on

    v_ext   <= v_calc;
    V_out_ext <= v_calc;
    v_hat_s <= vext_to_vhat(v_calc);

    -- defaults: init behavior (p=0, w_next=v)
    p_use  := SD_ZERO;
    w_next := v_calc;

    init_clean := (init_mode = '0') or (init_mode = '1');
    init_zero  := (init_mode = '0');

    p_clean := is_valid_sd_digit(p_raw); -- includes meta check

    -- synthesis translate_off
    if (now > 0 ps) and (not init_clean) then
      assert false
        report "msdf_mul_state_update_r2_sp_prec: init_mode meta after t=0."
        severity failure;
    end if;

    if (now > 0 ps) and init_zero and (not p_clean) then
      assert false
        report "msdf_mul_state_update_r2_sp_prec: p_raw meta/illegal after t=0."
        severity failure;
    end if;
    -- synthesis translate_on

    if init_zero then
      if p_clean then
        p_use := p_raw;
      else
        p_use := SD_ZERO;
      end if;

      p_ext := sd_to_ext_fixed(p_use);

      w_wide := resize(v_calc, EXT_TOTAL_BITS+1) - resize(p_ext, EXT_TOTAL_BITS+1);
      w_next := resize(w_wide, EXT_TOTAL_BITS);

      -- synthesis translate_off
      assert w_wide(EXT_TOTAL_BITS) = w_next(EXT_TOTAL_BITS-1)
        report "msdf_mul_state_update_r2_sp_prec: OVERFLOW in w_next=v-p (EXT)."
        severity failure;
      -- synthesis translate_on
    end if;

    p_digit   <= p_use;
    W_out_ext <= w_next;
  end process;

end architecture;
