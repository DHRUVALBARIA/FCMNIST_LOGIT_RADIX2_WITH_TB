library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;

entity new_msdf_core_seq_r2_dbg_clean is
  port (
    clk         : in  std_logic;
    rst_n       : in  std_logic;
    ce          : in  std_logic;

    x_digit_in  : in  sd_digit_t;   -- Streamed input (Activation)
    y_vector_in : in  w_fixed_t;    -- STATIC Parallel input (Weight)
    in_valid    : in  std_logic;

    p_digit_out : out sd_digit_t;
    out_valid   : out std_logic;

    -- debug taps (base precision outputs)
    dbg_w_reg     : out w_fixed_t;
    dbg_w_next    : out w_fixed_t;
    dbg_h1_term   : out w_fixed_t;
    dbg_v_term    : out v_fixed_t;
    dbg_v_hat     : out v_hat_t;
    dbg_p_raw     : out sd_digit_t;
    dbg_init_mode : out std_logic
  );
end entity;

architecture rtl of new_msdf_core_seq_r2_dbg_clean is

  constant DELTA_C : natural := ONLINE_DELAY_MUL;

  signal W_reg_ext  : w_ext_t := W_EXT_ZERO;
  signal W_next_ext : w_ext_t := W_EXT_ZERO;
  signal E_term_ext : w_ext_t := W_EXT_ZERO;

  -- Init counter (counts accepted steps, saturates at DELTA_C)
  signal init_cnt_q : natural range 0 to DELTA_C := 0;
  signal init_mode  : std_logic := '1';

  signal p_raw         : sd_digit_t := SD_ZERO;
  signal p_out_reg     : sd_digit_t := SD_ZERO;
  signal out_valid_reg : std_logic  := '0';

  signal V_ext_s  : v_ext_t := W_EXT_ZERO;
  signal v_hat_s  : v_hat_t := (others => '0');

begin

  init_mode <= '1' when (init_cnt_q < DELTA_C) else '0';

  -- Precision-safe serial-parallel term generator (EXT output)
  u_term_gen : entity work.msdf_mul_sp_term_gen
    port map (
      x_digit_in  => x_digit_in,
      y_vector_in => y_vector_in,
      E_out       => E_term_ext
    );

  -- EXT precision state update
  u_state_ext : entity work.msdf_mul_state_update_r2_sp_prec
    port map (
      W_in_ext  => W_reg_ext,
      E_in_ext  => E_term_ext,
      init_mode => init_mode,

      W_out_ext => W_next_ext,
      V_out_ext => V_ext_s,
      v_hat     => v_hat_s,
      p_digit   => p_raw
    );

  process(clk)
    variable step_en : boolean;
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        W_reg_ext     <= W_EXT_ZERO;
        init_cnt_q    <= 0;
        p_out_reg     <= SD_ZERO;
        out_valid_reg <= '0';
      else
        out_valid_reg <= '0';  -- default

        step_en := (ce = '1') and (in_valid = '1');

        if step_en then
          W_reg_ext <= W_next_ext;

          if init_cnt_q < DELTA_C then
            init_cnt_q <= init_cnt_q + 1;
          end if;

          if init_mode = '0' then
            p_out_reg     <= p_raw;
            out_valid_reg <= '1';
          else
            p_out_reg     <= SD_ZERO;
            out_valid_reg <= '0';
          end if;
        end if;
      end if;
    end if;
  end process;

  p_digit_out <= p_out_reg;
  out_valid   <= out_valid_reg;

  -- Debug taps: truncate EXT -> base precision for visibility
  dbg_w_reg     <= ext_to_w_trunc0(W_reg_ext);
  dbg_w_next    <= ext_to_w_trunc0(W_next_ext);
  dbg_h1_term   <= ext_to_w_trunc0(E_term_ext);
  dbg_v_term    <= ext_to_w_trunc0(V_ext_s);
  dbg_v_hat     <= v_hat_s;
  dbg_p_raw     <= p_raw;
  dbg_init_mode <= init_mode;

end architecture;
