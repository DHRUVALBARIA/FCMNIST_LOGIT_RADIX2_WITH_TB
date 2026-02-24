--library ieee;
--use ieee.std_logic_1164.all;
--use ieee.numeric_std.all;

--use work.msdf_mul_pkg_r2_core.all;

--entity msdf_mul_digit_select_r2 is
--  port (
--    v_hat   : in  v_hat_t;     -- signed estimate (scaled by 2^t)
--    p_digit : out sd_digit_t    -- {-1,0,+1}
--  );
--end entity;

--architecture rtl of msdf_mul_digit_select_r2 is
--begin
--  comb : process(v_hat)
--    variable vh_slv : std_logic_vector(v_hat'range);
--    variable vh_s   : signed(v_hat'range);
--    variable clean  : boolean;
--  begin
--    vh_slv := std_logic_vector(v_hat);

--    -- Default deterministic output (also covers meta cases safely)
--    p_digit <= SD_ZERO;

--    clean := is_01_only(vh_slv);

--    -- =========================
--    -- Simulation-only checking
--    -- =========================
--    -- synthesis translate_off
----    if not clean then
----      -- Time-0 / first-delta comb settle is allowed; do NOT kill sim.
----      if now = 0 ps then
----        assert false
----          report "msdf_mul_digit_select_r2: v_hat contains meta at time 0 (delta-settle). Ignoring at t=0."
----          severity warning;
----      else
----        assert false
----          report "msdf_mul_digit_select_r2: v_hat contains meta (U/X/Z) after t=0 -> REAL upstream bug."
----          severity failure;
----      end if;
----    end if;
--        if not clean then
--              -- FIX: Silently ignore Time-0 / first-delta comb settle meta values.
--              -- We only assert a failure if meta states exist AFTER startup.
--              if now > 0 ps then
--                assert false
--                  report "msdf_mul_digit_select_r2: v_hat contains meta (U/X/Z) after t=0 -> REAL upstream bug."
--                  severity failure;
--              end if;
--        end if;

--    -- SELM thresholds sanity (only meaningful once package constants are set)
--    assert (SELM_NEG_MIN_S <= SELM_NEG_MAX_S) and
--           (SELM_ZERO_MIN_S <= SELM_ZERO_MAX_S) and
--           (SELM_POS_MIN_S <= SELM_POS_MAX_S)
--      report "msdf_mul_digit_select_r2: bad SELM thresholds (ordering)."
--      severity failure;

--    assert (SELM_NEG_MIN_S = V_HAT_MIN_S) and (SELM_POS_MAX_S = V_HAT_MAX_S)
--      report "msdf_mul_digit_select_r2: thresholds not clipped to representable v_hat range."
--      severity failure;

--    assert (SELM_NEG_MAX_S + 1 = SELM_ZERO_MIN_S) and
--           (SELM_ZERO_MAX_S + 1 = SELM_POS_MIN_S)
--      report "msdf_mul_digit_select_r2: SELM ranges are not contiguous."
--      severity failure;
--    -- synthesis translate_on

--    -- =========================
--    -- Functional selection
--    -- =========================
--    -- Only interpret numerically if clean. Otherwise p_digit stays SD_ZERO.
--    if clean then
--      vh_s := signed(v_hat);

--      if (vh_s >= to_signed(SELM_POS_MIN_S, TOP_V_BITS)) and
--         (vh_s <= to_signed(SELM_POS_MAX_S, TOP_V_BITS)) then
--        p_digit <= SD_POS1;

--      elsif (vh_s >= to_signed(SELM_ZERO_MIN_S, TOP_V_BITS)) and
--            (vh_s <= to_signed(SELM_ZERO_MAX_S, TOP_V_BITS)) then
--        p_digit <= SD_ZERO;

--      elsif (vh_s >= to_signed(SELM_NEG_MIN_S, TOP_V_BITS)) and
--            (vh_s <= to_signed(SELM_NEG_MAX_S, TOP_V_BITS)) then
--        p_digit <= SD_NEG1;

--      else
--        -- Unreachable if ranges cover full representable range
--        -- synthesis translate_off
--        assert false
--          report "msdf_mul_digit_select_r2: v_hat fell outside SELM ranges (should be impossible)."
--          severity failure;
--        -- synthesis translate_on
--        p_digit <= SD_ZERO;
--      end if;
--    end if;

--  end process;
--end architecture;
-- ============================================================================
--  File        : msdf_mul_digit_select_r2.vhd
--  Project     : Radix-2 MSDF Online Arithmetic (Multiplier Core)
--  Target      : Xilinx Zynq-7020 (PYNQ-Z2) / Vivado
--
--  Purpose
--  . Implements the radix-2 MSDF online multiplication digit-selection function
--    (SELM-style) for digit set {-1,0,+1}.
--
--  FPGA Optimization (SOP Array Area Reduction)
--  . The meta-clean gating (is_01_only) has been strictly moved to the 
--    simulation-only translate_off block. This prevents Vivado from synthesizing
--    redundant checking logic (LUT waste) when this module is replicated 
--    massively in MAC/SOP tiles.
--  . Time-0 meta warnings have been silenced to prevent console spam during 
--    large array initializations.
-- ============================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;

entity msdf_mul_digit_select_r2 is
  port (
    v_hat   : in  v_hat_t;      -- signed estimate (scaled by 2^t)
    p_digit : out sd_digit_t    -- {-1,0,+1}
  );
end entity;

architecture rtl of msdf_mul_digit_select_r2 is
begin
  comb : process(v_hat)
    variable vh_slv : std_logic_vector(v_hat'range);
    variable vh_s   : signed(v_hat'range);
    variable clean  : boolean;
  begin
    vh_slv := std_logic_vector(v_hat);
    vh_s   := signed(v_hat);

    -- Default deterministic output
    p_digit <= SD_ZERO;

    -- =========================
    -- Simulation-only checking
    -- =========================
    -- synthesis translate_off
    clean := is_01_only(vh_slv);
    
    if not clean then
      -- SILENCED: Time-0 / first-delta comb settle is ignored silently.
      -- We only trigger a failure if meta values persist into active simulation.
      if now > 0 ps then
        assert false
          report "msdf_mul_digit_select_r2: v_hat contains meta (U/X/Z) after t=0 -> REAL upstream bug."
          severity failure;
      end if;
    end if;

    -- SELM thresholds sanity
    assert (SELM_NEG_MIN_S <= SELM_NEG_MAX_S) and
           (SELM_ZERO_MIN_S <= SELM_ZERO_MAX_S) and
           (SELM_POS_MIN_S <= SELM_POS_MAX_S)
      report "msdf_mul_digit_select_r2: bad SELM thresholds (ordering)."
      severity failure;

    assert (SELM_NEG_MIN_S = V_HAT_MIN_S) and (SELM_POS_MAX_S = V_HAT_MAX_S)
      report "msdf_mul_digit_select_r2: thresholds not clipped to representable v_hat range."
      severity failure;

    assert (SELM_NEG_MAX_S + 1 = SELM_ZERO_MIN_S) and
           (SELM_ZERO_MAX_S + 1 = SELM_POS_MIN_S)
      report "msdf_mul_digit_select_r2: SELM ranges are not contiguous."
      severity failure;
    -- synthesis translate_on

    -- =========================
    -- Functional selection (Synthesized Hardware)
    -- =========================
    -- By removing the 'if clean then' wrapper here, Vivado directly connects 
    -- v_hat to the comparators. Zero wasted LUTs.
    
-- =========================
    -- Functional selection (Synthesized Hardware)
    -- =========================
    -- Vivado directly connects v_hat to the comparators. Zero wasted LUTs.
    
    if (vh_s >= to_signed(SELM_POS_MIN_S, TOP_V_BITS)) and
       (vh_s <= to_signed(SELM_POS_MAX_S, TOP_V_BITS)) then
      p_digit <= SD_POS1;

    elsif (vh_s >= to_signed(SELM_ZERO_MIN_S, TOP_V_BITS)) and
          (vh_s <= to_signed(SELM_ZERO_MAX_S, TOP_V_BITS)) then
      p_digit <= SD_ZERO;

    elsif (vh_s >= to_signed(SELM_NEG_MIN_S, TOP_V_BITS)) and
          (vh_s <= to_signed(SELM_NEG_MAX_S, TOP_V_BITS)) then
      p_digit <= SD_NEG1;

    else
      -- synthesis translate_off
      -- THE FIX: Only assert failure if the signal is actually a valid number.
      -- If it's 'U'/'X' at startup, it naturally falls here, so we ignore the assert.
      if clean then
        assert false
          report "msdf_mul_digit_select_r2: v_hat fell outside SELM ranges (impossible)."
          severity failure;
      end if;
      -- synthesis translate_on
      
      p_digit <= SD_ZERO;
    end if;

  end process;
end architecture;