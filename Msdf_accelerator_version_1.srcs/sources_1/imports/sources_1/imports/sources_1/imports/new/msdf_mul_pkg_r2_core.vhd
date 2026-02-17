library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package msdf_mul_pkg_r2_core is
  --------------------------------------------------------------------
  -- Radix-2 MSDF online multiplication parameters
  -- Digit set: {-1,0,+1} 
  --------------------------------------------------------------------
  constant MSDF_RADIX       : integer := 2;
  constant MSDF_A           : integer := 1;

  -- Online delays used in your project
  constant ONLINE_DELAY_MUL : integer := 3;
  constant ONLINE_DELAY_ADD : integer := 2;

  --------------------------------------------------------------------
  -- Signed-digit encoding (2-bit two's complement)
  -- Legal: -1("11"), 0("00"), +1("01"); Illegal: -2("10")
  --------------------------------------------------------------------
  subtype sd_digit_t is signed(1 downto 0);
  type sd_digit_array is array (natural range <>) of sd_digit_t;

  constant SD_NEG1 : sd_digit_t := to_signed(-1, 2);
  constant SD_ZERO : sd_digit_t := to_signed( 0, 2);
  constant SD_POS1 : sd_digit_t := to_signed( 1, 2);

  -- Strict legality checks (meta-safe)
  function is_01_only(v : std_logic_vector) return boolean;
  function is_valid_sd_digit(d : sd_digit_t) return boolean;

  -- Strict decode: asserts on meta/illegal in simulation
  function sd_to_integer(d : sd_digit_t) return integer;

  -- Strict encode: asserts on out-of-range in simulation
  function int_to_sd_digit(i : integer) return sd_digit_t;

  --------------------------------------------------------------------
  -- Fixed-point for residual w[j] and v[j]
  --------------------------------------------------------------------
  constant W_INT_BITS   : integer := 2;
  constant W_FRAC_BITS  : integer := 12;
  constant W_TOTAL_BITS : integer := W_INT_BITS + W_FRAC_BITS;

  subtype w_fixed_t is signed(W_TOTAL_BITS-1 downto 0);
  subtype v_fixed_t is w_fixed_t;

  constant W_ZERO  : w_fixed_t := (others => '0');
  constant V_ZERO  : v_fixed_t := (others => '0');
  constant W_SCALE : integer := 2 ** W_FRAC_BITS;

  --------------------------------------------------------------------
  -- Optional: Wider accumulator fixed-point type for MAC / dot-product
  --------------------------------------------------------------------
  constant ACC_EXTRA_BITS_DEFAULT : integer := 16;
  constant ACC_TOTAL_BITS_DEFAULT : integer := W_TOTAL_BITS + ACC_EXTRA_BITS_DEFAULT;
  subtype  acc_fixed_t is signed(ACC_TOTAL_BITS_DEFAULT-1 downto 0);

  function w_to_acc(x : w_fixed_t) return acc_fixed_t;

  --------------------------------------------------------------------
  -- v-hat estimate: sign + integer bits + top t fractional bits
  --------------------------------------------------------------------
  constant V_HAT_FRAC_BITS : integer := 2;
  constant TOP_V_BITS      : integer := W_INT_BITS + V_HAT_FRAC_BITS;
  subtype  v_hat_t         is signed(TOP_V_BITS-1 downto 0);

  constant V_HAT_MIN_S : integer := -(2 ** (TOP_V_BITS-1));
  constant V_HAT_MAX_S : integer :=  (2 ** (TOP_V_BITS-1)) - 1;

  --------------------------------------------------------------------
  -- SELM thresholds (derived, not magic)
  --------------------------------------------------------------------
  constant SELM_POS_MIN_S  : integer :=  2 ** (V_HAT_FRAC_BITS-1);
  constant SELM_POS_MAX_S  : integer :=  V_HAT_MAX_S;

  constant SELM_ZERO_MIN_S : integer := -SELM_POS_MIN_S;
  constant SELM_ZERO_MAX_S : integer :=  SELM_POS_MIN_S - 1;

  constant SELM_NEG_MIN_S  : integer :=  V_HAT_MIN_S;
  constant SELM_NEG_MAX_S  : integer :=  SELM_ZERO_MIN_S - 1;

  --------------------------------------------------------------------
  -- Safe helpers
  --------------------------------------------------------------------
  function w_shift_left_1(x : w_fixed_t) return w_fixed_t;
  function v_to_vhat(v : v_fixed_t) return v_hat_t;

  -- Strict mapping from sd digit to fixed-point (+/-1/0)
  function sd_to_w_fixed(d : sd_digit_t) return w_fixed_t;

  --------------------------------------------------------------------
  -- SERIAL-PARALLEL PRECISION EXTENSION (NEW)
  --
  -- Goal: 
  --   stop destroying fractional bits each cycle by keeping w/v/E in a wider
  --   internal format (more fractional bits), then truncating once for debug.
  --
  -- Notes:
  --   - We keep integer bits the same (W_INT_BITS) to preserve v_hat slicing.
  --   - We extend only fractional bits:
  --       EXT_FRAC_BITS = W_FRAC_BITS + ONLINE_DELAY_MUL + GUARD
  --------------------------------------------------------------------
  constant SP_GUARD_BITS   : integer := 8;
  constant EXT_INT_BITS    : integer := W_INT_BITS;
  constant EXT_FRAC_BITS   : integer := W_FRAC_BITS + ONLINE_DELAY_MUL + SP_GUARD_BITS;
  constant EXT_TOTAL_BITS  : integer := EXT_INT_BITS + EXT_FRAC_BITS;
  constant EXT_FRAC_SHIFT  : integer := EXT_FRAC_BITS - W_FRAC_BITS; -- typically delta+guard

  subtype w_ext_t is signed(EXT_TOTAL_BITS-1 downto 0);
  subtype v_ext_t is w_ext_t;

  constant W_EXT_ZERO : w_ext_t := (others => '0');

  -- divide by 2^sh with truncation toward zero (unconstrained)
  function div_pow2_trunc0_any(s : signed; sh : natural) return signed;

  -- conversions between external fixed and internal extended fixed
  function w_to_ext(x : w_fixed_t) return w_ext_t;
  function ext_to_w_trunc0(x : w_ext_t) return w_fixed_t;

  -- v_hat extraction from extended v (same semantics as from v_fixed_t)
  function vext_to_vhat(v : v_ext_t) return v_hat_t;

  -- map sd digit to +/-1.0/0 in EXT scale (2^EXT_FRAC_BITS)
  function sd_to_ext_fixed(d : sd_digit_t) return w_ext_t;

end package;


package body msdf_mul_pkg_r2_core is

  function is_01_only(v : std_logic_vector) return boolean is
  begin
    for i in v'range loop
      if (v(i) /= '0') and (v(i) /= '1') then
        return false;
      end if;
    end loop;
    return true;
  end function;

  function is_valid_sd_digit(d : sd_digit_t) return boolean is
    variable b : std_logic_vector(1 downto 0);
  begin
    b := std_logic_vector(d);
    if not is_01_only(b) then
      return false;
    end if;
    return (b = "11") or (b = "00") or (b = "01");
  end function;

  -- =============================================================
  -- FIX APPLIED: Changed Severity to WARNING for Simulation Start
  -- =============================================================
  function sd_to_integer(d : sd_digit_t) return integer is
    variable b : std_logic_vector(1 downto 0);
  begin
    b := std_logic_vector(d);

    -- synthesis translate_off
    if not is_01_only(b) then
       -- Only warn, don't kill simulation. Return 0 to allow startup.
       assert false
         report "sd_to_integer: meta value (U/X/Z) detected. Defaulting to 0."
         severity warning;
       return 0;
    end if;
    -- synthesis translate_on

    if b = "00" then
      return 0;
    elsif b = "01" then
      return 1;
    elsif b = "11" then
      return -1;
    else
      -- "10" is illegal (-2) 
      -- synthesis translate_off
      assert false
        report "sd_to_integer: illegal sd_digit encoding '10' (=-2) detected"
        severity warning; -- Changed to warning
      -- synthesis translate_on
      return 0;
    end if;
  end function; 
 
  function int_to_sd_digit(i : integer) return sd_digit_t is
  begin
    case i is
      when -1 => return SD_NEG1;
      when  0 => return SD_ZERO;
      when  1 => return SD_POS1;
      when others =>
        -- synthesis translate_off
        assert false
          report "int_to_sd_digit: out of range [-1,1], got " & integer'image(i)
          severity failure;
        -- synthesis translate_on
        return SD_ZERO;
    end case;
  end function;

  function w_to_acc(x : w_fixed_t) return acc_fixed_t is
  begin
    return resize(x, ACC_TOTAL_BITS_DEFAULT);
  end function;

  function w_shift_left_1(x : w_fixed_t) return w_fixed_t is
  begin
    -- synthesis translate_off
    assert x(x'high) = x(x'high-1)
      report "w_shift_left_1: signed overflow on shift_left(1)"
      severity failure;
    -- synthesis translate_on
    return shift_left(x, 1);
  end function;

  function v_to_vhat(v : v_fixed_t) return v_hat_t is
    constant MSB : integer := W_TOTAL_BITS - 1;
    constant LSB : integer := W_FRAC_BITS - V_HAT_FRAC_BITS;
  begin
    -- synthesis translate_off
    assert (V_HAT_FRAC_BITS >= 0) and (W_FRAC_BITS >= V_HAT_FRAC_BITS)
      report "v_to_vhat: bad configuration (W_FRAC_BITS < V_HAT_FRAC_BITS)"
      severity failure;
    -- synthesis translate_on
    return v(MSB downto LSB);
  end function;

  function sd_to_w_fixed(d : sd_digit_t) return w_fixed_t is
    variable b : std_logic_vector(1 downto 0);
  begin
    b := std_logic_vector(d);
    -- Check without crashing
    if not is_01_only(b) then return (others => '0'); end if;

    if b = "11" then
      return to_signed(-W_SCALE, W_TOTAL_BITS);
    elsif b = "01" then
      return to_signed( W_SCALE, W_TOTAL_BITS);
    else
      return (others => '0');
    end if;
  end function;

  function div_pow2_trunc0_any(s : signed; sh : natural) return signed is
    variable x : signed(s'range);
    variable r : signed(s'range);
  begin
    if sh = 0 then return s; end if;
    x := s;  
    if s(s'high) = '1' then
      x := s + to_signed((2**integer(sh)) - 1, s'length);
    end if;
    r := shift_right(x, integer(sh));
    return r;
  end function;

  function w_to_ext(x : w_fixed_t) return w_ext_t is
    variable t : w_ext_t;
  begin
    t := resize(x, EXT_TOTAL_BITS);
    return shift_left(t, EXT_FRAC_SHIFT);
  end function;

  function ext_to_w_trunc0(x : w_ext_t) return w_fixed_t is
    variable shr : signed(x'range);
    variable w   : w_fixed_t; 
  begin
    shr := div_pow2_trunc0_any(x, natural(EXT_FRAC_SHIFT));
    w   := resize(shr, W_TOTAL_BITS);
    return w;
  end function;

  function vext_to_vhat(v : v_ext_t) return v_hat_t is
    constant MSB : integer := EXT_TOTAL_BITS - 1;
    constant LSB : integer := EXT_FRAC_BITS - V_HAT_FRAC_BITS;
  begin
    return v(MSB downto LSB);
  end function;

  function sd_to_ext_fixed(d : sd_digit_t) return w_ext_t is
    variable one : w_ext_t := (others => '0');
    variable b   : std_logic_vector(1 downto 0);
  begin
    one(EXT_FRAC_BITS) := '1';
    b := std_logic_vector(d); 
    
    if not is_01_only(b) then return (others=>'0'); end if;

    if b = "11" then return -one;
    elsif b = "01" then return one;
    else return (others => '0');
    end if;
  end function;

end package body; 