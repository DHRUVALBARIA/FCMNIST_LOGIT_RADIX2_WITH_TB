-- ============================================================
-- File:    msdf_bias_mem.vhd
-- Purpose: Bias memory for msdf_fc_ctrl_10 (MNIST 784->10 demo)
--
-- Key properties
-- . 10 entries (N_OUT) of signed biases
-- . Write is synchronous (for TB init now, AXI-Lite later)
-- . Read is combinational by bias_sel (no added latency)
--
-- Why combinational read?
-- . msdf_fc_ctrl_10 expects bias_in to be "just there" in ST_ADD_BIAS.
-- . Adding a 1-cycle RAM latency would require modifying the controller
--   (a bias_valid or a wait state). This avoids that.
-- ============================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_accel_pkg.all;

entity msdf_bias_mem is
  generic (
    BIAS_BITS_G : positive := 16
  );
  port (
    clk   : in  std_logic;
    rst_n : in  std_logic;  -- active-low synchronous reset
    ce    : in  std_logic;

    -- Write interface (TB init now; AXI-Lite later)
    wr_en   : in  std_logic;
    wr_addr : in  neur_idx_t;                  -- 0..N_OUT-1
    wr_data : in  std_logic_vector(31 downto 0);

    -- Read interface (from msdf_fc_ctrl_10)
    bias_sel : in  neur_idx_t;                 -- neuron index
    bias_o   : out signed(BIAS_BITS_G-1 downto 0)
  );
end entity;

architecture rtl of msdf_bias_mem is

  type mem_t is array (0 to N_OUT-1) of signed(BIAS_BITS_G-1 downto 0);
  signal mem : mem_t := (others => (others => '0'));

  function idx_clamp(u : neur_idx_t) return natural is
    variable i : integer;
  begin
    i := to_integer(u);

    -- synthesis translate_off
    assert (i >= 0) and (i < N_OUT)
      report "msdf_bias_mem: index out of range: " & integer'image(i)
      severity warning;
    -- synthesis translate_on

    if (i < 0) or (i >= N_OUT) then
      return 0;
    else
      return natural(i);
    end if;
  end function;

begin

  -- Combinational read (no latency)
  bias_o <= mem(idx_clamp(bias_sel));

  -- Synchronous write (gated by ce to match your stall convention)
  process(clk)
    variable wa : natural;
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        mem <= (others => (others => '0'));
      elsif ce = '1' then
        if wr_en = '1' then
          wa := idx_clamp(wr_addr);
          mem(wa) <= signed(wr_data(BIAS_BITS_G-1 downto 0));
        end if;
      end if;
    end if;
  end process;

  -- synthesis translate_off
  assert N_OUT = 10
    report "msdf_bias_mem: note: designed for MNIST demo (N_OUT=10)"
    severity note;
  -- synthesis translate_on

end architecture;
