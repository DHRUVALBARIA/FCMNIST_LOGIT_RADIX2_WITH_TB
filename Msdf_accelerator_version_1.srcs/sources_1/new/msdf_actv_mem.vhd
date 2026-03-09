-- ============================================================
-- File:    msdf_actv_mem.vhd
-- Purpose: Activation memory wrapper for FC controller
--
-- Behavior:
--   . Write port: stores low 16 bits of wr_data into RAM at wr_addr
--                (writes are NOT gated by ce; PS should be able to write anytime)
--   . Read port (FC): x_rd_en + x_rd_addr -> 1-cycle later x_rd_valid pulses
--                    and x_o holds the read value (resized to w_fixed_t)
--
-- Important:
--   . x_rd_valid is generated only when ce='1', matching msdf_fc_ctrl_10 sampling.
-- ============================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

entity msdf_actv_mem is
  port (
    clk   : in  std_logic;
    rst_n : in  std_logic;  -- active-low synchronous reset
    ce    : in  std_logic;

    -- Write interface (AXI-Lite side later)
    wr_en   : in  std_logic;
    wr_addr : in  addr_idx_t;                 -- 0..783
    wr_data : in  std_logic_vector(31 downto 0);

    -- Read interface (from msdf_fc_ctrl_10)
    x_rd_en    : in  std_logic;
    x_rd_addr  : in  addr_idx_t;              -- 0..783
    x_rd_valid : out std_logic;
    x_o        : out w_fixed_t
  );
end entity;

architecture rtl of msdf_actv_mem is

  signal rst_sync : std_logic;

  -- RAM read data (stored as 16-bit signed)
  signal rdata16 : std_logic_vector(15 downto 0) := (others => '0');

  -- Valid pipeline (1-cycle)
  signal vld_r : std_logic := '0';

  -- Handshake gating
  signal rd_fire  : std_logic := '0';
  signal ren_s    : std_logic := '0';
  signal we_s     : std_logic := '0';
  signal wdata16  : std_logic_vector(15 downto 0) := (others => '0');

begin

  -- If your dp-ram expects active-high reset named "rst", keep this mapping
  rst_sync <= not rst_n;

  -- Prepare write signals (do NOT gate with ce)
  we_s    <= wr_en;
  wdata16 <= wr_data(15 downto 0);

  -- Gate FC reads with ce so the controller will see the valid pulse
  rd_fire <= x_rd_en and ce;
  ren_s   <= rd_fire;

  -- Output data conversion: 16-bit stored -> resize to w_fixed_t (14-bit)
  -- This assumes your SW writes sign-extended values (recommended).
  x_o <= resize(signed(rdata16), W_TOTAL_BITS);

  -- Valid pulse aligned to synchronous read latency
  x_rd_valid <= vld_r;

  u_ram : entity work.msdf_dp_ram_s16
    generic map (
      DEPTH_G     => VEC_LEN,     -- 784
      ADDR_BITS_G => ADDR_BITS_C, -- 10
      RAM_STYLE_G => "block"
    )
    port map (
      clk   => clk,
      rst   => rst_sync,

      -- Port A write
      we    => we_s,
      waddr => wr_addr,
      wdata => wdata16,

      -- Port B read
      ren   => ren_s,
      raddr => x_rd_addr,
      rdata => rdata16
    );

  process(clk)
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        vld_r <= '0';
      else
        -- Only generate/advance the valid pulse when ce='1'
        if ce = '1' then
          vld_r <= rd_fire;     -- 1-cycle delayed pulse
        else
          vld_r <= '0';
        end if;
      end if;
    end if;
  end process;

  -- synthesis translate_off
  assert (VEC_LEN = 784)
    report "msdf_actv_mem: expected VEC_LEN=784 for MNIST FC demo"
    severity note;
  -- synthesis translate_on

end architecture;
