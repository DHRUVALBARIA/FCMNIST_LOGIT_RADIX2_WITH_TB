-- ============================================================
-- File:    msdf_accel_axi_top.vhd
-- Purpose: Phase-8 top-level wrapper with AXI4-Lite slave
--
-- What this does:
-- . Presents a single AXI4-Lite slave interface to Vivado Block Design
-- . Instantiates:
--   . msdf_axi_lite_regs  (AXI-Lite register file + load windows)
--   . msdf_accel_top      (your proven Phase-5 compute top)
-- . Wires the reg-file "host side" signals into msdf_accel_top
--
-- Clock/reset assumptions (bring-up friendly):
-- . Use ONE clock domain for AXI and accelerator (s_axi_aclk drives clk)
-- . Use ONE reset domain (s_axi_aresetn drives rst_n)
-- . ce is tied high by default; you may later drive ce from a clock-enable
--   generator if you want throttling.
--
-- Real-life / thesis mapping:
-- . This is the “hardware peripheral shell” that lets a Zynq PS (ARM)
--   treat your accelerator as memory-mapped registers + RAM windows:
--   . PS writes X, W, B via AXI-Lite 
--   . PS pulses START
--   . PS polls DONE and reads LOGITS + CYCLES
-- . This is the exact integration step required for PYNQ overlays.
-- ============================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

entity msdf_accel_axi_top is
  generic (
    C_S_AXI_ADDR_WIDTH : natural  := 16;
    ACC_BITS_G         : positive := 40;
    BIAS_BITS_G        : positive := 16;
    FLUSH_EXTRA_G      : integer  := 2;
    SOFT_RESET_G       : boolean  := true
  );
  port (
    -- ==========================================================
    -- AXI4-Lite Slave Interface (to PS / AXI interconnect)
    -- ==========================================================
    s_axi_aclk    : in  std_logic;
    s_axi_aresetn : in  std_logic;

    s_axi_awaddr  : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
    s_axi_awvalid : in  std_logic;
    s_axi_awready : out std_logic;

    s_axi_wdata   : in  std_logic_vector(31 downto 0);
    s_axi_wstrb   : in  std_logic_vector(3 downto 0);
    s_axi_wvalid  : in  std_logic;
    s_axi_wready  : out std_logic;

    s_axi_bresp   : out std_logic_vector(1 downto 0);
    s_axi_bvalid  : out std_logic;
    s_axi_bready  : in  std_logic;

    s_axi_araddr  : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
    s_axi_arvalid : in  std_logic;
    s_axi_arready : out std_logic;

    s_axi_rdata   : out std_logic_vector(31 downto 0);
    s_axi_rresp   : out std_logic_vector(1 downto 0);
    s_axi_rvalid  : out std_logic;
    s_axi_rready  : in  std_logic
  );
end entity;

architecture rtl of msdf_accel_axi_top is 

  -- Internal “fabric” clock/reset/ce for msdf_accel_top
  signal clk   : std_logic;
  signal rst_n : std_logic;
  signal ce    : std_logic := '1';

  -- Reg-file -> accel_top host-side controls
  signal start_pulse_s      : std_logic;
  signal clear_done_pulse_s : std_logic;

  signal actv_wr_en_s   : std_logic;
  signal actv_wr_addr_s : addr_idx_t;
  signal actv_wr_data_s : std_logic_vector(31 downto 0);

  signal bias_wr_en_s   : std_logic;
  signal bias_wr_addr_s : neur_idx_t;
  signal bias_wr_data_s : std_logic_vector(31 downto 0);

  signal w_wr_en_s     : std_logic;
  signal w_wr_neuron_s : neur_idx_t;
  signal w_wr_tile_s   : tile_idx_t;
  signal w_wr_tile_i_s : w_fixed_vec_t;

  -- Accel_top -> reg-file status/readback
  signal busy_s        : std_logic;
  signal done_s        : std_logic;
  signal cycles_total_s: unsigned(31 downto 0);
  signal logits_pack_s : std_logic_vector(N_OUT*ACC_BITS_G-1 downto 0);

begin

  -- Single-clock integration (simplest for PYNQ)
  clk   <= s_axi_aclk;
  rst_n <= s_axi_aresetn;

  -- ============================================================
  -- AXI-Lite register file / load windows
  -- ============================================================
  u_regs : entity work.msdf_axi_lite_regs
    generic map (
      C_S_AXI_ADDR_WIDTH => C_S_AXI_ADDR_WIDTH,
      ACC_BITS_G         => ACC_BITS_G,
      BIAS_BITS_G        => BIAS_BITS_G
    )
    port map (
      s_axi_aclk    => s_axi_aclk,
      s_axi_aresetn => s_axi_aresetn,

      s_axi_awaddr  => s_axi_awaddr,
      s_axi_awvalid => s_axi_awvalid,
      s_axi_awready => s_axi_awready,

      s_axi_wdata   => s_axi_wdata,
      s_axi_wstrb   => s_axi_wstrb,
      s_axi_wvalid  => s_axi_wvalid,
      s_axi_wready  => s_axi_wready,

      s_axi_bresp   => s_axi_bresp,
      s_axi_bvalid  => s_axi_bvalid,
      s_axi_bready  => s_axi_bready,

      s_axi_araddr  => s_axi_araddr,
      s_axi_arvalid => s_axi_arvalid,
      s_axi_arready => s_axi_arready,

      s_axi_rdata   => s_axi_rdata,
      s_axi_rresp   => s_axi_rresp,
      s_axi_rvalid  => s_axi_rvalid,
      s_axi_rready  => s_axi_rready,

      start_pulse      => start_pulse_s,
      clear_done_pulse => clear_done_pulse_s,

      actv_wr_en   => actv_wr_en_s,
      actv_wr_addr => actv_wr_addr_s,
      actv_wr_data => actv_wr_data_s,

      bias_wr_en   => bias_wr_en_s,
      bias_wr_addr => bias_wr_addr_s,
      bias_wr_data => bias_wr_data_s,

      w_wr_en     => w_wr_en_s,
      w_wr_neuron => w_wr_neuron_s,
      w_wr_tile   => w_wr_tile_s,
      w_wr_tile_i => w_wr_tile_i_s,

      busy_in          => busy_s,
      done_in          => done_s,
      cycles_total_in  => cycles_total_s,
      logits_pack_in   => logits_pack_s
    );

  -- ============================================================
  -- Accelerator core top (your Phase-5 proven wrapper)
  -- ============================================================
  u_accel : entity work.msdf_accel_top
    generic map (
      ACC_BITS_G    => ACC_BITS_G,
      BIAS_BITS_G   => BIAS_BITS_G,
      FLUSH_EXTRA_G => FLUSH_EXTRA_G,
      SOFT_RESET_G  => SOFT_RESET_G
    )
    port map (
      clk   => clk,
      rst_n => rst_n,
      ce    => ce,

      start      => start_pulse_s,
      clear_done => clear_done_pulse_s,

      busy => busy_s,
      done => done_s,

      actv_wr_en   => actv_wr_en_s,
      actv_wr_addr => actv_wr_addr_s,
      actv_wr_data => actv_wr_data_s,

      bias_wr_en   => bias_wr_en_s,
      bias_wr_addr => bias_wr_addr_s,
      bias_wr_data => bias_wr_data_s,

      w_wr_en     => w_wr_en_s,
      w_wr_neuron => w_wr_neuron_s,
      w_wr_tile   => w_wr_tile_s,
      w_wr_tile_i => w_wr_tile_i_s,

      cycles_total => cycles_total_s,
      logits_pack  => logits_pack_s
    );

end architecture;
