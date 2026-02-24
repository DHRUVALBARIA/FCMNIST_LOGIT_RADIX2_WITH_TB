-- ============================================================
-- File:    msdf_axi_lite_regs.vhd
-- Purpose: AXI4-Lite register + window bridge for msdf_accel_top
--
-- Synth robustness fix:
-- . Do NOT variable-slice the 400-bit logits_pack_in bus.
-- . Pre-slice into an array using constant generate slices, then mux by idx.
--
-- Behavior:
-- . Single-outstanding write (AW and W may arrive independently; latched until both seen)
-- . Single-outstanding read
-- . 32-bit AXI-Lite data bus
-- . Activation/bias windows are direct writes
-- . Weight uses lane staging regs + commit -> tile write into msdf_accel_top
-- . Logits readback: 2x32-bit words per ACC_BITS_G logit (default 40)
-- ============================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

entity msdf_axi_lite_regs is
  generic (
    C_S_AXI_ADDR_WIDTH : natural  := 16;
    ACC_BITS_G         : positive := 40;
    BIAS_BITS_G        : positive := 16
  );
  port (
    -- AXI4-Lite clock/reset
    s_axi_aclk    : in  std_logic;
    s_axi_aresetn : in  std_logic;

    -- AXI4-Lite write address channel
    s_axi_awaddr  : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
    s_axi_awvalid : in  std_logic;
    s_axi_awready : out std_logic;

    -- AXI4-Lite write data channel
    s_axi_wdata   : in  std_logic_vector(31 downto 0);
    s_axi_wstrb   : in  std_logic_vector(3 downto 0);
    s_axi_wvalid  : in  std_logic;
    s_axi_wready  : out std_logic;

    -- AXI4-Lite write response channel
    s_axi_bresp   : out std_logic_vector(1 downto 0);
    s_axi_bvalid  : out std_logic;
    s_axi_bready  : in  std_logic;

    -- AXI4-Lite read address channel
    s_axi_araddr  : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
    s_axi_arvalid : in  std_logic;
    s_axi_arready : out std_logic;

    -- AXI4-Lite read data channel
    s_axi_rdata   : out std_logic_vector(31 downto 0);
    s_axi_rresp   : out std_logic_vector(1 downto 0);
    s_axi_rvalid  : out std_logic;
    s_axi_rready  : in  std_logic;

    -- Connections to msdf_accel_top "host side"
    start_pulse      : out std_logic;
    clear_done_pulse : out std_logic;

    actv_wr_en   : out std_logic;
    actv_wr_addr : out addr_idx_t;
    actv_wr_data : out std_logic_vector(31 downto 0);

    bias_wr_en   : out std_logic;
    bias_wr_addr : out neur_idx_t;
    bias_wr_data : out std_logic_vector(31 downto 0);

    w_wr_en     : out std_logic;
    w_wr_neuron : out neur_idx_t;
    w_wr_tile   : out tile_idx_t;
    w_wr_tile_i : out w_fixed_vec_t;

    -- Readback inputs from msdf_accel_top
    busy_in         : in  std_logic;
    done_in         : in  std_logic;
    cycles_total_in : in  unsigned(31 downto 0);
    logits_pack_in  : in  std_logic_vector(N_OUT*ACC_BITS_G-1 downto 0)
  );
end entity;

architecture rtl of msdf_axi_lite_regs is

  subtype acc_t is signed(ACC_BITS_G-1 downto 0);
  type acc_vec_t is array (0 to N_OUT-1) of acc_t;

  constant RESP_OKAY_C : std_logic_vector(1 downto 0) := "00";

  -- Address map
  constant CTRL_ADDR_C     : unsigned(C_S_AXI_ADDR_WIDTH-1 downto 0) := to_unsigned(16#0000#, C_S_AXI_ADDR_WIDTH);
  constant STATUS_ADDR_C   : unsigned(C_S_AXI_ADDR_WIDTH-1 downto 0) := to_unsigned(16#0004#, C_S_AXI_ADDR_WIDTH);
  constant CYCLES_ADDR_C   : unsigned(C_S_AXI_ADDR_WIDTH-1 downto 0) := to_unsigned(16#0008#, C_S_AXI_ADDR_WIDTH);
  constant CONFIG0_ADDR_C  : unsigned(C_S_AXI_ADDR_WIDTH-1 downto 0) := to_unsigned(16#000C#, C_S_AXI_ADDR_WIDTH);
  constant CONFIG1_ADDR_C  : unsigned(C_S_AXI_ADDR_WIDTH-1 downto 0) := to_unsigned(16#0010#, C_S_AXI_ADDR_WIDTH);

  constant LOGITS_BASE_C   : unsigned(C_S_AXI_ADDR_WIDTH-1 downto 0) := to_unsigned(16#0100#, C_S_AXI_ADDR_WIDTH);

  constant ACTV_BASE_C     : unsigned(C_S_AXI_ADDR_WIDTH-1 downto 0) := to_unsigned(16#1000#, C_S_AXI_ADDR_WIDTH);
  constant BIAS_BASE_C     : unsigned(C_S_AXI_ADDR_WIDTH-1 downto 0) := to_unsigned(16#2000#, C_S_AXI_ADDR_WIDTH);

  constant W_NEUR_ADDR_C   : unsigned(C_S_AXI_ADDR_WIDTH-1 downto 0) := to_unsigned(16#3000#, C_S_AXI_ADDR_WIDTH);
  constant W_TILE_ADDR_C   : unsigned(C_S_AXI_ADDR_WIDTH-1 downto 0) := to_unsigned(16#3004#, C_S_AXI_ADDR_WIDTH);
  constant W_LANE_BASE_C   : unsigned(C_S_AXI_ADDR_WIDTH-1 downto 0) := to_unsigned(16#3010#, C_S_AXI_ADDR_WIDTH);
  constant W_COMMIT_ADDR_C : unsigned(C_S_AXI_ADDR_WIDTH-1 downto 0) := to_unsigned(16#3030#, C_S_AXI_ADDR_WIDTH);

  -- Internal ready signals (never read OUT ports)
  signal awready_s : std_logic;
  signal wready_s  : std_logic;
  signal arready_s : std_logic;

  -- Write latching
  signal aw_hold  : std_logic := '0';
  signal w_hold   : std_logic := '0';
  signal awaddr_r : unsigned(C_S_AXI_ADDR_WIDTH-1 downto 0) := (others => '0');
  signal wdata_r  : std_logic_vector(31 downto 0) := (others => '0');
  signal wstrb_r  : std_logic_vector(3 downto 0)  := (others => '0');
  signal bvalid_r : std_logic := '0';

  -- Read channel
  signal rvalid_r : std_logic := '0';
  signal rdata_r  : std_logic_vector(31 downto 0) := (others => '0');

  -- Staging regs to msdf_accel_top
  signal start_pulse_r      : std_logic := '0';
  signal clear_done_pulse_r : std_logic := '0';

  signal actv_wr_en_r   : std_logic := '0';
  signal actv_wr_addr_r : addr_idx_t := (others => '0');
  signal actv_wr_data_r : std_logic_vector(31 downto 0) := (others => '0');

  signal bias_wr_en_r   : std_logic := '0';
  signal bias_wr_addr_r : neur_idx_t := (others => '0');
  signal bias_wr_data_r : std_logic_vector(31 downto 0) := (others => '0');

  signal w_wr_en_r      : std_logic := '0';
  signal w_wr_neuron_r  : neur_idx_t := (others => '0');
  signal w_wr_tile_r    : tile_idx_t := (others => '0');
  signal w_wr_tile_i_r  : w_fixed_vec_t := (others => (others => '0'));

  type s16_vec_t is array (0 to TILE_LEN-1) of signed(15 downto 0);
  signal w_lane_r : s16_vec_t := (others => (others => '0'));

  signal w_sel_neur_r : neur_idx_t := (others => '0');
  signal w_sel_tile_r : tile_idx_t := (others => '0');

  -- Pre-sliced logits array (constant slices -> clean synthesis)
  signal logits_arr_s : acc_vec_t := (others => (others => '0'));

  function uaddr_word(a : unsigned) return unsigned is
    variable x : unsigned(a'range);
  begin
    x := a;
    x(1 downto 0) := (others => '0');
    return x;
  end function;

begin

  -- Pre-slice packed logits into array using constant generate indices
  gen_logits : for j in 0 to N_OUT-1 generate
  begin
    logits_arr_s(j) <= signed(logits_pack_in((j+1)*ACC_BITS_G-1 downto j*ACC_BITS_G));
  end generate;

  -- Fixed responses
  s_axi_bresp <= RESP_OKAY_C;
  s_axi_rresp <= RESP_OKAY_C;

  -- Outputs
  s_axi_bvalid <= bvalid_r;
  s_axi_rvalid <= rvalid_r;
  s_axi_rdata  <= rdata_r;

  start_pulse      <= start_pulse_r;
  clear_done_pulse <= clear_done_pulse_r;

  actv_wr_en   <= actv_wr_en_r;
  actv_wr_addr <= actv_wr_addr_r;
  actv_wr_data <= actv_wr_data_r;

  bias_wr_en   <= bias_wr_en_r;
  bias_wr_addr <= bias_wr_addr_r;
  bias_wr_data <= bias_wr_data_r;

  w_wr_en     <= w_wr_en_r;
  w_wr_neuron <= w_wr_neuron_r;
  w_wr_tile   <= w_wr_tile_r;
  w_wr_tile_i <= w_wr_tile_i_r;

  -- Ready logic (single outstanding)
  awready_s <= '1' when (aw_hold = '0') and (bvalid_r = '0') else '0';
  wready_s  <= '1' when (w_hold  = '0') and (bvalid_r = '0') else '0';
  arready_s <= '1' when (rvalid_r = '0') else '0';

  s_axi_awready <= awready_s;
  s_axi_wready  <= wready_s;
  s_axi_arready <= arready_s;

  process(s_axi_aclk)
    variable addr_w   : unsigned(C_S_AXI_ADDR_WIDTH-1 downto 0);
    variable offs     : integer;
    variable idx      : integer;
    variable lane     : integer;

    variable cfg0     : std_logic_vector(31 downto 0);
    variable cfg1     : std_logic_vector(31 downto 0);

    variable w_ok16   : boolean;

    variable logit_s  : acc_t;
    variable lo32     : std_logic_vector(31 downto 0);
    variable hi32     : std_logic_vector(31 downto 0);
  begin
    if rising_edge(s_axi_aclk) then
      if s_axi_aresetn = '0' then
        aw_hold  <= '0';
        w_hold   <= '0';
        awaddr_r <= (others => '0');
        wdata_r  <= (others => '0');
        wstrb_r  <= (others => '0');
        bvalid_r <= '0';

        rvalid_r <= '0';
        rdata_r  <= (others => '0');

        start_pulse_r      <= '0';
        clear_done_pulse_r <= '0';

        actv_wr_en_r   <= '0';
        actv_wr_addr_r <= (others => '0');
        actv_wr_data_r <= (others => '0');

        bias_wr_en_r   <= '0';
        bias_wr_addr_r <= (others => '0');
        bias_wr_data_r <= (others => '0');

        w_wr_en_r      <= '0';
        w_wr_neuron_r  <= (others => '0');
        w_wr_tile_r    <= (others => '0');
        w_wr_tile_i_r  <= (others => (others => '0'));

        w_lane_r       <= (others => (others => '0'));
        w_sel_neur_r   <= (others => '0');
        w_sel_tile_r   <= (others => '0');

      else
        -- Default: deassert one-cycle pulses
        start_pulse_r      <= '0';
        clear_done_pulse_r <= '0';
        actv_wr_en_r       <= '0';
        bias_wr_en_r       <= '0';
        w_wr_en_r          <= '0';

        -- Capture AW
        if (s_axi_awvalid = '1') and (awready_s = '1') then
          awaddr_r <= unsigned(s_axi_awaddr);
          aw_hold  <= '1';
        end if;

        -- Capture W
        if (s_axi_wvalid = '1') and (wready_s = '1') then
          wdata_r <= s_axi_wdata;
          wstrb_r <= s_axi_wstrb;
          w_hold  <= '1';
        end if;

        -- Execute write when both captured
        if (aw_hold = '1') and (w_hold = '1') and (bvalid_r = '0') then
          addr_w := uaddr_word(awaddr_r);
          w_ok16 := (wstrb_r(1 downto 0) = "11");

          if addr_w = CTRL_ADDR_C then
            if wdata_r(0) = '1' then
              start_pulse_r <= '1';
            end if;
            if wdata_r(1) = '1' then
              clear_done_pulse_r <= '1';
            end if;

          elsif w_ok16 and (addr_w >= ACTV_BASE_C) and (addr_w < (ACTV_BASE_C + to_unsigned(VEC_LEN*4, C_S_AXI_ADDR_WIDTH))) then
            offs := to_integer(addr_w - ACTV_BASE_C);
            idx  := offs / 4;
            if (idx >= 0) and (idx < VEC_LEN) then
              actv_wr_addr_r <= to_unsigned(idx, ADDR_BITS_C);
              actv_wr_data_r <= wdata_r;
              actv_wr_en_r   <= '1';
            end if;

          elsif w_ok16 and (addr_w >= BIAS_BASE_C) and (addr_w < (BIAS_BASE_C + to_unsigned(N_OUT*4, C_S_AXI_ADDR_WIDTH))) then
            offs := to_integer(addr_w - BIAS_BASE_C);
            idx  := offs / 4;
            if (idx >= 0) and (idx < N_OUT) then
              bias_wr_addr_r <= to_unsigned(idx, NEUR_BITS_C);
              bias_wr_data_r <= wdata_r;
              bias_wr_en_r   <= '1';
            end if;

          elsif w_ok16 and (addr_w = W_NEUR_ADDR_C) then
            w_sel_neur_r <= unsigned(wdata_r(NEUR_BITS_C-1 downto 0));

          elsif w_ok16 and (addr_w = W_TILE_ADDR_C) then
            w_sel_tile_r <= unsigned(wdata_r(TILE_BITS_C-1 downto 0));

          elsif w_ok16 and (addr_w >= W_LANE_BASE_C) and (addr_w < (W_LANE_BASE_C + to_unsigned(TILE_LEN*4, C_S_AXI_ADDR_WIDTH))) then
            offs := to_integer(addr_w - W_LANE_BASE_C);
            lane := offs / 4;
            if (lane >= 0) and (lane < TILE_LEN) then
              w_lane_r(lane) <= signed(wdata_r(15 downto 0));
            end if;

          elsif addr_w = W_COMMIT_ADDR_C then
            w_wr_neuron_r <= w_sel_neur_r;
            w_wr_tile_r   <= w_sel_tile_r;

            for l in 0 to TILE_LEN-1 loop
              w_wr_tile_i_r(l) <= resize(w_lane_r(l), W_TOTAL_BITS);
            end loop;

            w_wr_en_r <= '1';
          end if;

          bvalid_r <= '1';
          aw_hold  <= '0';
          w_hold   <= '0';
        end if;

        -- Write response handshake
        if (bvalid_r = '1') and (s_axi_bready = '1') then
          bvalid_r <= '0';
        end if;

        -- Read address accept
        if (s_axi_arvalid = '1') and (arready_s = '1') then
          addr_w  := uaddr_word(unsigned(s_axi_araddr));
          rdata_r <= (others => '0');

          cfg0 := (others => '0');
          cfg0(7 downto 0)   := std_logic_vector(to_unsigned(TILE_LEN, 8));
          cfg0(15 downto 8)  := std_logic_vector(to_unsigned(N_DIGITS, 8));
          cfg0(23 downto 16) := std_logic_vector(to_unsigned(FLUSH_EXTRA_G, 8));

          cfg1 := (others => '0');
          cfg1(15 downto 0)  := std_logic_vector(to_unsigned(VEC_LEN, 16));
          cfg1(23 downto 16) := std_logic_vector(to_unsigned(N_OUT, 8));
          cfg1(31 downto 24) := std_logic_vector(to_unsigned(N_TILES, 8));

          if addr_w = STATUS_ADDR_C then
            rdata_r(0) <= busy_in;
            rdata_r(1) <= done_in;

          elsif addr_w = CYCLES_ADDR_C then
            rdata_r <= std_logic_vector(cycles_total_in);

          elsif addr_w = CONFIG0_ADDR_C then
            rdata_r <= cfg0;

          elsif addr_w = CONFIG1_ADDR_C then
            rdata_r <= cfg1;

          -- LOGITS window: 2 words per logit, stride 8 bytes
          elsif (addr_w >= LOGITS_BASE_C) and (addr_w < (LOGITS_BASE_C + to_unsigned(N_OUT*8, C_S_AXI_ADDR_WIDTH))) then
            offs := to_integer(addr_w - LOGITS_BASE_C);
            idx  := offs / 8;

            if (idx >= 0) and (idx < N_OUT) then
              logit_s := logits_arr_s(idx);

              lo32 := std_logic_vector(logit_s(31 downto 0));

              hi32 := (others => logit_s(ACC_BITS_G-1));
              if ACC_BITS_G > 32 then
                hi32(ACC_BITS_G-33 downto 0) := std_logic_vector(logit_s(ACC_BITS_G-1 downto 32));
              end if;

              if (offs mod 8) = 0 then
                rdata_r <= lo32;
              else
                rdata_r <= hi32;
              end if;
            end if;

          else
            rdata_r <= (others => '0');
          end if;

          rvalid_r <= '1';
        end if;

        -- Read data handshake
        if (rvalid_r = '1') and (s_axi_rready = '1') then
          rvalid_r <= '0';
        end if;

      end if;
    end if;
  end process;

  -- synthesis translate_off
  assert (VEC_LEN mod TILE_LEN) = 0
    report "msdf_axi_lite_regs: VEC_LEN must be divisible by TILE_LEN"
    severity failure;
  -- synthesis translate_on

end architecture;
