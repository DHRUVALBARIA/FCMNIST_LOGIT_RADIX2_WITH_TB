-- ============================================================
-- File:    tb_msdf_accel_axi_top_axil_txt.vhd
-- Purpose: AXI4-Lite simulation testbench for msdf_accel_axi_top
--
-- What it verifies (practically useful before Vivado BD):
-- . AXI-Lite write path (AW/W/B) into:
--   . activation window
--   . bias window
--   . weight staging regs + commit
--   . CTRL (clear_done, start)
-- . AXI-Lite read path (AR/R) from:
--   . STATUS (busy/done)
--   . CYCLES
--   . LOGITS window (2x32-bit words per 40-bit logit)
-- . End-to-end: loads one or more inference cases from txt and checks logits.
--
-- Vector file format (same as your pre-AXI TB):
-- . w_fixed.txt      : tile-major (j,t,lane) => 10 * N_TILES * TILE_LEN lines
-- . b_fixed.txt      : 10 lines
-- . x_fixed.txt      : n_cases * 784 lines (case-major)
-- . logits_exp.txt   : n_cases * 10 lines (case-major)
--
-- Address map assumed (matches your msdf_axi_lite_regs "new" file):
-- . 0x0000 CTRL      : bit0 start (W1P), bit1 clear_done (W1P)
-- . 0x0004 STATUS    : bit0 busy, bit1 done
-- . 0x0008 CYCLES    : 32-bit cycles_total
-- . 0x0100 LOGITS    : 2 words per logit, stride 8 bytes (lo @ +0, hi @ +4)
-- . 0x1000 ACTV      : idx = (addr-0x1000)/4, 0..783
-- . 0x2000 BIAS      : idx = (addr-0x2000)/4, 0..9
-- . 0x3000 W_NEUR    : select neuron for weight tile commit
-- . 0x3004 W_TILE    : select tile for weight tile commit
-- . 0x3010 W_LANE[k] : lane k at 0x3010 + 4*k
-- . 0x3030 W_COMMIT  : commit staged lanes as one tile write
-- ============================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use std.textio.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

entity tb_msdf_accel_axi_top_axil_txt is
  generic (
    -- Use absolute path for robustness in XSim
    G_VEC_DIR        : string  := "/home/bad32576/Serial_Parallel_Mul/Msdf_accelerator_version_1";

    -- Number of inference cases to run from x_fixed/logits_exp
    G_N_CASES        : natural := 1000;

    -- Allowed absolute error on logits (your bound-based verification style)
    G_BOUND_FINAL    : integer := 10;

    -- Optional cycles check
    G_CHECK_CYCLES   : boolean := true;
    G_EXPECT_CYCLES  : natural := 61781;

    -- Progress print
    G_PROGRESS_EVERY : natural := 50;

    -- Clock period
    G_CLK_PERIOD     : time := 10 ns
  );
end entity;

architecture tb of tb_msdf_accel_axi_top_axil_txt is

  constant C_ADDR_W_C : natural := 16;
  constant ACC_BITS_C : positive := 40;
  constant BIAS_BITS_C: positive := 16;

  constant FLUSH_EXTRA_C : integer := 2;
  constant SOFT_RESET_C  : boolean := true;

  -- AXI clock/reset
  signal s_axi_aclk    : std_logic := '0';
  signal s_axi_aresetn : std_logic := '0';

  -- AXI write address
  signal s_axi_awaddr  : std_logic_vector(C_ADDR_W_C-1 downto 0) := (others => '0');
  signal s_axi_awvalid : std_logic := '0';
  signal s_axi_awready : std_logic;

  -- AXI write data
  signal s_axi_wdata   : std_logic_vector(31 downto 0) := (others => '0');
  signal s_axi_wstrb   : std_logic_vector(3 downto 0)  := (others => '0');
  signal s_axi_wvalid  : std_logic := '0';
  signal s_axi_wready  : std_logic;

  -- AXI write response
  signal s_axi_bresp   : std_logic_vector(1 downto 0);
  signal s_axi_bvalid  : std_logic;
  signal s_axi_bready  : std_logic := '1';

  -- AXI read address
  signal s_axi_araddr  : std_logic_vector(C_ADDR_W_C-1 downto 0) := (others => '0');
  signal s_axi_arvalid : std_logic := '0';
  signal s_axi_arready : std_logic;

  -- AXI read data
  signal s_axi_rdata   : std_logic_vector(31 downto 0);
  signal s_axi_rresp   : std_logic_vector(1 downto 0);
  signal s_axi_rvalid  : std_logic;
  signal s_axi_rready  : std_logic := '1';

  -- Address constants (byte addresses)
  constant CTRL_ADDR_C     : natural := 16#0000#;
  constant STATUS_ADDR_C   : natural := 16#0004#;
  constant CYCLES_ADDR_C   : natural := 16#0008#;

  constant LOGITS_BASE_C   : natural := 16#0100#;

  constant ACTV_BASE_C     : natural := 16#1000#;
  constant BIAS_BASE_C     : natural := 16#2000#;

  constant W_NEUR_ADDR_C   : natural := 16#3000#;
  constant W_TILE_ADDR_C   : natural := 16#3004#;
  constant W_LANE_BASE_C   : natural := 16#3010#;
  constant W_COMMIT_ADDR_C : natural := 16#3030#;

  subtype acc_t is signed(ACC_BITS_C-1 downto 0);
  type acc_vec_t is array (0 to N_OUT-1) of acc_t;

  -- Helpers
  function slv32_from_s16(i : integer) return std_logic_vector is
    variable s16 : signed(15 downto 0);
    variable s32 : signed(31 downto 0);
  begin
    s16 := to_signed(i, 16);
    s32 := resize(s16, 32);
    return std_logic_vector(s32);
  end function;

  function slv32_from_u(i : natural) return std_logic_vector is
    variable u32 : unsigned(31 downto 0);
  begin
    u32 := to_unsigned(i, 32);
    return std_logic_vector(u32);
  end function;

  function rebuild_logit(lo32, hi32 : std_logic_vector(31 downto 0)) return acc_t is
    variable s : acc_t := (others => '0');
    variable hi_bits : integer;
  begin
    -- Low 32 bits
    s(31 downto 0) := signed(lo32);

    -- Upper bits are packed into HI word LSBs (sign-extended above)
    if ACC_BITS_C > 32 then
      hi_bits := ACC_BITS_C - 32; -- for 40-bit, hi_bits=8
      s(ACC_BITS_C-1 downto 32) := signed(hi32(hi_bits-1 downto 0));
    end if;

    return s;
  end function;

begin

  -- Clock
  s_axi_aclk <= not s_axi_aclk after G_CLK_PERIOD/2;

  -- DUT
  dut : entity work.msdf_accel_axi_top
    generic map (
      C_S_AXI_ADDR_WIDTH => C_ADDR_W_C,
      ACC_BITS_G         => ACC_BITS_C,
      BIAS_BITS_G        => BIAS_BITS_C,
      FLUSH_EXTRA_G      => FLUSH_EXTRA_C,
      SOFT_RESET_G       => SOFT_RESET_C
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
      s_axi_rready  => s_axi_rready
    );

  -- ============================================================
  -- Stimulus
  -- ============================================================
  stim : process
    file f_w  : text;
    file f_b  : text;
    file f_x  : text;
    file f_le : text;

    variable st : file_open_status;
    variable L  : line;

    variable w_i  : integer;
    variable b_i  : integer;
    variable x_i  : integer;
    variable le_i : integer;

    variable exp_logits : acc_vec_t;
    variable got_logits : acc_vec_t;

    variable status_w : std_logic_vector(31 downto 0);
    variable cycles_w : std_logic_vector(31 downto 0);
    variable lo_w     : std_logic_vector(31 downto 0);
    variable hi_w     : std_logic_vector(31 downto 0);

    variable err   : integer;
    variable abs_e : integer;

    variable k    : natural;
    variable j    : natural;
    variable t    : natural;
    variable lane : natural;
    variable idx  : natural;

    constant W_FILE  : string := G_VEC_DIR & "/w_fixed.txt";
    constant B_FILE  : string := G_VEC_DIR & "/b_fixed.txt";
    constant X_FILE  : string := G_VEC_DIR & "/x_fixed.txt";
    constant LE_FILE : string := G_VEC_DIR & "/logits_exp.txt";

    constant WATCHDOG_CYC : natural := 400000;

    procedure tick is
    begin
      wait until rising_edge(s_axi_aclk);
    end procedure;

    procedure axi_write(addr : natural; data : std_logic_vector(31 downto 0)) is
      variable aw_done : boolean := false;
      variable w_done  : boolean := false;
      variable bd_done : boolean := false;
    begin
      -- Drive write address + data (can be accepted same or different cycles)
      s_axi_awaddr  <= std_logic_vector(to_unsigned(addr, C_ADDR_W_C));
      s_axi_awvalid <= '1';

      s_axi_wdata   <= data;
      s_axi_wstrb   <= "1111";
      s_axi_wvalid  <= '1';

      -- Handshake AW and W (independently)
      while (not aw_done) or (not w_done) loop
        tick;

        if (not aw_done) and (s_axi_awvalid = '1') and (s_axi_awready = '1') then
          s_axi_awvalid <= '0';
          aw_done := true;
        end if;

        if (not w_done) and (s_axi_wvalid = '1') and (s_axi_wready = '1') then
          s_axi_wvalid <= '0';
          w_done := true;
        end if;
      end loop;

      -- Wait for write response
      while not bd_done loop
        tick;
        if s_axi_bvalid = '1' then
          bd_done := true; -- bready is held at '1', so it will complete
        end if;
      end loop;

      -- One extra cycle to let bvalid clear in the DUT
      tick;
    end procedure;

    procedure axi_read(addr : natural; variable data_out : out std_logic_vector(31 downto 0)) is
      variable ar_done : boolean := false;
      variable rd_done : boolean := false;
    begin
      s_axi_araddr  <= std_logic_vector(to_unsigned(addr, C_ADDR_W_C));
      s_axi_arvalid <= '1';

      -- AR handshake
      while not ar_done loop
        tick;
        if (s_axi_arvalid = '1') and (s_axi_arready = '1') then
          s_axi_arvalid <= '0';
          ar_done := true;
        end if;
      end loop;

      -- Wait for RVALID then capture
      while not rd_done loop
        tick;
        if s_axi_rvalid = '1' then
          data_out := s_axi_rdata; -- rready held at '1'
          rd_done := true;
        end if;
      end loop;

      -- One extra cycle to let rvalid clear in the DUT
      tick;
    end procedure;

    procedure poll_done is
      variable wd : natural := 0;
      variable done_bit : std_logic;
    begin
      loop
        axi_read(STATUS_ADDR_C, status_w);
        done_bit := status_w(1);
        exit when done_bit = '1';

        wd := wd + 1;
        if wd = WATCHDOG_CYC then
          assert false report "TB FAIL: timeout waiting for DONE via AXI status" severity failure;
        end if;
      end loop;
    end procedure;

  begin
    -- Reset
    s_axi_aresetn <= '0';
    s_axi_awvalid <= '0';
    s_axi_wvalid  <= '0';
    s_axi_arvalid <= '0';
    s_axi_wstrb   <= (others => '0');

    for i in 0 to 10 loop
      tick;
    end loop;

    s_axi_aresetn <= '1';
    for i in 0 to 5 loop
      tick;
    end loop;

    report "TB: G_VEC_DIR = " & G_VEC_DIR severity note;
    report "TB: opening  " & W_FILE severity note;
    report "TB: opening  " & B_FILE severity note;
    report "TB: opening  " & X_FILE severity note;
    report "TB: opening  " & LE_FILE severity note;

    file_open(st, f_w,  W_FILE,  read_mode);
    assert st = open_ok report "TB FAIL: cannot open " & W_FILE severity failure;

    file_open(st, f_b,  B_FILE,  read_mode);
    assert st = open_ok report "TB FAIL: cannot open " & B_FILE severity failure;

    file_open(st, f_x,  X_FILE,  read_mode);
    assert st = open_ok report "TB FAIL: cannot open " & X_FILE severity failure;

    file_open(st, f_le, LE_FILE, read_mode);
    assert st = open_ok report "TB FAIL: cannot open " & LE_FILE severity failure;

    -- ==========================================================
    -- Load weights once (AXI path): (j,t,lane) with staging+commit
    -- ==========================================================
    for j in 0 to N_OUT-1 loop
      for t in 0 to N_TILES-1 loop

        axi_write(W_NEUR_ADDR_C, slv32_from_u(j));
        axi_write(W_TILE_ADDR_C, slv32_from_u(t));

        for lane in 0 to TILE_LEN-1 loop
          readline(f_w, L);
          read(L, w_i);
          axi_write(W_LANE_BASE_C + integer(lane)*4, slv32_from_s16(w_i));
        end loop;

        axi_write(W_COMMIT_ADDR_C, x"00000001");
      end loop;
    end loop;

    -- ==========================================================
    -- Load biases once (AXI path)
    -- ==========================================================
    for j in 0 to N_OUT-1 loop
      readline(f_b, L);
      read(L, b_i);
      axi_write(BIAS_BASE_C + integer(j)*4, slv32_from_s16(b_i));
    end loop;

    -- Small settle
    for i in 0 to 10 loop
      tick;
    end loop;

    -- ==========================================================
    -- Run cases
    -- ==========================================================
    for k in 0 to G_N_CASES-1 loop

      -- Activations for this case
      for idx in 0 to VEC_LEN-1 loop
        readline(f_x, L);
        read(L, x_i);
        axi_write(ACTV_BASE_C + integer(idx)*4, slv32_from_s16(x_i));
      end loop;

      -- Expected logits for this case
      for j in 0 to N_OUT-1 loop
        readline(f_le, L);
        read(L, le_i);
        exp_logits(j) := to_signed(le_i, ACC_BITS_C);
      end loop;

      -- clear_done then start
      axi_write(CTRL_ADDR_C, x"00000002"); -- clear_done pulse
      axi_write(CTRL_ADDR_C, x"00000001"); -- start pulse

      -- Poll done via AXI STATUS
      poll_done;

      -- Optional cycles check
      if G_CHECK_CYCLES then
        axi_read(CYCLES_ADDR_C, cycles_w);
        assert unsigned(cycles_w) = to_unsigned(G_EXPECT_CYCLES, 32)
          report "TB FAIL: cycles_total mismatch. got=" &
                 integer'image(to_integer(unsigned(cycles_w))) &
                 " exp=" & integer'image(integer(G_EXPECT_CYCLES))
          severity failure;
      end if;

      -- Read logits window and compare
      for j in 0 to N_OUT-1 loop
        axi_read(LOGITS_BASE_C + integer(j)*8 + 0, lo_w);
        axi_read(LOGITS_BASE_C + integer(j)*8 + 4, hi_w);
        got_logits(j) := rebuild_logit(lo_w, hi_w);
      end loop;

      for j in 0 to N_OUT-1 loop
        err := to_integer(got_logits(j)) - to_integer(exp_logits(j));
        abs_e := err;
        if abs_e < 0 then abs_e := -abs_e; end if;

        assert abs_e <= G_BOUND_FINAL
          report "TB FAIL: case=" & integer'image(integer(k)) &
                 " j=" & integer'image(integer(j)) &
                 " got=" & integer'image(to_integer(got_logits(j))) &
                 " exp=" & integer'image(to_integer(exp_logits(j))) &
                 " err=" & integer'image(err) &
                 " bound=" & integer'image(G_BOUND_FINAL)
          severity failure;
      end loop;

      if (G_PROGRESS_EVERY /= 0) and ((k mod G_PROGRESS_EVERY) = 0) then
        report "TB: case=" & integer'image(integer(k)) &
               " PASS"
          severity note;
      end if;

      -- Idle gap between runs (PS-like)
      for i in 0 to 20 loop
        tick;
      end loop;

    end loop;

    file_close(f_w);
    file_close(f_b);
    file_close(f_x);
    file_close(f_le);

    report "TB PASSED: tb_msdf_accel_axi_top_axil_txt" severity note;
    assert false report "TB STOP" severity failure;
  end process;

end architecture;
