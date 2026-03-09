---- ============================================================
---- File:    msdf_weight_mem.vhd
---- Purpose: Weight memory wrapper for msdf_fc_ctrl_10
----
---- Delivers:
----   . w_tile_o : w_fixed_vec_t (TILE_LEN weights at once)
----   . w_rd_valid pulses 1 cycle after w_rd_en (synchronous RAM read)
----
---- Implementation choice (important):
----   . Banked-by-lane RAMs: TILE_LEN banks, each stores one lane's weights.
----   . Each bank depth = N_OUT * N_TILES (for MNIST: 10*98 = 980).
----   . This allows reading an entire tile vector in one cycle.
----
---- Write addressing (flat linear):
----   . wr_addr spans 0 .. (N_OUT*VEC_LEN - 1) = 7839
----   . Decode:
----       word  = wr_addr
----       j     = word / VEC_LEN
----       i     = word mod VEC_LEN
----       tile  = i / TILE_LEN
----       lane  = i mod TILE_LEN
----       bank_addr = j*N_TILES + tile
----     -> write goes to bank[lane] at bank_addr
----
---- Read addressing (controller-side):
----   . bank_addr = to_integer(w_rd_neuron)*N_TILES + to_integer(w_rd_tile)
----   . ren asserted to all banks simultaneously
----   . next cycle: rdata from all banks forms w_tile_o
---- ============================================================

--library ieee;
--use ieee.std_logic_1164.all;
--use ieee.numeric_std.all;

--use work.msdf_mul_pkg_r2_core.all;
--use work.msdf_accel_pkg.all;

--entity msdf_weight_mem_flataxi is
--  port (
--    clk   : in  std_logic;
--    rst_n : in  std_logic;  -- active-low synchronous reset
--    ce    : in  std_logic;

--    -- Write interface (AXI-Lite regs later)
--    wr_en   : in  std_logic;
--    wr_addr : in  unsigned(clog2_p(N_OUT*VEC_LEN)-1 downto 0);  -- 0..7839
--    wr_data : in  std_logic_vector(31 downto 0);

--    -- Read interface (from msdf_fc_ctrl_10)
--    w_rd_en     : in  std_logic;
--    w_rd_neuron : in  neur_idx_t;
--    w_rd_tile   : in  tile_idx_t;

--    w_rd_valid  : out std_logic;
--    w_tile_o    : out w_fixed_vec_t
--  );
--end entity;

--architecture rtl of msdf_weight_mem_flataxi is

--  constant W_DEPTH_C      : natural := N_OUT * VEC_LEN;   -- 7840
--  constant W_ADDR_BITS_C  : natural := clog2_p(W_DEPTH_C);

--  constant BANK_DEPTH_C     : natural := N_OUT * N_TILES; -- 980
--  constant BANK_ADDR_BITS_C : natural := clog2_p(BANK_DEPTH_C);

--  subtype bank_addr_t is unsigned(BANK_ADDR_BITS_C-1 downto 0);

--  type slv16_vec_t is array (0 to TILE_LEN-1) of std_logic_vector(15 downto 0);
--  type sl_vec_t2   is array (0 to TILE_LEN-1) of std_logic;

--  signal rst_sync : std_logic;

--  -- Per-lane bank write enables
--  signal we_bank   : sl_vec_t2 := (others => '0');
--  signal waddr_bank: bank_addr_t := (others => '0');
--  signal wdata16   : std_logic_vector(15 downto 0) := (others => '0');

--  -- Read control (common addr to all banks)
--  signal ren_s     : std_logic := '0';
--  signal raddr_s   : bank_addr_t := (others => '0');

--  -- Per-lane bank read data
--  signal rdata16_bank : slv16_vec_t := (others => (others => '0'));

--  -- 1-cycle valid pipeline
--  signal vld_r : std_logic := '0';

--begin

--  rst_sync <= not rst_n;

--  -- Default outputs
--  w_rd_valid <= vld_r;

--  -- Convert each bank's 16-bit signed value to w_fixed_t (14-bit)
--  gen_out : for lane in 0 to TILE_LEN-1 generate
--    w_tile_o(lane) <= resize(signed(rdata16_bank(lane)), W_TOTAL_BITS);
--  end generate;

--  -- ============================================================
--  -- Read side: one common address, read all banks in parallel
--  -- ============================================================
--  process(all)
--    variable j_i    : integer;
--    variable t_i    : integer;
--    variable addr_i : integer;
--  begin
--    ren_s   <= '0';
--    raddr_s <= (others => '0');

--    if (w_rd_en = '1') and (ce = '1') then
--      j_i := to_integer(w_rd_neuron);
--      t_i := to_integer(w_rd_tile);

--      addr_i := j_i * integer(N_TILES) + t_i;
--      raddr_s <= to_unsigned(addr_i, BANK_ADDR_BITS_C);
--      ren_s   <= '1';
--    end if;
--  end process;

--  -- 1-cycle delayed valid for synchronous read
--  process(clk)
--  begin
--    if rising_edge(clk) then
--      if rst_n = '0' then
--        vld_r <= '0';
--      else
--        if ce = '1' then
--          vld_r <= ren_s;
--        else
--          vld_r <= '0';
--        end if;
--      end if;
--    end if;
--  end process;

--  -- ============================================================
--  -- Write side: decode flat address -> (j,tile,lane) -> bank write
--  -- ============================================================
--  process(all)
--    variable word_i : integer;
--    variable j_i    : integer;
--    variable i_i    : integer;
--    variable tile_i : integer;
--    variable lane_i : integer;
--    variable baddr_i: integer;
--  begin
--    we_bank    <= (others => '0');
--    waddr_bank <= (others => '0');
--    wdata16    <= wr_data(15 downto 0);

--    if wr_en = '1' then
--      word_i := to_integer(wr_addr);

--      -- Decode neuron + in-neuron index
--      j_i := word_i / integer(VEC_LEN);
--      i_i := word_i mod integer(VEC_LEN);

--      -- Decode tile + lane within tile
--      tile_i := i_i / integer(TILE_LEN);
--      lane_i := i_i mod integer(TILE_LEN);

--      -- Bank address per lane bank
--      baddr_i := j_i * integer(N_TILES) + tile_i;

--      if (lane_i >= 0) and (lane_i < integer(TILE_LEN)) then
--        we_bank(lane_i) <= '1';
--      end if;

--      waddr_bank <= to_unsigned(baddr_i, BANK_ADDR_BITS_C);
--    end if;
--  end process;

--  -- ============================================================
--  -- Instantiate TILE_LEN RAM banks
--  -- ============================================================
--  gen_banks : for lane in 0 to TILE_LEN-1 generate
--    u_bank : entity work.msdf_dp_ram_s16
--      generic map (
--        DEPTH_G     => BANK_DEPTH_C,      -- 980
--        ADDR_BITS_G => BANK_ADDR_BITS_C,  -- 10
--        RAM_STYLE_G => "block"
--      )
--      port map (
--        clk   => clk,
--        rst   => rst_sync,

--        -- Port A write
--        we    => we_bank(lane),
--        waddr => waddr_bank,
--        wdata => wdata16,

--        -- Port B read (common)
--        ren   => ren_s,
--        raddr => raddr_s,
--        rdata => rdata16_bank(lane)
--      );
--  end generate;

--  -- synthesis translate_off
--  assert (VEC_LEN mod TILE_LEN) = 0
--    report "msdf_weight_mem: VEC_LEN must be divisible by TILE_LEN"
--    severity failure;

--  assert (W_DEPTH_C = 7840)
--    report "msdf_weight_mem: expected MNIST demo W_DEPTH=7840"
--    severity note;

--  assert (BANK_DEPTH_C = 980)
--    report "msdf_weight_mem: expected MNIST demo bank depth=980"
--    severity note;
--  -- synthesis translate_on

--end architecture;
