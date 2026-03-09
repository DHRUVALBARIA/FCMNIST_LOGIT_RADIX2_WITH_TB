library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

-- ============================================================================
-- File : msdf_fc_ctrl_10.vhd
-- Phase 4.2: FC controller for 784 -> 10 (MNIST FC layer)
--
-- Instrumentation behavior:
--   . cycles_total (cyc_r) is reset ONLY when a run starts (start seen in ST_IDLE)
--   . cyc_r is NOT cleared in ST_IDLE every cycle (so regs can latch a nonzero value)
-- ============================================================================

entity msdf_fc_ctrl_10 is
  generic (
    ACC_BITS_G    : positive := 40;
    BIAS_BITS_G   : positive := 16;
    FLUSH_EXTRA_G : integer  := 2;
    SOFT_RESET_G  : boolean  := true
  );
  port (
    clk   : in  std_logic;
    rst_n : in  std_logic;  -- active-low synchronous reset
    ce    : in  std_logic;

    -- Control
    start : in  std_logic;
    busy  : out std_logic;
    done  : out std_logic;

    -- Weight memory read interface
    w_rd_en     : out std_logic;
    w_rd_neuron : out neur_idx_t;
    w_rd_tile   : out tile_idx_t;
    w_rd_valid  : in  std_logic;
    w_tile_i    : in  w_fixed_vec_t;

    -- Activation buffer read interface
    x_rd_en    : out std_logic;
    x_rd_addr  : out addr_idx_t;
    x_rd_valid : in  std_logic;
    x_i        : in  w_fixed_t;

    -- Bias supply
    bias_sel : out neur_idx_t;
    bias_in  : in  signed(BIAS_BITS_G-1 downto 0);

    -- Logit write-out
    logit_we   : out std_logic;
    logit_idx  : out neur_idx_t;
    logit_data : out signed(ACC_BITS_G-1 downto 0);

    -- Optional profiling
    cycles_total : out unsigned(31 downto 0)
  );
end entity;

architecture rtl of msdf_fc_ctrl_10 is

  subtype acc_t is signed(ACC_BITS_G-1 downto 0);

  type st_t is (
    ST_IDLE,
    ST_INIT_NEURON,

    ST_ISSUE_W,
    ST_CAP_W,

    ST_ISSUE_X,
    ST_CAP_X,

    ST_START_TILE,
    ST_RUN_TILE,

    ST_ACCUM_TILE,

    ST_ADD_BIAS,
    ST_WRITE_LOGIT,

    ST_NEXT_NEURON,
    ST_DONE
  );

  signal st : st_t := ST_IDLE;

  signal j_cnt    : neur_idx_t := (others => '0');
  signal t_cnt    : tile_idx_t := (others => '0');
  signal lane_cnt : lane_idx_t := (others => '0');

  signal x_tile_r : w_fixed_vec_t := (others => (others => '0'));
  signal w_tile_r : w_fixed_vec_t := (others => (others => '0'));

  signal acc_r : acc_t := (others => '0');

  -- Cycle counter (profiling)
  signal cyc_r : unsigned(31 downto 0) := (others => '0');

  signal w_rd_en_r   : std_logic := '0';
  signal x_rd_en_r   : std_logic := '0';
  signal w_neur_r    : neur_idx_t := (others => '0');
  signal w_tile_ridx : tile_idx_t := (others => '0');
  signal x_addr_r    : addr_idx_t := (others => '0');

  signal logit_we_r : std_logic := '0';
  signal done_r     : std_logic := '0';

  signal tile_start_s : std_logic := '0';
  signal in_ready_s   : std_logic := '1';

  signal tile_active_s : std_logic;
  signal done_tile_s   : std_logic;

  signal sum_d : sd_digit_t := SD_ZERO;
  signal sum_v : std_logic  := '0';

  signal tile_val_s : tile_recon_t := (others => '0');
  signal tile_vld_s : std_logic    := '0';

  signal seen_done_tile : std_logic := '0';
  signal seen_tile_vld  : std_logic := '0';
  signal tile_val_lat   : tile_recon_t := (others => '0');

  constant LAST_NEUR_C : neur_idx_t := to_unsigned(N_OUT-1, NEUR_BITS_C);
  constant LAST_TILE_C : tile_idx_t := to_unsigned(N_TILES-1, TILE_BITS_C);
  constant LAST_LANE_C : lane_idx_t := to_unsigned(TILE_LEN-1, LANE_BITS_C);

begin

  w_rd_en     <= w_rd_en_r;
  x_rd_en     <= x_rd_en_r;

  w_rd_neuron <= w_neur_r;
  w_rd_tile   <= w_tile_ridx;

  x_rd_addr   <= x_addr_r;

  bias_sel    <= j_cnt;

  logit_we    <= logit_we_r;
  logit_idx   <= j_cnt;
  logit_data  <= acc_r;

  busy        <= '0' when (st = ST_IDLE) else '1';
  done        <= done_r;

  cycles_total <= cyc_r;

  u_tile : entity work.msdf_tile_sop
    generic map (
      FLUSH_EXTRA_G => FLUSH_EXTRA_G,
      SOFT_RESET_G  => SOFT_RESET_G
    )
    port map (
      clk           => clk,
      rst_n         => rst_n,
      ce            => ce,

      tile_start    => tile_start_s,
      in_ready      => in_ready_s,

      x_tile_in     => x_tile_r,
      w_tile_in     => w_tile_r,

      active        => tile_active_s,
      done_tile     => done_tile_s,

      sum_digit_out => sum_d,
      sum_valid_out => sum_v
    );

  u_recon : entity work.msdf_tile_recon_r2
    generic map (
      N_DIGITS_G   => N_DIGITS,
      CA_BITS_G    => CA_BITS,
      OUT_BITS_G   => TILE_RECON_BITS,
      SHIFT_LEFT_G => 0
    )
    port map (
      clk            => clk,
      rst_n          => rst_n,
      ce             => ce,

      start          => tile_start_s,
      step           => sum_v,
      d_in           => sum_d,

      tile_value_out => tile_val_s,
      tile_valid     => tile_vld_s,
      busy           => open,
      digit_count    => open
    );

  -- synthesis translate_off
  assert (VEC_LEN mod TILE_LEN) = 0
    report "msdf_fc_ctrl_10: VEC_LEN must be divisible by TILE_LEN"
    severity failure;
  -- synthesis translate_on

  process(clk)
    variable base_u : addr_idx_t;
    variable addr_u : addr_idx_t;
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        st <= ST_IDLE;

        j_cnt    <= (others => '0');
        t_cnt    <= (others => '0');
        lane_cnt <= (others => '0');

        x_tile_r <= (others => (others => '0'));
        w_tile_r <= (others => (others => '0'));

        acc_r <= (others => '0');

        w_rd_en_r    <= '0';
        x_rd_en_r    <= '0';
        w_neur_r     <= (others => '0');
        w_tile_ridx  <= (others => '0');
        x_addr_r     <= (others => '0');

        tile_start_s <= '0';
        in_ready_s   <= '1';

        seen_done_tile <= '0';
        seen_tile_vld  <= '0';
        tile_val_lat   <= (others => '0');

        logit_we_r <= '0';
        done_r     <= '0';

        cyc_r <= (others => '0');

      elsif ce = '1' then
        w_rd_en_r    <= '0';
        x_rd_en_r    <= '0';
        tile_start_s <= '0';
        logit_we_r   <= '0';
        done_r       <= '0';

        -- count cycles only when not IDLE (do NOT clear in IDLE)
        if st /= ST_IDLE then
          cyc_r <= cyc_r + 1;
        end if;

        if st = ST_RUN_TILE then
          if done_tile_s = '1' then
            seen_done_tile <= '1';
          end if;
          if tile_vld_s = '1' then
            seen_tile_vld <= '1';
            tile_val_lat  <= tile_val_s;
          end if;
        end if;

        case st is

          when ST_IDLE =>
            acc_r <= (others => '0');

            -- reset cycles only at run start
            if start = '1' then
              cyc_r <= (others => '0');
              j_cnt <= (others => '0');
              st    <= ST_INIT_NEURON;
            end if;

          when ST_INIT_NEURON =>
            acc_r <= (others => '0');
            t_cnt <= (others => '0');
            st    <= ST_ISSUE_W;

          when ST_ISSUE_W =>
            w_neur_r     <= j_cnt;
            w_tile_ridx  <= t_cnt;
            w_rd_en_r    <= '1';
            st           <= ST_CAP_W;

          when ST_CAP_W =>
            if w_rd_valid = '1' then
              w_tile_r  <= w_tile_i;
              lane_cnt  <= (others => '0');
              st        <= ST_ISSUE_X;
            end if;

          when ST_ISSUE_X =>
            base_u := shift_left(resize(t_cnt, ADDR_BITS_C), LANE_BITS_C);
            addr_u := base_u + resize(lane_cnt, ADDR_BITS_C);

            x_addr_r  <= addr_u;
            x_rd_en_r <= '1';
            st        <= ST_CAP_X;

          when ST_CAP_X =>
            if x_rd_valid = '1' then
              x_tile_r(to_integer(lane_cnt)) <= x_i;

              if lane_cnt = LAST_LANE_C then
                st <= ST_START_TILE;
              else
                lane_cnt <= lane_cnt + 1;
                st <= ST_ISSUE_X;
              end if;
            end if;

          when ST_START_TILE =>
            tile_start_s   <= '1';
            in_ready_s     <= '1';
            seen_done_tile <= '0';
            seen_tile_vld  <= '0';
            tile_val_lat   <= (others => '0');
            st             <= ST_RUN_TILE;

          when ST_RUN_TILE =>
            if (seen_done_tile = '1') and (seen_tile_vld = '1') then
              st <= ST_ACCUM_TILE;
            end if;

          when ST_ACCUM_TILE =>
            acc_r <= acc_r + resize(tile_val_lat, ACC_BITS_G);

            if t_cnt = LAST_TILE_C then
              st <= ST_ADD_BIAS;
            else
              t_cnt <= t_cnt + 1;
              st <= ST_ISSUE_W;
            end if;

          when ST_ADD_BIAS =>
            acc_r <= acc_r + resize(bias_in, ACC_BITS_G);
            st <= ST_WRITE_LOGIT;

          when ST_WRITE_LOGIT =>
            logit_we_r <= '1';
            st <= ST_NEXT_NEURON;

          when ST_NEXT_NEURON =>
            if j_cnt = LAST_NEUR_C then
              st <= ST_DONE;
            else
              j_cnt <= j_cnt + 1;
              st <= ST_INIT_NEURON;
            end if;

          when ST_DONE =>
            done_r <= '1';
            st <= ST_IDLE;

          when others =>
            st <= ST_IDLE;

        end case;
      end if;
    end if;
  end process;

end architecture;
