-- ============================================================
-- File:    msdf_accel_top.vhd
-- Purpose: Phase-5 top wrapper (pre-AXI)
--
-- Integrates:
-- . msdf_actv_mem     (784 deep)
-- . msdf_weight_mem   (tile read/write; TB-proven version)
-- . msdf_bias_mem     (10 deep, combinational read)
-- . msdf_fc_ctrl_10   (your controller)
-- . logit register bank (10 regs)
--
-- Exposes "host side" ports that Step-7 AXI-Lite regs will drive:
-- . write activations
-- . write bias
-- . write weights (tile-wise, matches the old weight_mem)
-- . start / clear_done
-- . read logits (packed) + cycles_total + busy/done
-- ============================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

entity msdf_accel_top is
  generic (
    ACC_BITS_G    : positive := 40;
    BIAS_BITS_G   : positive := 16;
    FLUSH_EXTRA_G : integer  := 2;
    SOFT_RESET_G  : boolean  := true
  );
  port (
    clk   : in  std_logic;
    rst_n : in  std_logic;   -- active-low synchronous reset
    ce    : in  std_logic;

    -- Host control (Step-7 will drive)
    start      : in  std_logic;
    clear_done : in  std_logic;

    busy : out std_logic;
    done : out std_logic;

    -- Host write: activations (784 words)
    actv_wr_en   : in  std_logic;
    actv_wr_addr : in  addr_idx_t;
    actv_wr_data : in  std_logic_vector(31 downto 0);

    -- Host write: bias (10 words)
    bias_wr_en   : in  std_logic;
    bias_wr_addr : in  neur_idx_t;
    bias_wr_data : in  std_logic_vector(31 downto 0);

    -- Host write: weights (tile-wise, matches TB-proven msdf_weight_mem)
    w_wr_en     : in  std_logic;
    w_wr_neuron : in  neur_idx_t;
    w_wr_tile   : in  tile_idx_t;
    w_wr_tile_i : in  w_fixed_vec_t;

    -- Readback
    cycles_total : out unsigned(31 downto 0);

    -- Packed logits: [j=0] in LSB chunk, [j=9] in MSB chunk
    logits_pack : out std_logic_vector(N_OUT*ACC_BITS_G-1 downto 0)
  );
end entity;

architecture rtl of msdf_accel_top is

  subtype acc_t is signed(ACC_BITS_G-1 downto 0);

  type logit_vec_t is array (0 to N_OUT-1) of acc_t;

  signal logits_r : logit_vec_t := (others => (others => '0'));

  -- Controller <-> memories
  signal w_rd_en_s     : std_logic;
  signal w_rd_neur_s   : neur_idx_t;
  signal w_rd_tile_s   : tile_idx_t;
  signal w_rd_valid_s  : std_logic;
  signal w_tile_s      : w_fixed_vec_t;

  signal x_rd_en_s     : std_logic;
  signal x_rd_addr_s   : addr_idx_t;
  signal x_rd_valid_s  : std_logic;
  signal x_data_s      : w_fixed_t;

  signal bias_sel_s    : neur_idx_t;
  signal bias_s        : signed(BIAS_BITS_G-1 downto 0);

  signal logit_we_s    : std_logic;
  signal logit_idx_s   : neur_idx_t;
  signal logit_data_s  : acc_t;

  signal busy_s        : std_logic;
  signal done_pulse_s  : std_logic;

  -- Done sticky for software-style polling
  signal done_sticky_r : std_logic := '0';

  function pack_logits(v : logit_vec_t) return std_logic_vector is
    variable p : std_logic_vector(N_OUT*ACC_BITS_G-1 downto 0) := (others => '0');
    variable lo : integer;
    variable hi : integer;
  begin
    for j in 0 to N_OUT-1 loop
      lo := j*ACC_BITS_G;
      hi := (j+1)*ACC_BITS_G - 1;
      p(hi downto lo) := std_logic_vector(v(j));
    end loop;
    return p;
  end function;

begin

  busy <= busy_s;
  done <= done_sticky_r;

  logits_pack <= pack_logits(logits_r);

  -- ============================================================
  -- Activation memory
  -- ============================================================
  u_actv : entity work.msdf_actv_mem
    port map (
      clk        => clk,
      rst_n      => rst_n,
      ce         => ce,

      wr_en      => actv_wr_en,
      wr_addr    => actv_wr_addr,
      wr_data    => actv_wr_data,

      x_rd_en    => x_rd_en_s,
      x_rd_addr  => x_rd_addr_s,
      x_rd_valid => x_rd_valid_s,
      x_o        => x_data_s
    );

  -- ============================================================
  -- Weight memory (use your TB-proven old module)
  -- ============================================================
  u_wmem : entity work.msdf_weight_mem
    port map (
      clk       => clk,
      rst_n     => rst_n,
      ce        => ce,

      rd_en     => w_rd_en_s,
      rd_neuron => w_rd_neur_s,
      rd_tile   => w_rd_tile_s,
      rd_valid  => w_rd_valid_s,
      w_tile_o  => w_tile_s,

      wr_en     => w_wr_en,
      wr_neuron => w_wr_neuron,
      wr_tile   => w_wr_tile,
      w_tile_i  => w_wr_tile_i
    );

  -- ============================================================
  -- Bias memory
  -- ============================================================
  u_bmem : entity work.msdf_bias_mem
    generic map (
      BIAS_BITS_G => BIAS_BITS_G
    )
    port map (
      clk      => clk,
      rst_n    => rst_n,
      ce       => ce,

      wr_en    => bias_wr_en,
      wr_addr  => bias_wr_addr,
      wr_data  => bias_wr_data,

      bias_sel => bias_sel_s,
      bias_o   => bias_s
    );

  -- ============================================================
  -- FC controller
  -- ============================================================
  u_ctrl : entity work.msdf_fc_ctrl_10
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

      start => start,
      busy  => busy_s,
      done  => done_pulse_s,

      w_rd_en     => w_rd_en_s,
      w_rd_neuron => w_rd_neur_s,
      w_rd_tile   => w_rd_tile_s,
      w_rd_valid  => w_rd_valid_s,
      w_tile_i    => w_tile_s,

      x_rd_en     => x_rd_en_s,
      x_rd_addr   => x_rd_addr_s,
      x_rd_valid  => x_rd_valid_s,
      x_i         => x_data_s,

      bias_sel    => bias_sel_s,
      bias_in     => bias_s,

      logit_we    => logit_we_s,
      logit_idx   => logit_idx_s,
      logit_data  => logit_data_s,

      cycles_total => cycles_total
    );

  -- ============================================================
  -- Logit register bank + sticky done
  -- ============================================================
  process(clk)
    variable j : integer;
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        logits_r       <= (others => (others => '0'));
        done_sticky_r  <= '0';

      elsif ce = '1' then
        -- clear done on explicit clear, and also clear when a new run starts
        if (clear_done = '1') or ((start = '1') and (busy_s = '0')) then
          done_sticky_r <= '0';
        end if;

        if done_pulse_s = '1' then
          done_sticky_r <= '1';
        end if;

        if logit_we_s = '1' then
          j := to_integer(logit_idx_s);
          if (j >= 0) and (j < N_OUT) then
            logits_r(j) <= logit_data_s;
          end if;
        end if;
      end if;
    end if;
  end process;

  -- synthesis translate_off
  assert (VEC_LEN mod TILE_LEN) = 0
    report "msdf_accel_top: VEC_LEN must be divisible by TILE_LEN"
    severity failure;
  -- synthesis translate_on

end architecture;
