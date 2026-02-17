library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

entity msdf_weight_mem is
  port (
    clk   : in  std_logic;
    rst_n : in  std_logic;
    ce    : in  std_logic;

    -- Read (synchronous). Output valid is 1-cycle delayed.
    rd_en     : in  std_logic;
    rd_neuron : in  neur_idx_t;   -- FIX: constrained
    rd_tile   : in  tile_idx_t;   -- FIX: constrained
    rd_valid  : out std_logic;
    w_tile_o  : out w_fixed_vec_t;

    -- Write (for TB init now; later replaced by AXI write path)
    wr_en     : in  std_logic;
    wr_neuron : in  neur_idx_t;   -- FIX: constrained
    wr_tile   : in  tile_idx_t;   -- FIX: constrained
    w_tile_i  : in  w_fixed_vec_t
  );
end entity;

architecture rtl of msdf_weight_mem is

  constant N_TILES_C : natural := natural(N_TILES);
  constant N_OUT_C   : natural := natural(N_OUT);
  constant DEPTH_C   : natural := N_OUT_C * N_TILES_C;

  constant WPACK_BITS_C : natural := natural(TILE_LEN) * natural(W_TOTAL_BITS);
  subtype wpack_t is std_logic_vector(WPACK_BITS_C-1 downto 0);

  type mem_t is array (0 to DEPTH_C-1) of wpack_t;

  signal mem : mem_t := (others => (others => '0'));
  attribute ram_style : string;
  attribute ram_style of mem : signal is "block";

  signal rd_pack_r  : wpack_t := (others => '0');
  signal rd_valid_r : std_logic := '0';

  function pack_w_vec(wv : w_fixed_vec_t) return wpack_t is
    variable p : wpack_t := (others => '0');
  begin
    for i in 0 to TILE_LEN-1 loop
      p((i+1)*W_TOTAL_BITS-1 downto i*W_TOTAL_BITS) := std_logic_vector(wv(i));
    end loop;
    return p;
  end function;

  function unpack_w_vec(p : wpack_t) return w_fixed_vec_t is
    variable wv : w_fixed_vec_t;
  begin
    for i in 0 to TILE_LEN-1 loop
      wv(i) := signed(p((i+1)*W_TOTAL_BITS-1 downto i*W_TOTAL_BITS));
    end loop;
    return wv;
  end function;

  function lin_idx(neuron_u : neur_idx_t; tile_u : tile_idx_t) return natural is
    variable j   : natural;
    variable t   : natural;
    variable idx : natural;
  begin
    j := natural(to_integer(neuron_u));
    t := natural(to_integer(tile_u));

    -- synthesis translate_off
    assert j < N_OUT_C
      report "msdf_weight_mem: neuron index out of range: " & integer'image(j)
      severity failure;
    assert t < N_TILES_C
      report "msdf_weight_mem: tile index out of range: " & integer'image(t)
      severity failure;
    -- synthesis translate_on

    idx := j * N_TILES_C + t;
    return idx;
  end function;

begin

  -- synthesis translate_off
  assert (VEC_LEN mod TILE_LEN) = 0
    report "msdf_weight_mem: VEC_LEN must be divisible by TILE_LEN"
    severity failure;
  -- synthesis translate_on

  process(clk)
    variable ridx : natural;
    variable widx : natural;
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        rd_pack_r  <= (others => '0');
        rd_valid_r <= '0';
      elsif ce = '1' then
        rd_valid_r <= rd_en;

        if rd_en = '1' then
          ridx := lin_idx(rd_neuron, rd_tile);
          rd_pack_r <= mem(ridx);
        end if;

        if wr_en = '1' then
          widx := lin_idx(wr_neuron, wr_tile);
          mem(widx) <= pack_w_vec(w_tile_i);
        end if;
      end if;
    end if;
  end process;

  rd_valid <= rd_valid_r;                 -- IMPORTANT: don’t omit this
  w_tile_o <= unpack_w_vec(rd_pack_r);

end architecture;
