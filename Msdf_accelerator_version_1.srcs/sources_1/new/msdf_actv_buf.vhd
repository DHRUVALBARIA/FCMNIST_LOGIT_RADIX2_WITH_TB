library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_mul_pkg_r2_core.all;
use work.msdf_accel_pkg.all;

entity msdf_actv_buf is
  port (
    clk   : in  std_logic;
    rst_n : in  std_logic;
    ce    : in  std_logic;

    -- Write one activation (TB init now, AXI later)
    wr_en   : in  std_logic;
    wr_addr : in  addr_idx_t;     -- FIX: constrained width from msdf_accel_pkg
    x_i     : in  w_fixed_t;

    -- Read one activation (synchronous)
    rd_en    : in  std_logic;
    rd_addr  : in  addr_idx_t;    -- FIX: constrained width from msdf_accel_pkg
    rd_valid : out std_logic;
    x_o      : out w_fixed_t
  );
end entity;

architecture rtl of msdf_actv_buf is

  constant DEPTH_C : natural := natural(VEC_LEN);

  type mem_t is array (0 to DEPTH_C-1) of w_fixed_t;

  signal mem : mem_t := (others => (others => '0'));
  attribute ram_style : string;
  attribute ram_style of mem : signal is "block";

  signal x_r        : w_fixed_t := (others => '0');
  signal rd_valid_r : std_logic := '0';

begin

  -- synthesis translate_off
  assert (VEC_LEN > 0) report "msdf_actv_buf: VEC_LEN must be > 0" severity failure;
  -- synthesis translate_on

  process(clk)
    variable wa : natural;
    variable ra : natural;
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        x_r        <= (others => '0');
        rd_valid_r <= '0';
      elsif ce = '1' then
        rd_valid_r <= rd_en;

        if rd_en = '1' then
          ra := natural(to_integer(rd_addr));

          -- synthesis translate_off
          assert ra < DEPTH_C
            report "msdf_actv_buf: rd_addr out of range: " & integer'image(ra)
            severity failure;
          -- synthesis translate_on

          x_r <= mem(ra);
        end if;

        if wr_en = '1' then
          wa := natural(to_integer(wr_addr));

          -- synthesis translate_off
          assert wa < DEPTH_C
            report "msdf_actv_buf: wr_addr out of range: " & integer'image(wa)
            severity failure;
          -- synthesis translate_on

          mem(wa) <= x_i;
        end if;
      end if;
    end if;
  end process;

  rd_valid <= rd_valid_r;
  x_o      <= x_r;

end architecture;
