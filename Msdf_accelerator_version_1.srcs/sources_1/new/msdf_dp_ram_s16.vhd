-- ============================================================
-- File:    msdf_dp_ram_s16.vhd
-- Purpose: Simple dual-port RAM, 16-bit data.
--          Port A: write-only (AXI side)
--          Port B: read-only  (FC engine side)
--
-- Behavior:
--  . Synchronous write
--  . Synchronous read with 1-cycle latency when ren='1'
--  . Out-of-range addresses are ignored on write and return 0 on read
--
-- Vivado BRAM inference:
--  . Synchronous read style + ram_style="block" encourages BRAM.
-- ============================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity msdf_dp_ram_s16 is
  generic (
    DEPTH_G      : positive := 784;  -- number of 16-bit words
    ADDR_BITS_G  : natural  := 10;   -- must cover DEPTH_G-1
    RAM_STYLE_G  : string   := "block"
  );
  port (
    clk   : in  std_logic;
    rst   : in  std_logic;  -- synchronous reset for read data reg only

    -- Port A: write (AXI side)
    we    : in  std_logic;
    waddr : in  unsigned(ADDR_BITS_G-1 downto 0);
    wdata : in  std_logic_vector(15 downto 0);

    -- Port B: read (FC side)
    ren   : in  std_logic;
    raddr : in  unsigned(ADDR_BITS_G-1 downto 0);
    rdata : out std_logic_vector(15 downto 0)
  );
end entity;

architecture rtl of msdf_dp_ram_s16 is

  type ram_t is array (0 to DEPTH_G-1) of std_logic_vector(15 downto 0);
  signal ram : ram_t := (others => (others => '0'));

  signal rdata_r : std_logic_vector(15 downto 0) := (others => '0');

  attribute ram_style : string;
  attribute ram_style of ram : signal is RAM_STYLE_G;

begin

  rdata <= rdata_r;

  process (clk)
    variable wi : integer;
    variable ri : integer;
  begin
    if rising_edge(clk) then

      -- Optional sync reset for output reg only
      if rst = '1' then
        rdata_r <= (others => '0');
      else
        -- Write port
        if we = '1' then
          wi := to_integer(waddr);
          if (wi >= 0) and (wi < DEPTH_G) then
            ram(wi) <= wdata;
          else
            -- synthesis translate_off
            assert false
              report "msdf_dp_ram_s16: write address out of range: " & integer'image(wi)
              severity warning;
            -- synthesis translate_on
          end if;
        end if;

        -- Read port (registered)
        if ren = '1' then
          ri := to_integer(raddr);
          if (ri >= 0) and (ri < DEPTH_G) then
            rdata_r <= ram(ri);
          else
            rdata_r <= (others => '0');
            -- synthesis translate_off
            assert false
              report "msdf_dp_ram_s16: read address out of range: " & integer'image(ri)
              severity warning;
            -- synthesis translate_on
          end if;
        end if;

      end if;
    end if;
  end process;

end architecture;
