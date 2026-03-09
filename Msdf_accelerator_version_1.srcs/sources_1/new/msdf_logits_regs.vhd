library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.msdf_accel_pkg.all;

entity msdf_logits_regs is
  generic (
    ACC_BITS_G  : positive := 40;
    N_OUT_G     : positive := N_OUT;
    IDX_BITS_G  : positive := NEUR_BITS_C  -- FIX: avoid function call in generic default
  );
  port (
    clk   : in  std_logic;
    rst_n : in  std_logic;   -- active-low synchronous reset
    ce    : in  std_logic;

    clear : in  std_logic;   -- sync clear

    logit_we   : in  std_logic;
    logit_idx  : in  unsigned(IDX_BITS_G-1 downto 0);
    logit_data : in  signed(ACC_BITS_G-1 downto 0);

    done_pulse_in    : in  std_logic;
    cycles_total_in  : in  unsigned(31 downto 0);

    logits_packed_o  : out signed((ACC_BITS_G*N_OUT_G)-1 downto 0);
    valid_mask_o     : out std_logic_vector(N_OUT_G-1 downto 0);
    done_hold_o      : out std_logic;
    cycles_latched_o : out unsigned(31 downto 0)
  );
end entity;

architecture rtl of msdf_logits_regs is

  signal logits_r : signed((ACC_BITS_G*N_OUT_G)-1 downto 0) := (others => '0');
  signal mask_r   : std_logic_vector(N_OUT_G-1 downto 0)    := (others => '0');
  signal done_r   : std_logic := '0';
  signal cyc_r    : unsigned(31 downto 0) := (others => '0');

  procedure set_logit(
    signal p   : inout signed;
    constant i : in integer;
    constant v : in signed
  ) is
    variable lo : integer;
    variable hi : integer;
  begin
    lo := i * ACC_BITS_G;
    hi := (i+1) * ACC_BITS_G - 1;
    p(hi downto lo) <= resize(v, ACC_BITS_G);
  end procedure;

begin

  -- synthesis translate_off
  assert IDX_BITS_G = clog2_p(N_OUT_G)
    report "msdf_logits_regs: IDX_BITS_G mismatch. Expected " &
           integer'image(clog2_p(N_OUT_G)) & " got " & integer'image(IDX_BITS_G)
    severity failure;
  -- synthesis translate_on

  process(clk)
    variable idx_i : integer;
  begin
    if rising_edge(clk) then
      if rst_n = '0' then
        logits_r <= (others => '0');
        mask_r   <= (others => '0');
        done_r   <= '0';
        cyc_r    <= (others => '0');
      elsif ce = '1' then

        if clear = '1' then
          logits_r <= (others => '0');
          mask_r   <= (others => '0');
          done_r   <= '0';
          cyc_r    <= (others => '0');
        else
          if done_pulse_in = '1' then
            done_r <= '1';
            cyc_r  <= cycles_total_in;
          end if;

          if logit_we = '1' then
            idx_i := to_integer(logit_idx);

            -- synthesis translate_off
            assert (idx_i >= 0) and (idx_i < integer(N_OUT_G))
              report "msdf_logits_regs: logit_idx out of range: " & integer'image(idx_i)
              severity failure;
            -- synthesis translate_on

            set_logit(logits_r, idx_i, logit_data);
            mask_r(idx_i) <= '1';
          end if;
        end if;

      end if;
    end if;
  end process;

  logits_packed_o  <= logits_r;
  valid_mask_o     <= mask_r;
  done_hold_o      <= done_r;
  cycles_latched_o <= cyc_r;

end architecture;
