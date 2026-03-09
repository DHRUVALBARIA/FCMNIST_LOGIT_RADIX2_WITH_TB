library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity tb_msdf_tile_stepper is
end entity;

architecture tb of tb_msdf_tile_stepper is
  constant CLK_PERIOD : time := 10 ns;

  -- TB-config (do NOT depend on package flush constants here)
  constant N_DIGITS_TB : integer := 16;
  constant FLUSH_TB    : integer := 2;

  signal clk   : std_logic := '0';
  signal rst_n : std_logic := '0';
  signal ce    : std_logic := '1';

  signal tile_start : std_logic := '0';
  signal in_ready   : std_logic := '0';

  signal active     : std_logic;
  signal step_fire  : std_logic;
  signal done_digits: std_logic;
  signal done_tile  : std_logic;

  signal digit_idx  : integer range 0 to N_DIGITS_TB-1;
  signal step_count : integer range 0 to (N_DIGITS_TB + FLUSH_TB);

  -- Tiny deterministic PRNG
  function next_seed(s : integer) return integer is
  begin
    return (s * 1103515245 + 12345) mod 2147483647;
  end function;

begin
  clk <= not clk after CLK_PERIOD/2;

  dut : entity work.msdf_tile_stepper
    generic map (
      N_DIGITS_G => N_DIGITS_TB,
      FLUSH_G    => FLUSH_TB
    )
    port map (
      clk         => clk,
      rst_n       => rst_n,
      ce          => ce,
      tile_start  => tile_start,
      in_ready    => in_ready,
      active      => active,
      step_fire   => step_fire,
      digit_idx   => digit_idx,
      step_count  => step_count,
      done_digits => done_digits,
      done_tile   => done_tile
    );

  stim : process
    procedure tick is
    begin
      wait until rising_edge(clk);
      wait for 1 ns;
    end procedure;

    procedure do_reset is
    begin
      rst_n <= '0';
      ce <= '1';
      tile_start <= '0';
      in_ready <= '0';
      tick; tick;
      rst_n <= '1';
      tick;
    end procedure;

    variable seed      : integer := 7;
    variable acc_steps : integer := 0;

    variable cyc : integer;
  begin
    do_reset;

    --------------------------------------------------------------------------
    -- Test 1: random in_ready stalls, must eventually finish after
    -- (N_DIGITS_TB + FLUSH_TB) accepted steps
    --------------------------------------------------------------------------
    tile_start <= '1';
    in_ready   <= '0';
    tick;
    tile_start <= '0';

    acc_steps := 0;

    for cyc in 0 to 5000 loop
      seed := next_seed(seed);
      if (seed mod 5) = 0 then
        in_ready <= '0';
      else
        in_ready <= '1';
      end if;

      tick;

      -- Count accepted steps using the DUT's own step_fire pulse
      if (ce = '1') and (step_fire = '1') then
        acc_steps := acc_steps + 1;
      end if;

      -- done_digits must occur exactly on the N_DIGITS-th accepted step
      if done_digits = '1' then
        assert acc_steps = N_DIGITS_TB
          report "TB ERROR(stepper): done_digits asserted at wrong accepted-step count"
          severity failure;
      end if;

      -- done_tile must occur exactly on total accepted steps
      if done_tile = '1' then
        assert acc_steps = (N_DIGITS_TB + FLUSH_TB)
          report "TB ERROR(stepper): done_tile asserted at wrong accepted-step count"
          severity failure;
        exit;
      end if;
    end loop;

    assert done_tile = '1'
      report "TB ERROR(stepper): did not finish within cycle budget"
      severity failure;

    --------------------------------------------------------------------------
    -- Test 2: ce=0 gating: no pulses should occur
    --------------------------------------------------------------------------
    do_reset;
    tile_start <= '1';
    in_ready   <= '1';
    tick;
    tile_start <= '0';

    ce <= '0';
    for cyc in 0 to 20 loop
      tick;
      assert step_fire = '0'
        report "TB ERROR(stepper): step_fire must be 0 when ce=0"
        severity failure;
      assert done_digits = '0'
        report "TB ERROR(stepper): done_digits must be 0 when ce=0"
        severity failure;
      assert done_tile = '0'
        report "TB ERROR(stepper): done_tile must be 0 when ce=0"
        severity failure;
    end loop;
    ce <= '1';
    tick;

    report "TB PASSED: tb_msdf_tile_stepper" severity note;
    wait;
  end process;

end architecture;
