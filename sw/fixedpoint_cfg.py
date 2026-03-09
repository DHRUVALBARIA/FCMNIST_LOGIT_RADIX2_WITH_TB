from __future__ import annotations
from dataclasses import dataclass

@dataclass(frozen=True)
class FixedCfg:
    # Match VHDL msdf_mul_pkg_r2_core / msdf_accel_pkg
    W_FRAC_BITS: int = 12
    W_TOTAL_BITS: int = 14

    # Activations use w_fixed_t in RTL
    A_FRAC_BITS: int = 12
    A_TOTAL_BITS: int = 14

    # Bias domain (pre-shifted by BIAS_RSHIFT before TB/RTL add)
    B_FRAC_BITS: int = 12
    B_TOTAL_BITS: int = 16

    ACC_BITS: int = 40

    VEC_LEN: int = 784
    N_OUT: int = 10

    TILE_LEN: int = 8

    # For traceability: set to your Phase-4 locked point.
    # SW golden logits do not depend on N_DIGITS, but meta should match HW config.
    N_DIGITS: int = 18

    WEIGHT_RSHIFT: int = 2
    BIAS_RSHIFT: int = 2

    QUANT_MODE: str = "round"  # "round" or "trunc0"

    # FC arithmetic contract (must match HW)
    # tile_sumraw_ashr:
    #   tile = (sum(x*w)) >> W_FRAC_BITS  (arithmetic shift)
    FC_SCALE_MODE: str = "tile_sumraw_ashr"

    @property
    def ONE_W(self) -> int:
        return 1 << self.W_FRAC_BITS

    @property
    def ONE_A(self) -> int:
        return 1 << self.A_FRAC_BITS

    @property
    def ONE_B(self) -> int:
        return 1 << self.B_FRAC_BITS


CFG = FixedCfg()
