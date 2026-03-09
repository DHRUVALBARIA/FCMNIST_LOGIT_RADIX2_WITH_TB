from __future__ import annotations
import numpy as np
from sw.fixedpoint_cfg import CFG, FixedCfg

def _range_limits(total_bits: int) -> tuple[int, int]:
    lo = -(1 << (total_bits - 1))
    hi = (1 << (total_bits - 1)) - 1
    return lo, hi

def quantize_float_to_fixed(x: np.ndarray, frac_bits: int, total_bits: int, mode: str) -> np.ndarray:
    scale = 1 << frac_bits
    xf = x.astype(np.float64) * scale

    if mode == "round":
        # np.rint is deterministic; ties-to-even.
        xi = np.rint(xf).astype(np.int64)
    elif mode == "trunc0":
        xi = np.trunc(xf).astype(np.int64)
    else:
        raise ValueError(f"Unknown QUANT_MODE: {mode}")

    lo, hi = _range_limits(total_bits)
    xi = np.clip(xi, lo, hi)
    return xi.astype(np.int64)

def apply_rshift_signed(arr: np.ndarray, rshift: int) -> np.ndarray:
    a = arr.astype(np.int64, copy=False)
    if rshift <= 0:
        return a
    # Arithmetic shift for signed int64, matches VHDL shift_right(signed,...)
    return (a >> int(rshift)).astype(np.int64)

def golden_fc_logits(
    x_fixed: np.ndarray,           # (VEC_LEN,)
    w_fixed: np.ndarray,           # (N_OUT, VEC_LEN)
    b_fixed: np.ndarray,           # (N_OUT,)
    cfg: FixedCfg = CFG,
) -> np.ndarray:
    x_fixed = np.asarray(x_fixed, dtype=np.int64)
    w_fixed = np.asarray(w_fixed, dtype=np.int64)
    b_fixed = np.asarray(b_fixed, dtype=np.int64)

    assert x_fixed.shape == (cfg.VEC_LEN,), f"x_fixed shape {x_fixed.shape} != ({cfg.VEC_LEN},)"
    assert w_fixed.shape == (cfg.N_OUT, cfg.VEC_LEN), f"w_fixed shape {w_fixed.shape} != ({cfg.N_OUT},{cfg.VEC_LEN})"
    assert b_fixed.shape == (cfg.N_OUT,), f"b_fixed shape {b_fixed.shape} != ({cfg.N_OUT},)"

    if cfg.FC_SCALE_MODE != "tile_sumraw_ashr":
        raise ValueError(f"Unsupported FC_SCALE_MODE={cfg.FC_SCALE_MODE}. Expected 'tile_sumraw_ashr'.")

    logits = np.zeros((cfg.N_OUT,), dtype=np.int64)
    n_tiles = cfg.VEC_LEN // cfg.TILE_LEN

    for j in range(cfg.N_OUT):
        acc = 0  # Python int
        for t in range(n_tiles):
            base = t * cfg.TILE_LEN

            tile_raw = 0
            for lane in range(cfg.TILE_LEN):
                xi = int(x_fixed[base + lane])
                wi = int(w_fixed[j, base + lane])
                tile_raw += xi * wi

            # Shift once after summing raw products
            tile_q = int(tile_raw >> cfg.W_FRAC_BITS)
            acc += tile_q

        # Bias is expected to be pre-shifted by BIAS_RSHIFT already
        acc += int(b_fixed[j])
        logits[j] = np.int64(acc)

    return logits

def argmax_label(logits: np.ndarray) -> int:
    return int(np.argmax(np.asarray(logits)))

def self_check() -> None:
    rng = np.random.default_rng(1)
    x = rng.uniform(-0.9, 0.9, size=(CFG.VEC_LEN,))
    w = rng.uniform(-0.5, 0.5, size=(CFG.N_OUT, CFG.VEC_LEN))
    b = rng.uniform(-0.2, 0.2, size=(CFG.N_OUT,))

    xq = quantize_float_to_fixed(x, CFG.A_FRAC_BITS, CFG.A_TOTAL_BITS, CFG.QUANT_MODE)
    wq = quantize_float_to_fixed(w, CFG.W_FRAC_BITS, CFG.W_TOTAL_BITS, CFG.QUANT_MODE)
    bq = quantize_float_to_fixed(b, CFG.B_FRAC_BITS, CFG.B_TOTAL_BITS, CFG.QUANT_MODE)

    wq = apply_rshift_signed(wq, CFG.WEIGHT_RSHIFT)
    bq = apply_rshift_signed(bq, CFG.BIAS_RSHIFT)

    logits = golden_fc_logits(xq, wq, bq, CFG)
    print("Self-check logits stats:", int(logits.min()), int(logits.max()), "label=", argmax_label(logits))
    print("FC_SCALE_MODE =", CFG.FC_SCALE_MODE)

if __name__ == "__main__":
    self_check()
