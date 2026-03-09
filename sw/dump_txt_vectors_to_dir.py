from __future__ import annotations

import os
import argparse
import numpy as np

from sw.fixedpoint_cfg import CFG
from sw.golden_fc import golden_fc_logits


def count_lines(path: str) -> int:
    n = 0
    with open(path, "r") as f:
        for _ in f:
            n += 1
    return n


def _require_key(d: np.lib.npyio.NpzFile, key: str) -> None:
    if key not in d.files:
        raise KeyError(f"NPZ missing required key '{key}'. Present keys: {d.files}")


def _range_limits(bits: int) -> tuple[int, int]:
    lo = -(1 << (bits - 1))
    hi = (1 << (bits - 1)) - 1
    return lo, hi


def _check_in_range(name: str, arr: np.ndarray, bits: int) -> None:
    lo, hi = _range_limits(bits)
    mn = int(arr.min())
    mx = int(arr.max())
    if mn < lo or mx > hi:
        raise ValueError(f"{name} out of range for {bits}-bit signed: min={mn} max={mx} allowed=[{lo},{hi}]")


def main() -> None:
    ap = argparse.ArgumentParser()

    # --- MODIFIED SECTION START ---
    # Changed required=True to defaults so the script runs easily in PyCharm
    default_npz = os.path.join("sw", "test_vectors", "mnist_fc784x10_seed12345_n1000.npz")
    default_out = os.path.join("sw", "test_vectors", "dump")

    ap.add_argument("--npz", type=str, default=default_npz,
                    help=f"Input .npz (Default: {default_npz})")
    ap.add_argument("--out_dir", type=str, default=default_out,
                    help=f"Output directory (Default: {default_out})")
    # --- MODIFIED SECTION END ---

    ap.add_argument("--n_cases", type=int, default=-1, help="Default: dump all cases in NPZ")
    ap.add_argument("--force", action="store_true", help="Allow overwriting an existing output directory")
    ap.add_argument("--verify_npz_logits", action="store_true",
                    help="If NPZ contains 'logits', assert it matches golden_fc_logits")
    args = ap.parse_args()

    # User feedback to show what is happening
    print(f"Running with:\n  Input NPZ: {args.npz}\n  Output Dir: {args.out_dir}")

    if not os.path.exists(args.npz):
        raise FileNotFoundError(f"NPZ not found: {args.npz}")

    if os.path.exists(args.out_dir):
        if not args.force:
            # Check if directory is empty, if so, we can use it, otherwise warn
            if os.listdir(args.out_dir):
                print(f"Warning: Output directory '{args.out_dir}' already exists and is not empty.")
                print("To overwrite, add --force argument or delete the folder.")
                # For convenience in IDE, we won't crash here if it exists,
                # but purely strictly we might want to. Let's rely on standard logic:
                # If you want to be strict, uncomment the next line:
                # raise FileExistsError(f"Refusing to overwrite: {args.out_dir} (use --force)")
    else:
        os.makedirs(args.out_dir, exist_ok=False)

    d = np.load(args.npz, allow_pickle=True)
    _require_key(d, "x_fixed")
    _require_key(d, "w_fixed")
    _require_key(d, "b_fixed")

    x_fixed = d["x_fixed"].astype(np.int64)  # (n_cases, VEC_LEN)
    w_fixed = d["w_fixed"].astype(np.int64)  # (N_OUT, VEC_LEN)
    b_fixed = d["b_fixed"].astype(np.int64)  # (N_OUT,)

    npz_logits = None
    if "logits" in d.files:
        npz_logits = d["logits"].astype(np.int64)

    assert w_fixed.shape == (CFG.N_OUT, CFG.VEC_LEN), f"w_fixed shape {w_fixed.shape}"
    assert b_fixed.shape == (CFG.N_OUT,), f"b_fixed shape {b_fixed.shape}"
    assert x_fixed.ndim == 2 and x_fixed.shape[1] == CFG.VEC_LEN, f"x_fixed shape {x_fixed.shape}"

    n_npz = int(x_fixed.shape[0])
    n_cases = n_npz if args.n_cases < 0 else min(int(args.n_cases), n_npz)
    if n_cases <= 0:
        raise ValueError("n_cases must be > 0")

    # Range checks: catch missed quantization or missed pre-shifts
    _check_in_range("x_fixed", x_fixed[:n_cases], CFG.A_TOTAL_BITS)
    _check_in_range("w_fixed", w_fixed, CFG.W_TOTAL_BITS)
    _check_in_range("b_fixed", b_fixed, CFG.B_TOTAL_BITS)

    if npz_logits is not None and args.verify_npz_logits:
        if npz_logits.shape != (n_npz, CFG.N_OUT):
            raise ValueError(f"NPZ logits shape {npz_logits.shape} expected ({n_npz},{CFG.N_OUT})")

    w_txt = os.path.join(args.out_dir, "w_fixed.txt")
    b_txt = os.path.join(args.out_dir, "b_fixed.txt")
    x_txt = os.path.join(args.out_dir, "x_fixed.txt")
    le_txt = os.path.join(args.out_dir, "logits_exp.txt")
    meta_txt = os.path.join(args.out_dir, "meta.txt")

    n_tiles = CFG.VEC_LEN // CFG.TILE_LEN

    # 1) Weights: tile-major order
    with open(w_txt, "w", newline="\n") as f:
        for j in range(CFG.N_OUT):
            for t in range(n_tiles):
                base = t * CFG.TILE_LEN
                for lane in range(CFG.TILE_LEN):
                    f.write(f"{int(w_fixed[j, base + lane])}\n")

    # 2) Bias
    with open(b_txt, "w", newline="\n") as f:
        for j in range(CFG.N_OUT):
            f.write(f"{int(b_fixed[j])}\n")

    # 3) Inputs
    with open(x_txt, "w", newline="\n") as f:
        for k in range(n_cases):
            for i in range(CFG.VEC_LEN):
                f.write(f"{int(x_fixed[k, i])}\n")

    # 4) Expected logits (golden)
    with open(le_txt, "w", newline="\n") as f:
        for k in range(n_cases):
            logits_k = golden_fc_logits(x_fixed[k], w_fixed, b_fixed, CFG)

            if npz_logits is not None and args.verify_npz_logits:
                if not np.array_equal(logits_k, npz_logits[k]):
                    raise AssertionError(f"NPZ logits mismatch at case {k}")

            for j in range(CFG.N_OUT):
                f.write(f"{int(logits_k[j])}\n")

    # 5) Meta: full numeric contract snapshot
    with open(meta_txt, "w", newline="\n") as f:
        f.write(f"npz={args.npz}\n")
        f.write(f"n_cases={n_cases}\n")
        f.write(f"VEC_LEN={CFG.VEC_LEN}\n")
        f.write(f"N_OUT={CFG.N_OUT}\n")
        f.write(f"TILE_LEN={CFG.TILE_LEN}\n")
        f.write(f"N_DIGITS={CFG.N_DIGITS}\n")
        f.write(f"W_FRAC_BITS={CFG.W_FRAC_BITS}\n")
        f.write(f"W_TOTAL_BITS={CFG.W_TOTAL_BITS}\n")
        f.write(f"A_FRAC_BITS={CFG.A_FRAC_BITS}\n")
        f.write(f"A_TOTAL_BITS={CFG.A_TOTAL_BITS}\n")
        f.write(f"B_FRAC_BITS={CFG.B_FRAC_BITS}\n")
        f.write(f"B_TOTAL_BITS={CFG.B_TOTAL_BITS}\n")
        f.write(f"WEIGHT_RSHIFT={CFG.WEIGHT_RSHIFT}\n")
        f.write(f"BIAS_RSHIFT={CFG.BIAS_RSHIFT}\n")
        f.write(f"QUANT_MODE={CFG.QUANT_MODE}\n")
        f.write(f"FC_SCALE_MODE={CFG.FC_SCALE_MODE}\n")

    # Sanity: enforce exact case counts
    lx = count_lines(x_txt)
    lle = count_lines(le_txt)

    if (lx % CFG.VEC_LEN) != 0:
        raise AssertionError(f"x_fixed.txt linecount {lx} not divisible by {CFG.VEC_LEN}")
    if (lle % CFG.N_OUT) != 0:
        raise AssertionError(f"logits_exp.txt linecount {lle} not divisible by {CFG.N_OUT}")

    n_cases_x = lx // CFG.VEC_LEN
    n_cases_le = lle // CFG.N_OUT

    if n_cases_x != n_cases_le:
        raise AssertionError(f"case mismatch: x={n_cases_x} logits_exp={n_cases_le}")
    if n_cases_x != n_cases:
        raise AssertionError(f"dumped {n_cases_x} cases but expected {n_cases}")

    print("OK")
    print("  out_dir =", os.path.abspath(args.out_dir))
    print("  cases   =", n_cases)


if __name__ == "__main__":
    main()