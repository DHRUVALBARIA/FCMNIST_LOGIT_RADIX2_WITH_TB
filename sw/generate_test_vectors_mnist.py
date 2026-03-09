from __future__ import annotations
import os
import numpy as np
import torch
from torchvision import datasets, transforms

from sw.fixedpoint_cfg import CFG
from sw.golden_fc import (
    quantize_float_to_fixed,
    apply_rshift_signed,
    golden_fc_logits,
    argmax_label,
)


def hw_compatible_transform(img: torch.Tensor) -> torch.Tensor:
    """
    Matches the training transform:
    Maps [0,1] -> [-1 + EPS, 1 - EPS]
    """
    x = 2.0 * img - 1.0
    eps = 1.0 / (1 << CFG.A_FRAC_BITS)
    x = torch.clamp(x, -1.0 + eps, 1.0 - eps)
    return x.view(-1)


def main() -> None:
    seed = 12345
    n_cases = 1000

    model_npz = os.path.join("sw", "model", f"mnist_fc784x10_seed{seed}.npz")
    if not os.path.exists(model_npz):
        raise FileNotFoundError(f"Model file not found: {model_npz}. Run train_mnist_fc784x10.py first.")

    m = np.load(model_npz, allow_pickle=True)
    Wf = m["W"].astype(np.float64)  # (10,784)
    bf = m["b"].astype(np.float64)  # (10,)

    tfm = transforms.Compose([
        transforms.ToTensor(),
        transforms.Lambda(hw_compatible_transform)
    ])
    ds = datasets.MNIST(root="sw/datasets", train=False, download=True, transform=tfm)

    rng = np.random.default_rng(seed)
    idx = rng.choice(len(ds), size=n_cases, replace=False)

    x_float = np.zeros((n_cases, CFG.VEC_LEN), dtype=np.float64)
    y_true = np.zeros((n_cases,), dtype=np.int64)
    for k, ii in enumerate(idx):
        x, y = ds[int(ii)]
        x_float[k] = x.numpy().astype(np.float64)
        y_true[k] = int(y)

    x_fixed = quantize_float_to_fixed(x_float, CFG.A_FRAC_BITS, CFG.A_TOTAL_BITS, CFG.QUANT_MODE)
    w_fixed = quantize_float_to_fixed(Wf, CFG.W_FRAC_BITS, CFG.W_TOTAL_BITS, CFG.QUANT_MODE)
    b_fixed = quantize_float_to_fixed(bf, CFG.B_FRAC_BITS, CFG.B_TOTAL_BITS, CFG.QUANT_MODE)

    w_fixed = apply_rshift_signed(w_fixed, CFG.WEIGHT_RSHIFT)
    b_fixed = apply_rshift_signed(b_fixed, CFG.BIAS_RSHIFT)

    logits = np.zeros((n_cases, CFG.N_OUT), dtype=np.int64)
    labels = np.zeros((n_cases,), dtype=np.int64)
    for k in range(n_cases):
        logits[k] = golden_fc_logits(x_fixed[k], w_fixed, b_fixed, CFG)
        labels[k] = argmax_label(logits[k])

    # Path fix applied here as well
    os.makedirs(os.path.join("sw", "test_vectors"), exist_ok=True)
    out_path = os.path.join("sw", "test_vectors", f"mnist_fc784x10_seed{seed}_n{n_cases}.npz")

    np.savez(
        out_path,
        seed=np.int64(seed),
        idx=idx.astype(np.int64),
        y_true=y_true,
        W_float=Wf, b_float=bf,
        x_fixed=x_fixed, w_fixed=w_fixed, b_fixed=b_fixed,
        logits=logits, labels=labels,
        cfg=np.array([
            CFG.W_FRAC_BITS, CFG.W_TOTAL_BITS,
            CFG.A_FRAC_BITS, CFG.A_TOTAL_BITS,
            CFG.B_FRAC_BITS, CFG.B_TOTAL_BITS,
            CFG.TILE_LEN, CFG.N_DIGITS,
            CFG.WEIGHT_RSHIFT, CFG.BIAS_RSHIFT
        ], dtype=np.int64),
        fc_scale_mode=np.array([CFG.FC_SCALE_MODE], dtype=object),
        quant_mode=np.array([CFG.QUANT_MODE], dtype=object),
    )

    print("Saved:", out_path)
    print("Stats:")
    print("  x_fixed min/max:", int(x_fixed.min()), int(x_fixed.max()))

    # Validation check for the user
    min_val = -(1 << CFG.A_FRAC_BITS)
    print(f"  Check: Min value possible is {min_val}.")
    if int(x_fixed.min()) == min_val:
        print("  WARNING: x_fixed still contains exactly -1.0. Clamping might not be working.")
    else:
        print("  SUCCESS: x_fixed does not hit the negative boundary.")

    print("  w_fixed min/max:", int(w_fixed.min()), int(w_fixed.max()))
    print("  b_fixed min/max:", int(b_fixed.min()), int(b_fixed.max()))
    print("  logits  min/max:", int(logits.min()), int(logits.max()))
    print("  fixed labels sample:", labels[:10].tolist())
    print("  mnist y_true sample :", y_true[:10].tolist())


if __name__ == "__main__":
    main()