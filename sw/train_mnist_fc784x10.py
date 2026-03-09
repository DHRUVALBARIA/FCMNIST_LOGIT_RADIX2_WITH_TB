from __future__ import annotations
import os
import random
import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader
from torchvision import datasets, transforms
from sw.fixedpoint_cfg import CFG  # Imported to get fractional bits


def set_deterministic(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False
    torch.use_deterministic_algorithms(True)


def hw_compatible_transform(img: torch.Tensor) -> torch.Tensor:
    """
    Standard MNIST is [0, 1].
    We map to approx [-1, 1], but we MUST avoid exactly -1.0 or +1.0
    because the hardware digit generator (MSDF) requires an open interval.

    We clamp to [-1 + EPS, 1 - EPS].
    For 12 fractional bits, EPS = 1/4096.
    """
    # 1. Convert [0,1] -> [-1, 1]
    x = 2.0 * img - 1.0

    # 2. Calculate Epsilon (1 LSB)
    eps = 1.0 / (1 << CFG.A_FRAC_BITS)

    # 3. Clamp safely inside the boundary
    x = torch.clamp(x, -1.0 + eps, 1.0 - eps)

    # 4. Flatten to (784,)
    return x.view(-1)


def main() -> None:
    seed = 12345
    set_deterministic(seed)

    # Use the new hardware-safe transform
    tfm = transforms.Compose([
        transforms.ToTensor(),
        transforms.Lambda(hw_compatible_transform)
    ])

    train_ds = datasets.MNIST(root="sw/datasets", train=True, download=True, transform=tfm)
    test_ds = datasets.MNIST(root="sw/datasets", train=False, download=True, transform=tfm)

    train_ld = DataLoader(train_ds, batch_size=256, shuffle=True, num_workers=0)
    test_ld = DataLoader(test_ds, batch_size=512, shuffle=False, num_workers=0)

    model = nn.Linear(784, 10, bias=True)
    opt = torch.optim.SGD(model.parameters(), lr=0.1, momentum=0.9, weight_decay=1e-4)
    loss_fn = nn.CrossEntropyLoss()

    for epoch in range(1, 6):
        model.train()
        for x, y in train_ld:
            opt.zero_grad(set_to_none=True)
            logits = model(x)
            loss = loss_fn(logits, y)
            loss.backward()
            opt.step()

        model.eval()
        correct = 0
        total = 0
        with torch.no_grad():
            for x, y in test_ld:
                pred = model(x).argmax(dim=1)
                correct += int((pred == y).sum().item())
                total += int(y.numel())
        print(f"epoch={epoch} test_acc={correct / total:.4f}")

    W = model.weight.detach().cpu().numpy().astype(np.float64)  # (10,784)
    b = model.bias.detach().cpu().numpy().astype(np.float64)  # (10,)

    os.makedirs("sw/model", exist_ok=True)
    out_path = os.path.join("sw", "model", f"mnist_fc784x10_seed{seed}.npz")
    np.savez(out_path, W=W, b=b, seed=np.int64(seed))
    print("Saved:", out_path)
    print("W float min/max:", float(W.min()), float(W.max()))
    print("b float min/max:", float(b.min()), float(b.max()))


if __name__ == "__main__":
    main()