#!/usr/bin/env python3
"""Precompute a global descriptor for every cam0 frame of EuRoC sequences, for MC-SLAM's loop closer.

    embed_sequence.py --model ijepa_h14_1k --layer 28 --out emb_dir/ [--euroc ~/datasets/euroc] SEQ ...

Writes <out>/<SEQ>.bin, read by mcslam::EmbeddingTable:
    int32 N, int32 D, then N records of (float64 timestamp [s], float32[D] L2-normalised descriptor).
The descriptor is the mean of the patch tokens after transformer block --layer (as in embed.py
mean_L<k>); blocks after it are dropped, so inference stops there.
"""
import argparse
import os
import struct
import time
from pathlib import Path

import numpy as np
import torch

from embed import MODELS, load, preprocess


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", required=True, choices=sorted(MODELS))
    ap.add_argument("--layer", type=int, required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--euroc", default=os.path.expanduser("~/datasets/euroc"))
    ap.add_argument("--height", type=int, default=224)
    ap.add_argument("--batch", type=int, default=32)
    ap.add_argument("seqs", nargs="+")
    a = ap.parse_args()
    model, family, patch = load(a.model)
    assert family in ("ijepa", "dinov2"), "per-frame export supports I-JEPA and DINOv2"
    # stop after block --layer (block list location differs between transformers versions)
    if hasattr(model, "layers"):
        model.layers = model.layers[:a.layer]
    else:
        model.encoder.layer = model.encoder.layer[:a.layer]
    first = 1 if family == "dinov2" else 0
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    for seq in a.seqs:
        ns = np.loadtxt(Path(a.euroc) / seq / "mav0/cam0/data.csv", delimiter=",", comments="#",
                        dtype=np.int64, usecols=0)
        paths = [str(Path(a.euroc) / seq / "mav0/cam0/data" / f"{n}.png") for n in ns]
        feats, t0, n_img = [], time.time(), 0
        for i in range(0, len(paths), a.batch):
            x = torch.stack([preprocess(p, a.height, patch) for p in paths[i:i + a.batch]]).cuda().half()
            with torch.no_grad():
                kwargs = {"interpolate_pos_encoding": True} if family == "ijepa" else {}
                h = model(pixel_values=x, output_hidden_states=True, **kwargs).hidden_states[a.layer]
            f = torch.nn.functional.normalize(h[:, first:].mean(1).float(), dim=1)
            feats.append(f.cpu().numpy().astype(np.float32))
            n_img += len(x)
        torch.cuda.synchronize()
        feats = np.concatenate(feats)
        with open(out / f"{seq}.bin", "wb") as fh:
            fh.write(struct.pack("<ii", len(ns), feats.shape[1]))
            for n, f in zip(ns, feats):
                fh.write(struct.pack("<d", n * 1e-9))
                fh.write(f.tobytes())
        dt = time.time() - t0
        print(f"{seq}: {len(ns)} frames x {feats.shape[1]}, {dt:.0f} s incl. image loading "
              f"({1000 * dt / n_img:.1f} ms/frame)")


if __name__ == "__main__":
    main()
