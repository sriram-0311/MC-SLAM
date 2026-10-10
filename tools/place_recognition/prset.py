#!/usr/bin/env python3
"""Build a place-recognition benchmark set from EuRoC.

Frames of cam0 are sampled uniformly in time (default 2 Hz) inside the span covered by the
ground truth. For every frame the ground-truth camera position and optical axis are stored, so
that "same place" can later be defined geometrically.

    prset.py --out prset/ [--euroc ~/datasets/euroc] [--rate 2.0] [SEQ ...]

Writes prset/<SEQ>.npz with: t [s], images (paths), pos (N,3) camera centres, fwd (N,3) unit
optical axes, all in the ground-truth world frame of that sequence.
"""
import argparse
import os
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

SEQS = ["MH_01_easy", "MH_02_easy", "MH_03_medium", "MH_04_difficult", "MH_05_difficult",
        "V1_01_easy", "V1_02_medium", "V1_03_difficult", "V2_01_easy", "V2_02_medium",
        "V2_03_difficult"]
REPO = Path(__file__).resolve().parents[2]


def load_tbc(calib: Path) -> np.ndarray:
    text = "\n".join(l for l in calib.read_text().splitlines() if not l.startswith("%YAML"))
    return np.array(yaml.safe_load(text)["imu"]["Tbc"], dtype=float)  # cam0 -> body


def build(seq_dir: Path, rate: float, T_bc: np.ndarray):
    gt = np.loadtxt(seq_dir / "gt_tum.txt")
    ns = np.loadtxt(seq_dir / "mav0/cam0/data.csv", delimiter=",", comments="#", dtype=np.int64,
                    usecols=0)
    ns = ns[(ns * 1e-9 >= gt[0, 0]) & (ns * 1e-9 <= gt[-1, 0])]
    keep, next_t = [], -np.inf
    for n in ns:
        if n * 1e-9 >= next_t:
            keep.append(n)
            next_t = n * 1e-9 + 1.0 / rate
    picked_ns = np.array(keep, dtype=np.int64)
    picked = picked_ns * 1e-9
    idx = np.clip(np.searchsorted(gt[:, 0], picked), 1, len(gt) - 1)
    idx = np.where(np.abs(gt[idx - 1, 0] - picked) < np.abs(gt[idx, 0] - picked), idx - 1, idx)
    ok = np.abs(gt[idx, 0] - picked) < 0.01
    picked, picked_ns, idx = picked[ok], picked_ns[ok], idx[ok]
    pos, fwd = [], []
    for g in gt[idx]:
        T_wb = np.eye(4)
        T_wb[:3, :3] = Rotation.from_quat(g[4:8]).as_matrix()
        T_wb[:3, 3] = g[1:4]
        T_wc = T_wb @ T_bc
        pos.append(T_wc[:3, 3])
        fwd.append(T_wc[:3, 2])
    images = [str(seq_dir / "mav0/cam0/data" / f"{n}.png") for n in picked_ns]
    return picked, np.array(images), np.array(pos), np.array(fwd)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--euroc", default=os.path.expanduser("~/datasets/euroc"))
    ap.add_argument("--rate", type=float, default=2.0)
    ap.add_argument("--calib", default=str(REPO / "params/euroc/calib_euroc.yaml"))
    ap.add_argument("seqs", nargs="*", default=SEQS)
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    T_bc = load_tbc(Path(a.calib))
    for s in a.seqs:
        t, images, pos, fwd = build(Path(a.euroc) / s, a.rate, T_bc)
        assert all(os.path.exists(p) for p in images), f"missing images in {s}"
        np.savez(out / f"{s}.npz", t=t, images=images, pos=pos, fwd=fwd)
        print(f"{s}: {len(t)} frames over {t[-1] - t[0]:.0f} s")


if __name__ == "__main__":
    main()
