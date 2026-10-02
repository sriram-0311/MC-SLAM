#!/usr/bin/env python3
"""Convert EuRoC state_groundtruth_estimate0/data.csv to TUM format.

EuRoC columns: timestamp[ns], px, py, pz, qw, qx, qy, qz, velocities, biases...
TUM output:    timestamp[s] tx ty tz qx qy qz qw

Usage: euroc_gt_to_tum.py <seq_dir_or_csv> [out.txt]
  If given a sequence dir (containing mav0/), writes <seq_dir>/gt_tum.txt by default.
Stdlib only.
"""
import os
import sys


def convert(csv_path, out_path):
    n = 0
    with open(csv_path) as f, open(out_path, "w") as o:
        for line in f:
            if not line.strip() or line.startswith("#"):
                continue
            c = line.strip().split(",")
            t = int(c[0]) / 1e9
            px, py, pz, qw, qx, qy, qz = c[1:8]
            o.write(f"{t:.9f} {px} {py} {pz} {qx} {qy} {qz} {qw}\n")
            n += 1
    return n


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    src = sys.argv[1]
    if os.path.isdir(src):
        csv = os.path.join(src, "mav0/state_groundtruth_estimate0/data.csv")
        out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(src, "gt_tum.txt")
    else:
        csv, out = src, sys.argv[2] if len(sys.argv) > 2 else "gt_tum.txt"
    print(f"{convert(csv, out)} poses -> {out}")


if __name__ == "__main__":
    main()
