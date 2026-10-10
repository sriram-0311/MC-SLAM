#!/usr/bin/env python3
"""Judge the loop closures MC-SLAM accepted against EuRoC ground truth.

    loop_report.py --run results/<sweep>/ [--dist 1.5 --angle 30] [SEQ ...]

Reads "LOOP_DETECTED query_t <t> match_t <t>" lines from <run>/<SEQ>/run.log. A loop is correct
if the two cam0 frames are within --dist metres and --angle degrees (ground truth), the same
definition as the place-recognition benchmark. Also reports the ATE of the run.
"""
import argparse
import os
import re
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from prset import REPO, SEQS, load_tbc


def cam_pose(gt: np.ndarray, t: float, T_bc: np.ndarray):
    i = int(np.argmin(np.abs(gt[:, 0] - t)))
    if abs(gt[i, 0] - t) > 0.02:
        return None
    T_wb = np.eye(4)
    T_wb[:3, :3] = Rotation.from_quat(gt[i, 4:8]).as_matrix()
    T_wb[:3, 3] = gt[i, 1:4]
    T_wc = T_wb @ T_bc
    return T_wc[:3, 3], T_wc[:3, 2]


def ate(run: Path) -> str:
    try:
        for line in open(run / "ape.txt"):
            if line.strip().startswith("rmse"):
                return f"{float(line.split()[1]) * 100:.1f} cm"
    except OSError:
        pass
    return "-"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", required=True)
    ap.add_argument("--euroc", default=os.path.expanduser("~/datasets/euroc"))
    ap.add_argument("--dist", type=float, default=1.5)
    ap.add_argument("--angle", type=float, default=30.0)
    ap.add_argument("seqs", nargs="*", default=SEQS)
    a = ap.parse_args()
    T_bc = load_tbc(REPO / "params/euroc/calib_euroc.yaml")
    pat = re.compile(r"LOOP_DETECTED query_t ([0-9.]+) match_t ([0-9.]+)")
    total_ok = total = 0
    print(f"{'sequence':18s} {'loops':>5s} {'correct':>7s} {'wrong':>5s}  ATE      errors of wrong loops")
    for s in a.seqs:
        run = Path(a.run) / s
        if not (run / "run.log").exists() and (Path(a.run) / "run.log").exists() and len(a.seqs) == 1:
            run = Path(a.run)  # a single-sequence output directory
        log = run / "run.log"
        if not log.exists():
            continue
        gt = np.loadtxt(Path(a.euroc) / s / "gt_tum.txt")
        ok = bad = 0
        wrong = []
        for q, m in pat.findall(log.read_text(errors="ignore")):
            pq, pm = cam_pose(gt, float(q), T_bc), cam_pose(gt, float(m), T_bc)
            if pq is None or pm is None:
                continue
            d = np.linalg.norm(pq[0] - pm[0])
            ang = np.degrees(np.arccos(np.clip(pq[1] @ pm[1], -1, 1)))
            if d < a.dist and ang < a.angle:
                ok += 1
            else:
                bad += 1
                wrong.append(f"{d:.1f}m/{ang:.0f}deg")
        total_ok += ok
        total += ok + bad
        print(f"{s:18s} {ok + bad:5d} {ok:7d} {bad:5d}  {ate(run):8s} {' '.join(wrong[:4])}")
    print(f"{'total':18s} {total:5d} {total_ok:7d} {total - total_ok:5d}")


if __name__ == "__main__":
    main()
