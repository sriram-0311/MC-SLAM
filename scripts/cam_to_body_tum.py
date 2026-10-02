#!/usr/bin/env python3
"""Convert a TUM trajectory of camera poses T_wc into body (IMU) poses T_wb = T_wc * Tbc^-1.

Tbc (cam0 -> body) is read from the `imu: Tbc:` block of an MC-SLAM calibration
file (e.g. params/euroc/calib_euroc.yaml). Needed to compare stereo-only runs
against EuRoC ground truth, which is expressed in the body frame.

Usage: cam_to_body_tum.py <calib.yaml> <traj_cam.txt> <traj_body.txt>
"""
import sys

import numpy as np
import yaml
from scipy.spatial.transform import Rotation


def load_tbc(path: str) -> np.ndarray:
    text = "\n".join(l for l in open(path).read().splitlines() if not l.startswith("%YAML"))
    return np.array(yaml.safe_load(text)["imu"]["Tbc"], dtype=float)


def main() -> int:
    if len(sys.argv) != 4:
        print(__doc__)
        return 1
    T_cb = np.linalg.inv(load_tbc(sys.argv[1]))
    with open(sys.argv[3], "w") as out:
        for line in open(sys.argv[2]):
            if not line.strip() or line.startswith("#"):
                continue
            t, x, y, z, qx, qy, qz, qw = map(float, line.split())
            T_wc = np.eye(4)
            T_wc[:3, :3] = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
            T_wc[:3, 3] = (x, y, z)
            T_wb = T_wc @ T_cb
            q = Rotation.from_matrix(T_wb[:3, :3]).as_quat()
            out.write(f"{t:.9f} " + " ".join(f"{v:.9f}" for v in (*T_wb[:3, 3], *q)) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
