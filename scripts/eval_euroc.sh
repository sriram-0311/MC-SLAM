#!/usr/bin/env bash
# Evaluate one MC-SLAM EuRoC run against ground truth (evo APE + RPE, SE(3) alignment).
#
#   scripts/eval_euroc.sh <SEQ> <out_dir>
#
# Without IMU the front end has no camera-body extrinsic (Tbc is identity), so the app's body
# trajectory holds camera poses. In that case traj_body.txt is rebuilt from traj_cam0.txt with
# the calibrated Tbc. The IMU setting is read from <out_dir>/run.cfg.
set -euo pipefail

SEQ=${1:?usage: eval_euroc.sh <SEQ> <out_dir>}
OUT=${2:?usage: eval_euroc.sh <SEQ> <out_dir>}
REPO=$(cd "$(dirname "$0")/.." && pwd)
EUROC_DIR=${EUROC_DIR:-$HOME/datasets/euroc}
EVO_BIN=${EVO_BIN:-$HOME/.local/share/mcslam-venv/bin}
GT=$EUROC_DIR/$SEQ/gt_tum.txt

[[ -f $GT ]] || { echo "missing $GT" >&2; exit 1; }
if grep -q '^imu=0' "$OUT/run.cfg" && [[ -s $OUT/traj_cam0.txt ]]; then
  calib=$(sed -n 's/^calib_file_path=//p' "$OUT/run.cfg")
  "$EVO_BIN/python" "$REPO/scripts/cam_to_body_tum.py" "$calib" "$OUT/traj_cam0.txt" "$OUT/traj_body.txt"
fi
[[ -s $OUT/traj_body.txt ]] || { echo "[$SEQ] no trajectory to evaluate" >&2; exit 1; }

# Stereo (with or without IMU) observes metric scale, so align with SE(3) only (no scale correction).
"$EVO_BIN/evo_ape" tum "$GT" "$OUT/traj_body.txt" --align --t_max_diff 0.02 \
    --save_results "$OUT/ape.zip" --no_warnings > "$OUT/ape.txt" 2>&1 || true
"$EVO_BIN/evo_ape" tum "$GT" "$OUT/traj_body.txt" --align --t_max_diff 0.02 \
    --pose_relation angle_deg --no_warnings > "$OUT/ape_rot.txt" 2>&1 || true
"$EVO_BIN/evo_rpe" tum "$GT" "$OUT/traj_body.txt" --align --delta 1 --delta_unit m \
    --t_max_diff 0.02 --save_results "$OUT/rpe.zip" --no_warnings > "$OUT/rpe.txt" 2>&1 || true
rmse() { grep -E '^\s+rmse' "$1" | awk '{print $2}'; }
echo "[$SEQ] ATE $(rmse "$OUT/ape.txt") m, ATE rot $(rmse "$OUT/ape_rot.txt") deg, RPE $(rmse "$OUT/rpe.txt") m/m"
