#!/usr/bin/env bash
# Run MC-SLAM headless on one EuRoC sequence and evaluate it against ground truth.
#
#   scripts/run_euroc.sh <SEQ> [out_dir] [extra mcslam_app flags...]
#
# Environment (defaults in brackets):
#   EUROC_DIR  [~/datasets/euroc]   sequences as <EUROC_DIR>/<SEQ>/mav0
#   VOCAB_DIR  [~/datasets/vocab]   ORBvoc.txt and orb_mur.fbow
#   BUILD_DIR  [<repo>/build]
#   EVO_BIN    [~/.local/share/mcslam-venv/bin]
set -euo pipefail

SEQ=${1:?usage: run_euroc.sh <SEQ> [out_dir] [flags...]}
REPO=$(cd "$(dirname "$0")/.." && pwd)
OUT=${2:-$REPO/results/$(date +%Y%m%d)_$SEQ}
shift $(( $# >= 2 ? 2 : 1 ))

EUROC_DIR=${EUROC_DIR:-$HOME/datasets/euroc}
VOCAB_DIR=${VOCAB_DIR:-$HOME/datasets/vocab}
BUILD_DIR=${BUILD_DIR:-$REPO/build}
EVO_BIN=${EVO_BIN:-$HOME/.local/share/mcslam-venv/bin}
PARAMS=$REPO/params/euroc
SEQ_DIR=$EUROC_DIR/$SEQ

[[ -d $SEQ_DIR/mav0 ]] || { echo "missing $SEQ_DIR/mav0" >&2; exit 1; }
mkdir -p "$OUT/logs"
OUT=$(cd "$OUT" && pwd)

render() {
  sed -e "s|@SEQ_DIR@|$SEQ_DIR|g" -e "s|@OUT_DIR@|$OUT|g" \
      -e "s|@PARAMS_DIR@|$PARAMS|g" -e "s|@VOCAB_DIR@|$VOCAB_DIR|g" "$1" > "$2"
}
render "$PARAMS/euroc_stereo_imu.cfg.in" "$OUT/run.cfg"
render "$PARAMS/frontend.yaml.in" "$OUT/frontend.yaml"

echo "[$SEQ] running -> $OUT"
start=$(date +%s.%N)
set +e
(cd "$OUT" && "$BUILD_DIR/bin/mcslam_app" --config_file="$OUT/run.cfg" --headless \
    --traj_file="$OUT/traj_cam0.txt" --body_traj_file="$OUT/traj_body.txt" \
    --timing_file="$OUT/timing.csv" "$@") > "$OUT/run.log" 2>&1
status=$?
set -e
end=$(date +%s.%N)
printf '%s\n' "exit=$status wall_s=$(echo "$end - $start" | bc)" > "$OUT/status.txt"
if [[ $status -ne 0 ]]; then
  echo "[$SEQ] FAILED (exit $status); tail of $OUT/run.log:" >&2
  tail -n 15 "$OUT/run.log" >&2
  exit "$status"
fi

GT=$SEQ_DIR/gt_tum.txt
if [[ -s $OUT/traj_body.txt && -f $GT ]]; then
  # Stereo-inertial: metric scale is observable, so align with SE(3) only (no scale correction).
  "$EVO_BIN/evo_ape" tum "$GT" "$OUT/traj_body.txt" --align --t_max_diff 0.02 \
      --save_results "$OUT/ape.zip" > "$OUT/ape.txt" 2>&1 || true
  "$EVO_BIN/evo_rpe" tum "$GT" "$OUT/traj_body.txt" --align --delta 1 --delta_unit m \
      --t_max_diff 0.02 --save_results "$OUT/rpe.zip" > "$OUT/rpe.txt" 2>&1 || true
  echo "[$SEQ] ATE: $(grep -E '^\s+rmse' "$OUT/ape.txt" | awk '{print $2}') m"
fi
