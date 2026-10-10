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
#   KF_ROT_DEG [5]                  keyframe rotation threshold (deg), tuning knob
#   RANSAC_PX  [0.88]               tracking RANSAC threshold (px), tuning knob
#   BACKEND_PARAMS [params/euroc/backend.yaml]  alternative back-end yaml (copied into out_dir)
#   LC_EMB_DIR []                   loop-closure descriptors <dir>/<SEQ>.bin instead of DBoW2
#   LC_EMB_ALPHA, LC_EMB_MIN [0.85, 0.5]  their relative / absolute score thresholds
#   VIEWER     [0]                  1 = show the Pangolin map viewer (needs a display)
#   IMU        [0]                  1 = stereo+IMU (needs a VI initializer; not yet working on EuRoC)
set -euo pipefail

SEQ=${1:?usage: run_euroc.sh <SEQ> [out_dir] [flags...]}
REPO=$(cd "$(dirname "$0")/.." && pwd)
OUT=${2:-$REPO/results/$(date +%Y%m%d)_$SEQ}
shift $(( $# >= 2 ? 2 : 1 ))

EUROC_DIR=${EUROC_DIR:-$HOME/datasets/euroc}
VOCAB_DIR=${VOCAB_DIR:-$HOME/datasets/vocab}
BUILD_DIR=${BUILD_DIR:-$REPO/build}
EVO_BIN=${EVO_BIN:-$HOME/.local/share/mcslam-venv/bin}
IMU=${IMU:-0}
KF_ROT_DEG=${KF_ROT_DEG:-5}
RANSAC_PX=${RANSAC_PX:-0.88}
BACKEND_PARAMS=${BACKEND_PARAMS:-}
VIEWER=${VIEWER:-0}
LC_EMB_DIR=${LC_EMB_DIR:-}
LC_EMB_ALPHA=${LC_EMB_ALPHA:-0.85}
LC_EMB_MIN=${LC_EMB_MIN:-0.5}
LC_EMB_FILE=
[[ -n $LC_EMB_DIR ]] && LC_EMB_FILE=$LC_EMB_DIR/$SEQ.bin
PARAMS=$REPO/params/euroc
SEQ_DIR=$EUROC_DIR/$SEQ

[[ -d $SEQ_DIR/mav0 ]] || { echo "missing $SEQ_DIR/mav0" >&2; exit 1; }
mkdir -p "$OUT/logs"
OUT=$(cd "$OUT" && pwd)

render() {
  sed -e "s|@SEQ_DIR@|$SEQ_DIR|g" -e "s|@OUT_DIR@|$OUT|g" \
      -e "s|@PARAMS_DIR@|$PARAMS|g" -e "s|@VOCAB_DIR@|$VOCAB_DIR|g" \
      -e "s|^imu=.*|imu=$IMU|" -e "s|@KF_ROT_DEG@|$KF_ROT_DEG|g" -e "s|@RANSAC_PX@|$RANSAC_PX|g" \
      -e "s|@LC_EMB_FILE@|$LC_EMB_FILE|g" -e "s|@LC_EMB_ALPHA@|$LC_EMB_ALPHA|g" -e "s|@LC_EMB_MIN@|$LC_EMB_MIN|g" "$1" > "$2"
}
render "$PARAMS/euroc_stereo_imu.cfg.in" "$OUT/run.cfg"
if [[ -n $BACKEND_PARAMS ]]; then
  cp "$BACKEND_PARAMS" "$OUT/backend.yaml"
  sed -i "s|^backend_params_file=.*|backend_params_file=$OUT/backend.yaml|" "$OUT/run.cfg"
fi
render "$PARAMS/frontend.yaml.in" "$OUT/frontend.yaml"

echo "[$SEQ] running -> $OUT"
start=$(date +%s.%N)
set +e
MODE=--headless
[[ $VIEWER == 1 ]] && MODE=--noheadless
(cd "$OUT" && "$BUILD_DIR/bin/mcslam_app" --config_file="$OUT/run.cfg" $MODE \
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

"$REPO/scripts/eval_euroc.sh" "$SEQ" "$OUT"
