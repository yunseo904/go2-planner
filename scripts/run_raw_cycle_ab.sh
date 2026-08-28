#!/usr/bin/env bash
# A/B the EXTRACTION method: median-over-cycles vs each raw cycle on its own.
#
#     scripts/run_raw_cycle_ab.sh TROT
#
# Needs data/raw_cycles_<CLIP>.npz from scripts/extract_raw_cycles.py, which needs
# the curated logs.  Every run uses the settings day 2 settled on and differs from
# every other run in one thing only: which cycles the clip was built from.
set -u
CLIP=${1:-TROT}
cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.."

ARCHIVE=data/raw_cycles_${CLIP}.npz
[ -f "$ARCHIVE" ] || { echo "no $ARCHIVE -- run scripts/extract_raw_cycles.py --clip $CLIP first"; exit 1; }

OUT=outputs/raw_cycle_ab/$CLIP
mkdir -p "$OUT" outputs/traces
NAMES=$(.venv/bin/python -c "
import numpy as np; print(' '.join(str(x) for x in np.load('$ARCHIVE')['clip_names']))")

common="--headless --device cpu --hip-sign keep --contact-threshold-n 30 --start-phase level --settle-mode stand"
for name in $NAMES; do
  echo "########## $name  $(date -Is)"
  QUIET=1 scripts/isaac_docker_run.sh scripts/verify_skill_replay.py \
      --clip-archive "$ARCHIVE" --clip "$name" --cycles 40 $common \
      --results-csv "$OUT/$name.csv" \
      --trace-npz "outputs/traces/${CLIP}_ab_${name}.npz" 2>&1
  echo "########## $name exit=$? $(date -Is)"
done
echo "ALL DONE $(date -Is)"
echo "now: scripts/analyze_drift.py --ab $OUT"
