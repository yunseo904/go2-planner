#!/usr/bin/env bash
# Stage 2 sweep: how far does the recording have to be overwritten before the gait
# holds?
#
#     scripts/run_foot_comp_sweep.sh A     # cap ladder on TROT + the WALK control
#     scripts/run_foot_comp_sweep.sh B     # sign control, k, and the WALK re-check
#
# One container per point, because the parameter is fixed before the sim starts and
# a crash in one point then costs one point.  Every run is FLAT GROUND, 60 cycles,
# and writes one row to outputs/foot_comp.csv plus its per-joint departure table to
# outputs/foot_comp_dev.csv.  Nothing here reads the benchmark, the terrain or the
# depth camera; the only knob that moves is how much of the clip may be overwritten.
set -u
cd "$(dirname "$0")/.."
OUT=outputs/foot_comp.csv
DEV=outputs/foot_comp_dev.csv
TR=outputs/traces/footcomp
LOG=logs/foot_comp.log
mkdir -p "$TR" logs

run() {   # run <tag> <clip> [extra args...]
    local tag=$1 clip=$2; shift 2
    echo "########### $(date +%T) $tag ($clip) $*" | tee -a "$LOG"
    QUIET=1 timeout 1800 scripts/isaac_docker_run.sh scripts/verify_skill_replay.py \
        --clip-archive data/skill_clips.npz --clip "$clip" --rate lo \
        --headless --device cpu --hip-sign keep --contact-threshold-n 30 \
        --start-phase level --settle-mode stand --cycles 60 \
        --results-csv "$OUT" --dev-csv "$DEV" --tag "$tag" \
        --trace-npz "$TR/$tag.npz" "$@" >>"$LOG" 2>&1
    local rc=$?
    grep -E "terminated at|overwritten:|verdict:|reproduced|Traceback|SystemExit|Error" "$LOG" \
        | tail -6
    echo "    exit $rc"
}

case "${1:-A}" in
A)
    # The null control first: --foot-clip-rad 0 may not differ from --foot-comp off
    # by a single step, or the comparison the whole ladder rests on is not one.
    run trot_off      TROT --foot-comp off
    run trot_cap000   TROT --foot-comp raibert --foot-clip-rad 0
    run trot_cap002   TROT --foot-comp raibert --foot-clip-rad 0.02
    run trot_cap005   TROT --foot-comp raibert --foot-clip-rad 0.05
    run trot_cap010   TROT --foot-comp raibert --foot-clip-rad 0.10
    run trot_cap020   TROT --foot-comp raibert --foot-clip-rad 0.20
    run trot_cap040   TROT --foot-comp raibert --foot-clip-rad 0.40
    run walk_off      WALK --foot-comp off
    ;;
run)
    # One point, chosen after reading stage A: scripts/run_foot_comp_sweep.sh run
    # <tag> <clip> [args...]
    shift
    run "$@"
    ;;
*)
    echo "usage: $0 A | $0 run <tag> <clip> [args...]" >&2; exit 2;;
esac
