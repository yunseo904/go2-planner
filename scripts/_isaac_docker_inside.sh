#!/bin/bash
# Runs INSIDE the Isaac Lab 3.0 container, for go2-planner scripts.
#
# Sibling of ~/isaaclab30_inside.sh, which serves the eurekaverse repo instead. Kept
# separate rather than parameterised: that script belongs to the distillation workflow
# someone else is running, and CLAUDE.md 1 puts every file we create inside go2-planner.
#
# Layout inside the container mirrors the host so terrain_toolkit/paths.py resolves
# UPSTREAM_ROOT by its normal sibling rule and needs no $EUREKAVERSE_ROOT override:
#     /workspace/go2-planner                (rw)  <- ~/projects/go2-planner
#     /workspace/eurekaverse-go2-parkour    (ro)  <- ~/ev-go2
# The upstream mount is :ro so the read-only rule is enforced by the kernel rather than
# by discipline. The host tree is deliberately NOT chmod-locked (CLAUDE.md 1).
set -eu
REPO=/workspace/go2-planner
PY="${ISAACLAB_PATH:-/workspace/isaaclab}/isaaclab.sh -p"
# /ihome/pysite carries the deps the image lacks. scipy is pinned to 1.13.1 there and
# numpy was deliberately removed; PYTHONPATH outranks site-packages, so a numpy in
# pysite would shadow the one this image's torch and isaacsim were built against.
export PYTHONPATH="$REPO:/ihome/pysite${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONPYCACHEPREFIX=/tmp/pycache

if [ "${QUIET:-0}" != "1" ]; then
  echo "[inside] uid=$(id -u) HOME=$HOME cwd=$REPO"
fi
cd "$REPO"
exec $PY "$@"
