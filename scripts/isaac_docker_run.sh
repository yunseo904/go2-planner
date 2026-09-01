#!/usr/bin/env bash
# Run one go2-planner script inside the Isaac Lab 3.0 container.
#
#     scripts/isaac_docker_run.sh scripts/probe_isaac_actuator.py --headless
#
# Isaac Lab is not installed natively on this machine; it exists only as the NGC image
# below. The five pieces of wiring here (entrypoint, uid, HOME, USER/LOGNAME, pysite
# PYTHONPATH) are the same ones ~/isaaclab30_run.sh needs and were each found the hard
# way -- see that script's comments.
#
#   GPU=none|0|1  which GPU to hand the container.  DEFAULT none, and that is not a
#             fallback -- it is what these runs need.  Measured 2026-08-29: TROT
#             --foot-comp raibert --foot-clip-rad 0.05, 60 cycles, run with no
#             /dev/nvidia* in the container at all, reproduces the GPU run BIT FOR BIT
#             on every column of the results row (stride, duty, vx, vy, yaw rate, base
#             height, 1920 steps, the deviation table, stride CV) in the same 36 s of
#             wall clock.  Physics is on the CPU (--device cpu) and headless Kit never
#             renders, so the card was doing nothing.  A whole sweep was run on a
#             borrowed GPU before anyone checked.
#
#             GPU=1 is needed for exactly two things, both of which RENDER:
#               --enable_cameras / --video   (side-view recording)
#               LIVE=1                       (WebRTC livestream)
#             Without a GPU those do not fail fast: the run hangs emitting
#             `carb.cudainterop ... cudaErrorInsufficientDriver` forever.  Measured the
#             same day.
#
#             GPU 1 carries someone else's training run and must be stopped to borrow it
#             (~/WMP_중단_재개.md), so ASK FIRST.  GPU 0 is not ours at any time.
#   NAME=...  container name.
#   LIVE=1    put the container on the HOST network, for Isaac Sim's WebRTC livestream.
#             Publishing -p 49100/-p 47998 is not enough: WebRTC negotiates a media path
#             by advertising the addresses it can be reached at (ICE candidates), and
#             inside a bridge network the only address it knows about is its own
#             172.17.x.x, which no client outside the box can route to. On the host
#             network it advertises z4's real address and the client connects.
#             Signalling is TCP 49100, media is UDP 47998 -- see the streaming section
#             of README.md for what has to be reachable and what to do when it is not.
set -u
NAME=${NAME:-go2planner_isaac}
IMG=${IMG:-nvcr.io/nvidia/isaac-lab:3.0.0-beta2-post1}
GPU=${GPU:-none}

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJ=$(dirname "$HERE")
# readlink -f because ~/projects/eurekaverse-go2-parkour is a symlink to ~/ev-go2 and a
# bind-mount source has to be the real path.
UPSTREAM=$(readlink -f "$PROJ/../eurekaverse-go2-parkour")
# legged_eval defines the benchmark (terrain, seeding, episode rules, aggregation) and is
# read-only: someone else's package. Mounted :ro so that is enforced, not just intended.
LEGGED_EVAL=$(readlink -f "${LEGGED_EVAL_ROOT:-$HOME/legged_eval}")
[ -f "$LEGGED_EVAL/legged_eval/terrain.py" ] || {
  echo "[docker] legged_eval not found at $LEGGED_EVAL -- it is the benchmark's" >&2
  echo "[docker]   definition, not an optional dependency. Set LEGGED_EVAL_ROOT." >&2
  exit 3
}

IHOME=$HOME/isaaclab_cache/home
mkdir -p "$IHOME/.cache/ov" "$IHOME/.nv" "$IHOME/.local/share/ov" "$HOME/isaaclab_cache/kit"
docker rm -f "$NAME" >/dev/null 2>&1 || true

# CUBLAS_WORKSPACE_CONFIG: with Kit holding the CUDA context, PyTorch segfaults grabbing a
# cuBLAS workspace dynamically. Confirmed by minimal repro; this variable is the fix.
# Host networking is opt-in: it removes the container's network isolation, so it is
# not something to leave on for a headless batch run.
NET=""
if [ "${LIVE:-0}" = "1" ]; then
  NET="--network host"
  echo "[docker] LIVE=1: --network host, WebRTC signalling tcp/49100, media udp/47998"
fi

# `none` hands the container no GPU at all, which is the default -- see the header.
GPUARG="--gpus \"device=$GPU\""
if [ "$GPU" = "none" ]; then
  GPUARG=""
  # The rendering paths do not fail fast without a card, they hang; say so here rather
  # than let a run sit in a CUDA error loop until someone notices.
  case " $* " in
    *" --enable_cameras "*|*" --video "*)
      echo "[docker] WARNING --enable_cameras/--video RENDER and need a GPU. With GPU=none" >&2
      echo "[docker]         this will hang in carb.cudainterop instead of failing. Borrow" >&2
      echo "[docker]         GPU 1 (ask first, ~/WMP_중단_재개.md) and re-run with GPU=1." >&2;;
  esac
  [ "${LIVE:-0}" = "1" ] && echo "[docker] WARNING LIVE=1 needs a GPU; see above." >&2
fi

exec docker run --rm --name "$NAME" ${GPUARG} \
  --entrypoint bash --user "$(id -u):$(id -g)" \
  ${NET} \
  ${DOCKER_EXTRA:-} \
  -e HOME=/ihome -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e USER="$(id -un)" -e LOGNAME="$(id -un)" \
  -e TORCHINDUCTOR_CACHE_DIR=/ihome/.cache/torchinductor \
  -e CUBLAS_WORKSPACE_CONFIG=:4096:8 \
  -e OMNI_KIT_ACCEPT_EULA=YES -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
  -e QUIET="${QUIET:-0}" \
  -v "$PROJ":/workspace/go2-planner \
  -v "$UPSTREAM":/workspace/eurekaverse-go2-parkour:ro \
  -v "$LEGGED_EVAL":/opt/legged_eval:ro \
  -v "$HERE/_isaac_docker_inside.sh":/opt/inside.sh:ro \
  -v "$IHOME":/ihome \
  -v "$HOME/isaaclab_cache/kit":/isaac-sim/kit/cache \
  "$IMG" /opt/inside.sh "$@"
