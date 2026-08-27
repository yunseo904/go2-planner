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
#   GPU=0|1   which GPU to hand the container (default 1, matching the convention in
#             ~/.bashrc and ~/isaaclab30_run.sh). Both GPUs on this box are shared.
#   NAME=...  container name.
set -u
NAME=${NAME:-go2planner_isaac}
IMG=${IMG:-nvcr.io/nvidia/isaac-lab:3.0.0-beta2-post1}
GPU=${GPU:-1}

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJ=$(dirname "$HERE")
# readlink -f because ~/projects/eurekaverse-go2-parkour is a symlink to ~/ev-go2 and a
# bind-mount source has to be the real path.
UPSTREAM=$(readlink -f "$PROJ/../eurekaverse-go2-parkour")

IHOME=$HOME/isaaclab_cache/home
mkdir -p "$IHOME/.cache/ov" "$IHOME/.nv" "$IHOME/.local/share/ov" "$HOME/isaaclab_cache/kit"
docker rm -f "$NAME" >/dev/null 2>&1 || true

# CUBLAS_WORKSPACE_CONFIG: with Kit holding the CUDA context, PyTorch segfaults grabbing a
# cuBLAS workspace dynamically. Confirmed by minimal repro; this variable is the fix.
exec docker run --rm --name "$NAME" --gpus "\"device=$GPU\"" \
  --entrypoint bash --user "$(id -u):$(id -g)" \
  ${DOCKER_EXTRA:-} \
  -e HOME=/ihome -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e USER="$(id -un)" -e LOGNAME="$(id -un)" \
  -e TORCHINDUCTOR_CACHE_DIR=/ihome/.cache/torchinductor \
  -e CUBLAS_WORKSPACE_CONFIG=:4096:8 \
  -e OMNI_KIT_ACCEPT_EULA=YES -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y \
  -e QUIET="${QUIET:-0}" \
  -v "$PROJ":/workspace/go2-planner \
  -v "$UPSTREAM":/workspace/eurekaverse-go2-parkour:ro \
  -v "$HERE/_isaac_docker_inside.sh":/opt/inside.sh:ro \
  -v "$IHOME":/ihome \
  -v "$HOME/isaaclab_cache/kit":/isaac-sim/kit/cache \
  "$IMG" /opt/inside.sh "$@"
