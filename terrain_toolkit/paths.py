"""Path resolution for the project. No hard-coded absolute paths.

* ``PROJECT_ROOT``  – this repository (``go2-planner``).
* ``UPSTREAM_ROOT`` – the read-only ``eurekaverse-go2-parkour`` checkout.
  Resolved from ``$EUREKAVERSE_ROOT`` if set, otherwise assumed to be a
  sibling directory of this repository.
"""

from __future__ import annotations

import os
from pathlib import Path

PROJECT_ROOT: Path = Path(__file__).resolve().parents[1]
DATA_DIR: Path = PROJECT_ROOT / "data"
OUTPUTS_DIR: Path = PROJECT_ROOT / "outputs"

_UPSTREAM_DIRNAME = "eurekaverse-go2-parkour"


def _resolve_upstream_root() -> Path:
    env = os.environ.get("EUREKAVERSE_ROOT")
    if env:
        return Path(env).expanduser().resolve()
    return (PROJECT_ROOT.parent / _UPSTREAM_DIRNAME).resolve()


UPSTREAM_ROOT: Path = _resolve_upstream_root()

# Relative locations inside the upstream repo.
UPSTREAM_LEGGED_GYM_UTILS = Path("extreme-parkour") / "legged_gym" / "legged_gym" / "utils"
BENCHMARK_MODULE_PATH: Path = UPSTREAM_ROOT / UPSTREAM_LEGGED_GYM_UTILS / "set_terrain_benchmark.py"
UPSTREAM_TERRAIN_UTILS_PATH: Path = UPSTREAM_ROOT / UPSTREAM_LEGGED_GYM_UTILS / "terrain_utils.py"
UPSTREAM_TERRAIN_GPT_PATH: Path = UPSTREAM_ROOT / UPSTREAM_LEGGED_GYM_UTILS / "terrain_gpt.py"
UPSTREAM_BENCHMARK_CFG_PATH: Path = UPSTREAM_ROOT / "eurekaverse" / "config" / "config.yaml"

# Frozen artefacts.
FROZEN_NPZ: Path = DATA_DIR / "benchmark_frozen.npz"
FROZEN_SHA256: Path = DATA_DIR / "benchmark_frozen.sha256"
FROZEN_META_JSON: Path = DATA_DIR / "benchmark_frozen.meta.json"
UPSTREAM_COMMIT_TXT: Path = DATA_DIR / "upstream_commit.txt"

# Profiling outputs.
PROFILE_CSV: Path = OUTPUTS_DIR / "terrain_profile.csv"
TASK_SUMMARY_CSV: Path = OUTPUTS_DIR / "task_summary.csv"
TERRAIN_RENDER_DIR: Path = OUTPUTS_DIR / "terrains"


def require_upstream() -> Path:
    """Return ``UPSTREAM_ROOT`` or raise a helpful error if it is missing."""
    if not BENCHMARK_MODULE_PATH.is_file():
        raise FileNotFoundError(
            f"Upstream benchmark module not found at {BENCHMARK_MODULE_PATH}.\n"
            f"Set $EUREKAVERSE_ROOT to the eurekaverse-go2-parkour checkout "
            f"(currently resolved to {UPSTREAM_ROOT})."
        )
    return UPSTREAM_ROOT
