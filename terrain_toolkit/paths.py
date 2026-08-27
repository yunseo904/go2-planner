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
_CURATED_DIRNAME = "curated"


def _resolve_upstream_root() -> Path:
    env = os.environ.get("EUREKAVERSE_ROOT")
    if env:
        return Path(env).expanduser().resolve()
    return (PROJECT_ROOT.parent / _UPSTREAM_DIRNAME).resolve()


UPSTREAM_ROOT: Path = _resolve_upstream_root()


def _resolve_curated_root() -> Path:
    env = os.environ.get("GO2_CURATED_ROOT")
    if env:
        return Path(env).expanduser().resolve()
    return (PROJECT_ROOT.parent / _CURATED_DIRNAME).resolve()


#: Read-only curated Go2 log set (36 sessions, ``INDEX.md`` + ``<group>/<session>/``).
CURATED_ROOT: Path = _resolve_curated_root()

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

# Calibration probes (synthetic; no upstream code involved).
CALIBRATION_NPZ: Path = DATA_DIR / "calibration_probes.npz"
CALIBRATION_SHA256: Path = DATA_DIR / "calibration_probes.sha256"
CALIBRATION_META_JSON: Path = DATA_DIR / "calibration_probes.meta.json"
CALIBRATION_PLAN_MD: Path = OUTPUTS_DIR / "calibration_plan.md"

# Replayable skill clips cut from the curated logs (no upstream code involved).
SKILL_CLIPS_NPZ: Path = DATA_DIR / "skill_clips.npz"
SKILL_CLIPS_SHA256: Path = DATA_DIR / "skill_clips.sha256"
SKILL_CLIPS_META_JSON: Path = DATA_DIR / "skill_clips.meta.json"
SKILL_CLIPS_MD: Path = OUTPUTS_DIR / "skill_clips.md"

GAIN_FEASIBILITY_MD: Path = OUTPUTS_DIR / "gain_feasibility.md"
SERVER_DAY1_MD: Path = OUTPUTS_DIR / "server_day1.md"
ISAAC_PROBE_JSON: Path = OUTPUTS_DIR / "isaac_actuator_probe.json"

# Calibration run results (produced on the sim machine by scripts/run_calibration.py).
CALIBRATION_RESULTS_CSV: Path = OUTPUTS_DIR / "calibration_results.csv"
CALIBRATION_REPORT_MD: Path = OUTPUTS_DIR / "calibration_report.md"

# Replay verification results (produced on the sim machine by scripts/verify_skill_replay.py).
REPLAY_RESULTS_CSV: Path = OUTPUTS_DIR / "replay_verify.csv"
REPLAY_REPORT_MD: Path = OUTPUTS_DIR / "replay_verify.md"

# Profiling outputs.
PROFILE_CSV: Path = OUTPUTS_DIR / "terrain_profile.csv"
TASK_SUMMARY_CSV: Path = OUTPUTS_DIR / "task_summary.csv"
TERRAIN_RENDER_DIR: Path = OUTPUTS_DIR / "terrains"

# Skill profiling outputs (curated log set).
SKILL_PROFILE_CSV: Path = OUTPUTS_DIR / "skill_profile.csv"
SKILL_PROFILE_MD: Path = OUTPUTS_DIR / "skill_profile.md"
SKILL_TRANSITION_MD: Path = OUTPUTS_DIR / "skill_transition.md"
JUMP_PROFILE_CSV: Path = OUTPUTS_DIR / "jump_profile.csv"

# Offline planner sweep outputs.
PLANNER_SUMMARY_CSV: Path = OUTPUTS_DIR / "planner_offline_summary.csv"
PLANNER_SEGMENTS_CSV: Path = OUTPUTS_DIR / "planner_offline_segments.csv"
PLANNER_UNSUPPORTED_CSV: Path = OUTPUTS_DIR / "planner_offline_unsupported.csv"
PLANNER_OFFLINE_MD: Path = OUTPUTS_DIR / "planner_offline.md"


def require_upstream() -> Path:
    """Return ``UPSTREAM_ROOT`` or raise a helpful error if it is missing."""
    if not BENCHMARK_MODULE_PATH.is_file():
        raise FileNotFoundError(
            f"Upstream benchmark module not found at {BENCHMARK_MODULE_PATH}.\n"
            f"Set $EUREKAVERSE_ROOT to the eurekaverse-go2-parkour checkout "
            f"(currently resolved to {UPSTREAM_ROOT})."
        )
    return UPSTREAM_ROOT


def require_curated() -> Path:
    """Return ``CURATED_ROOT`` or raise a helpful error if it is missing."""
    if not (CURATED_ROOT / "INDEX.md").is_file():
        raise FileNotFoundError(
            f"Curated log set not found at {CURATED_ROOT}.\n"
            f"Set $GO2_CURATED_ROOT to the directory holding INDEX.md."
        )
    return CURATED_ROOT
