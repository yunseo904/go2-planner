"""Rule-based discrete-skill planner for the Go2 parkour benchmark.

CPU only: nothing in this package imports Isaac Lab, Isaac Sim or torch.  The
terrain comes from the frozen archive via :mod:`terrain_toolkit`, the skill
limits from the curated-log measurements via :mod:`planner.config`.

    config.py    every parameter, tagged MEASURED / DERIVED / CALIBRATION_NEEDED
    features.py  sensor-limited geometry of the lookahead window
    skills.py    the four skills + the policy interface (dummy policies here)
    rules.py     the rule engine that picks one
"""

from .config import DEFAULT, PlannerConfig, Provenance  # noqa: F401
from .features import FeatureMemory, Observation, TerrainMap, extract, lookahead_distance, maps_from_archive  # noqa: F401
from .rules import Decision, RulePlanner, Unsupported  # noqa: F401
from .skills import Skill, SkillId, build_library, make_policy  # noqa: F401
