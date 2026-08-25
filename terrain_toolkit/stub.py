"""Minimal numpy-only stand-in for the upstream ``SubTerrain`` object.

Upstream reference (``legged_gym/utils/terrain_gpt.py``)::

    class SubTerrain:
        def __init__(self, terrain_name="terrain", width=256, length=256,
                     vertical_scale=1.0, horizontal_scale=1.0):
            self.terrain_name = terrain_name
            self.vertical_scale = vertical_scale
            self.horizontal_scale = horizontal_scale
            self.width = width
            self.length = length
            self.height_field_raw = np.zeros((self.width, self.length), dtype=np.int16)

and in ``Terrain.make_terrain``::

    set_seed(int(variation * 1e3 + difficulty * 1e6))
    # NOTE: Width and length are swapped in the terrain_utils.SubTerrain, careful!
    terrain = SubTerrain("terrain",
                         width=self.length_per_env_pixels,   # x cells (forward)
                         length=self.width_per_env_pixels,   # y cells (lateral)
                         vertical_scale=cfg.vertical_scale,
                         horizontal_scale=cfg.horizontal_scale)
    terrain.goals = np.zeros((cfg.num_goals, 2))
    set_idx = set_terrain_benchmark(terrain, variation, difficulty)

Attributes actually touched by ``set_terrain_benchmark.set_terrain`` and by
``terrain_utils.fix_terrain``:

    width, length, horizontal_scale, vertical_scale, height_field_raw, goals

``set_terrain`` fills ``height_field_raw`` (int16, units of ``vertical_scale``)
and ``goals`` (float64, metres) in place and returns the task index.
"""

from __future__ import annotations

import importlib.util
import random
import sys
from dataclasses import dataclass, field
from types import ModuleType
from typing import List, Optional

import numpy as np

from .paths import BENCHMARK_MODULE_PATH, require_upstream


@dataclass(frozen=True)
class BenchmarkTerrainCfg:
    """Benchmark grid parameters.

    Values mirror ``eurekaverse/config/config.yaml::terrain_benchmark`` and
    ``LeggedRobotCfg.terrain`` (``legged_robot_config.py``) at the frozen
    upstream commit. They are duplicated here on purpose so this package has
    no dependency on the upstream python package.
    """

    terrain_length: float = 18.0   # [m] along x (forward / travel direction)
    terrain_width: float = 4.0     # [m] along y (lateral)
    horizontal_scale: float = 0.05  # [m] per cell
    vertical_scale: float = 0.005   # [m] per height unit
    num_goals: int = 8
    num_rows: int = 10   # difficulty levels
    num_cols: int = 20   # terrain types (variations)
    # Robot spawn: ``env_origin_x = i * env_length + 1.0``, ``env_origin_y = (j + 0.5) * env_width``
    spawn_x: float = 1.0
    spawn_y: float = field(default=2.0)

    @property
    def x_cells(self) -> int:
        """``length_per_env_pixels`` upstream (passed as SubTerrain.width)."""
        return int(self.terrain_length / self.horizontal_scale)

    @property
    def y_cells(self) -> int:
        """``width_per_env_pixels`` upstream (passed as SubTerrain.length)."""
        return int(self.terrain_width / self.horizontal_scale)

    def difficulty(self, row: int) -> float:
        """``difficulty = i / (num_rows - 1)`` (0.5 when num_rows == 1)."""
        return row / (self.num_rows - 1) if self.num_rows > 1 else 0.5

    def variation(self, col: int) -> float:
        """``variation = j / num_cols``."""
        return col / self.num_cols

    def seed(self, variation: float, difficulty: float) -> int:
        """Replicates ``set_seed(int(variation * 1e3 + difficulty * 1e6))``."""
        return int(variation * 1e3 + difficulty * 1e6)


class SubTerrainStub:
    """Duck-typed replacement for ``terrain_gpt.SubTerrain``."""

    __slots__ = ("terrain_name", "vertical_scale", "horizontal_scale", "width", "length",
                 "height_field_raw", "goals", "idx")

    def __init__(self, terrain_name: str = "terrain", width: int = 256, length: int = 256,
                 vertical_scale: float = 1.0, horizontal_scale: float = 1.0, num_goals: int = 8):
        self.terrain_name = terrain_name
        self.vertical_scale = float(vertical_scale)
        self.horizontal_scale = float(horizontal_scale)
        self.width = int(width)    # x cells (upstream naming, swapped!)
        self.length = int(length)  # y cells
        self.height_field_raw = np.zeros((self.width, self.length), dtype=np.int16)
        self.goals = np.zeros((num_goals, 2), dtype=np.float64)
        self.idx: Optional[int] = None

    # Convenience -----------------------------------------------------------
    @property
    def env_length_m(self) -> float:
        return self.width * self.horizontal_scale

    @property
    def env_width_m(self) -> float:
        return self.length * self.horizontal_scale

    def height_field_m(self) -> np.ndarray:
        return self.height_field_raw.astype(np.float64) * self.vertical_scale

    def copy(self) -> "SubTerrainStub":
        t = SubTerrainStub(self.terrain_name, self.width, self.length, self.vertical_scale,
                           self.horizontal_scale, num_goals=self.goals.shape[0])
        t.height_field_raw = self.height_field_raw.copy()
        t.goals = self.goals.copy()
        t.idx = self.idx
        return t

    def __repr__(self) -> str:
        return (f"SubTerrainStub(cells=({self.width}, {self.length}), hs={self.horizontal_scale}, "
                f"vs={self.vertical_scale}, goals={self.goals.shape}, idx={self.idx})")


def make_stub(cfg: BenchmarkTerrainCfg = BenchmarkTerrainCfg()) -> SubTerrainStub:
    """Create an empty stub exactly like ``Terrain.make_terrain`` does."""
    return SubTerrainStub(
        "terrain",
        width=cfg.x_cells,
        length=cfg.y_cells,
        vertical_scale=cfg.vertical_scale,
        horizontal_scale=cfg.horizontal_scale,
        num_goals=cfg.num_goals,
    )


def set_seed(seed: int) -> None:
    """numpy/random subset of ``legged_gym.utils.helpers.set_seed`` (torch omitted).

    The benchmark terrain functions only draw from ``np.random``.
    """
    if seed == -1:
        seed = np.random.randint(0, 10000)
    random.seed(seed)
    np.random.seed(seed)


_MODULE_CACHE: dict = {}


def load_benchmark_module(path=None) -> ModuleType:
    """Import upstream ``set_terrain_benchmark.py`` from its file path.

    The module imports only numpy, so it is safe on a CPU-only machine.
    """
    path = BENCHMARK_MODULE_PATH if path is None else path
    key = str(path)
    if key in _MODULE_CACHE:
        return _MODULE_CACHE[key]
    require_upstream()
    spec = importlib.util.spec_from_file_location("upstream_set_terrain_benchmark", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    _MODULE_CACHE[key] = module
    return module


def dispatcher_task_names(module: ModuleType) -> List[str]:
    """Return task names in dispatcher order (the ``terrain_fns`` list inside ``set_terrain``).

    The list is a local variable, so we recover it from the function's constants
    and closure-free globals: every entry is a global function name referenced
    by ``set_terrain``.
    """
    import dis

    names: List[str] = []
    for ins in dis.get_instructions(module.set_terrain):
        if ins.opname in ("LOAD_GLOBAL",) and isinstance(ins.argval, str) and ins.argval.startswith("set_terrain_"):
            names.append(ins.argval[len("set_terrain_"):])
    return names


def defined_task_names(module: ModuleType) -> List[str]:
    """All ``set_terrain_*`` functions defined at module level (definition order)."""
    return [n[len("set_terrain_"):] for n, f in vars(module).items()
            if n.startswith("set_terrain_") and callable(f)]


def generate(module: ModuleType, cfg: BenchmarkTerrainCfg, col: int, row: int) -> SubTerrainStub:
    """Generate one benchmark sub-terrain exactly like ``Terrain.make_terrain`` (minus roughness).

    Returns the stub after ``set_terrain`` (before ``fix_terrain``).
    """
    variation, difficulty = cfg.variation(col), cfg.difficulty(row)
    set_seed(cfg.seed(variation, difficulty))
    terrain = make_stub(cfg)
    terrain.idx = int(module.set_terrain(terrain, variation, difficulty))
    return terrain
