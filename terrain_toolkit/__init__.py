"""CPU-only tooling for the eurekaverse Go2 parkour benchmark terrains.

Nothing in this package imports Isaac Lab / Isaac Sim / torch.  The only
upstream file that is executed is ``set_terrain_benchmark.py`` (numpy only),
loaded via :func:`terrain_toolkit.stub.load_benchmark_module`.
"""

from .paths import PROJECT_ROOT, UPSTREAM_ROOT, DATA_DIR, OUTPUTS_DIR  # noqa: F401
from .stub import BenchmarkTerrainCfg, SubTerrainStub, make_stub, set_seed, load_benchmark_module  # noqa: F401
