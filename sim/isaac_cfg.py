"""Read the Go2 articulation config out of the read-only upstream repo.

The numbers that decide whether a clip replay can work -- the PD gains, the
effort clips, the zero-action joint pose, the control rate -- live in
``extreme-parkour/legged_gym/legged_gym/envs/base/legged_robot_config.py``.  They
are *not* Isaac Lab's stock Go2 defaults, so quoting the stock ones would be
quoting the wrong robot.

The file cannot be imported here (it imports ``isaaclab``), so it is parsed with
``ast``: no execution, no Isaac dependency, and it reads the actual file rather
than a copy that can drift.  If upstream changes a gain, this notices.

Nothing here writes to the upstream tree.
"""

from __future__ import annotations

import ast
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from terrain_toolkit import paths

CONFIG_REL = Path("extreme-parkour") / "legged_gym" / "legged_gym" / "envs" / "base" / "legged_robot_config.py"


@dataclass
class ActuatorGroup:
    name: str
    cls: str
    joint_names_expr: List[str]
    stiffness: Optional[float]
    damping: Optional[float]
    effort_limit: Optional[float]
    velocity_limit: Optional[float]

    @property
    def explicit(self) -> bool:
        """True when the PD is computed in Python each step, not by PhysX.

        ``ImplicitActuator`` hands the gains to PhysX at write time and the only
        way to change them afterwards is ``write_joint_*_to_sim``.  Everything
        else -- ``IdealPDActuator``, ``DCMotor``, ``ActuatorNetMLP`` -- computes
        torque in ``ActuatorBase.compute()`` during ``write_data_to_sim()``,
        reading ``self.stiffness`` at that moment.  That distinction is the whole
        question of whether gains can be scheduled at runtime.
        """
        return "Implicit" not in self.cls


@dataclass
class Go2Config:
    source: Path
    joint_pos: Dict[str, float]
    actuators: List[ActuatorGroup]
    sim_dt: Optional[float] = None
    decimation: Optional[int] = None
    action_scale: Optional[float] = None
    init_pos_z: Optional[float] = None
    _raw: dict = field(default_factory=dict, repr=False)

    @property
    def control_hz(self) -> Optional[float]:
        if self.sim_dt and self.decimation:
            return 1.0 / (self.sim_dt * self.decimation)
        return None

    @property
    def all_explicit(self) -> bool:
        return all(a.explicit for a in self.actuators)

    def group_of(self, joint_name: str) -> Optional[ActuatorGroup]:
        import re
        for a in self.actuators:
            if any(re.fullmatch(e, joint_name) for e in a.joint_names_expr):
                return a
        return None

    def ordered(self, legs: List[str], joints: List[str]) -> Tuple[List[str], List[float], List[float]]:
        """``(names, zero-action pose, effort limit)`` in the clips' own order."""
        names = [f"{l}_{j}_joint" for l in legs for j in joints]
        pose = [self.joint_pos[n] for n in names]
        lim = [(self.group_of(n).effort_limit if self.group_of(n) else None) for n in names]
        return names, pose, lim


def _const(node):
    if isinstance(node, ast.Constant):
        return node.value
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.USub):
        v = _const(node.operand)
        return -v if isinstance(v, (int, float)) else None
    return None


def _kw(call: ast.Call, name: str):
    for k in call.keywords:
        if k.arg == name:
            return k.value
    return None


def load(root: Path | None = None, robot: str = "UNITREE_GO2_CFG") -> Go2Config:
    src_path = (root or paths.require_upstream()) / CONFIG_REL
    tree = ast.parse(src_path.read_text())

    cfg_call = None
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and isinstance(node.value, ast.Call):
            for t in node.targets:
                if isinstance(t, ast.Name) and t.id == robot:
                    cfg_call = node.value
    if cfg_call is None:
        raise KeyError(f"{robot} not found in {src_path}")

    init = _kw(cfg_call, "init_state")
    joint_pos: Dict[str, float] = {}
    init_z = None
    if isinstance(init, ast.Call):
        jp = _kw(init, "joint_pos")
        if isinstance(jp, ast.Dict):
            for k, v in zip(jp.keys, jp.values):
                key, val = _const(k), _const(v)
                if isinstance(key, str) and isinstance(val, (int, float)):
                    joint_pos[key] = float(val)
        pos = _kw(init, "pos")
        if isinstance(pos, ast.Tuple) and len(pos.elts) == 3:
            init_z = _const(pos.elts[2])

    groups: List[ActuatorGroup] = []
    acts = _kw(cfg_call, "actuators")
    if isinstance(acts, ast.Dict):
        for k, v in zip(acts.keys, acts.values):
            if not isinstance(v, ast.Call):
                continue
            cls = v.func.id if isinstance(v.func, ast.Name) else ast.unparse(v.func)
            expr = _kw(v, "joint_names_expr")
            names = [_const(e) for e in expr.elts] if isinstance(expr, ast.List) else []
            groups.append(ActuatorGroup(
                name=str(_const(k)), cls=cls, joint_names_expr=[n for n in names if isinstance(n, str)],
                stiffness=_const(_kw(v, "stiffness")), damping=_const(_kw(v, "damping")),
                effort_limit=_const(_kw(v, "effort_limit")), velocity_limit=_const(_kw(v, "velocity_limit"))))

    # sim dt / decimation / action_scale live in class bodies elsewhere in the file
    scalars: Dict[str, float] = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            name, val = node.targets[0].id, _const(node.value)
            if name in ("dt", "decimation", "action_scale") and isinstance(val, (int, float)):
                scalars.setdefault(name, val)
        if isinstance(node, ast.keyword) and node.arg == "dt":
            v = _const(node.value)
            if isinstance(v, (int, float)):
                scalars.setdefault("dt", v)

    return Go2Config(src_path, joint_pos, groups, scalars.get("dt"),
                     int(scalars["decimation"]) if "decimation" in scalars else None,
                     scalars.get("action_scale"), init_z)


# --------------------------------------------------------------------------- #
# Convention comparison — the half of the sign question that needs no simulator
# --------------------------------------------------------------------------- #

@dataclass
class ConventionCheck:
    joint_names: List[str]
    isaac_pose: List[float]
    clip_pose: List[float]
    delta: List[float]
    best_signs: Tuple[int, int, int]
    err_identity: float
    err_best: float
    per_type: Dict[str, dict]

    @property
    def hip_flip_suspected(self) -> bool:
        return self.best_signs[0] < 0


def compare_pose(clip_pose, cfg: Go2Config, legs: List[str], joints: List[str]) -> ConventionCheck:
    """Does the clip's posture sit in the same frame as the zero-action pose?

    Compares a clip's mean commanded joint position with the articulation's
    ``init_state.joint_pos`` -- the pose the env produces at ``action = 0`` -- and
    reports whether flipping a joint type's sign brings them closer.

    This is weaker evidence than a replay: a gait-cycle mean is not a nominal
    stand, so a small offset proves nothing.  What it *can* show is a *crossing*:
    if the log's left hips are negative where Isaac's are positive, and the right
    hips mirror it, that is a frame difference and not a posture.
    """
    import itertools
    import numpy as np

    names, iso, _ = cfg.ordered(legs, joints)
    a = np.asarray(clip_pose, dtype=float)
    d = np.asarray(iso, dtype=float)
    best, best_err = (1, 1, 1), float(np.abs(a - d).mean())
    for s in itertools.product((1, -1), repeat=len(joints)):
        e = float(np.abs(a * np.tile(s, len(legs)) - d).mean())
        if e < best_err - 1e-12:
            best, best_err = s, e
    per: Dict[str, dict] = {}
    for k, jn in enumerate(joints):
        m = np.arange(len(names)) % len(joints) == k
        crossed = int(np.sum(np.sign(a[m]) * np.sign(d[m]) < 0))
        per[jn] = {
            "isaac": d[m].tolist(), "clip": a[m].tolist(),
            "mean_abs_delta": float(np.abs(a[m] - d[m]).mean()),
            "mean_abs_delta_flipped": float(np.abs(-a[m] - d[m]).mean()),
            "sign_crossings": crossed, "n": int(m.sum()),
        }
    return ConventionCheck(names, d.tolist(), a.tolist(), (a - d).tolist(),
                           best, float(np.abs(a - d).mean()), best_err, per)
