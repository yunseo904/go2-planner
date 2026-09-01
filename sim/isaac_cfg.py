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
    friction_range: Optional[Tuple[float, float]] = None
    _raw: dict = field(default_factory=dict, repr=False)

    @property
    def control_hz(self) -> Optional[float]:
        if self.sim_dt and self.decimation:
            return 1.0 / (self.sim_dt * self.decimation)
        return None

    @property
    def ground_friction(self) -> Optional[float]:
        """The single ground friction the harness should stand on.

        The env randomises friction per episode over ``friction_range``; a replay
        or a calibration probe needs one fixed number, because a step-clearance
        threshold measured on a randomised floor folds friction variance into a
        geometric answer.  The midpoint is the least arbitrary fixed choice
        inside the distribution the policy is trained and evaluated on, and it
        comes from the env rather than from anything we measured -- CLAUDE.md 2
        forbids picking it to suit a score.  Isaac's own default (0.5) is below
        the whole range, so it is not a neutral fallback.
        """
        if not self.friction_range:
            return None
        lo, hi = self.friction_range
        return 0.5 * (lo + hi)

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
    ranges: Dict[str, Tuple[float, float]] = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            name, val = node.targets[0].id, _const(node.value)
            if name in ("dt", "decimation", "action_scale") and isinstance(val, (int, float)):
                scalars.setdefault(name, val)
            # friction_range lives in a domain-randomisation class body.  Reading it
            # from the AST means the commented-out alternatives in that file cannot
            # be picked up by accident -- only the assignment that is actually live.
            if name == "friction_range" and isinstance(node.value, (ast.List, ast.Tuple)):
                vals = [_const(e) for e in node.value.elts]
                if len(vals) == 2 and all(isinstance(v, (int, float)) for v in vals):
                    ranges.setdefault(name, (float(vals[0]), float(vals[1])))
        if isinstance(node, ast.keyword) and node.arg == "dt":
            v = _const(node.value)
            if isinstance(v, (int, float)):
                scalars.setdefault("dt", v)

    return Go2Config(src_path, joint_pos, groups, scalars.get("dt"),
                     int(scalars["decimation"]) if "decimation" in scalars else None,
                     scalars.get("action_scale"), init_z,
                     ranges.get("friction_range"))


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


def _fold(node):
    """literal_eval, plus the constant arithmetic this config is written in.

    ``ast.literal_eval`` refuses ``0.245 + 0.027`` and ``int(640 / 6)``, and CustomDepthCfg
    writes its mount and its resolution exactly that way -- as the sum of the two offsets
    that make it up, and as the D435's resolution over the scale factor.  Those spellings
    are the documentation, so they are folded here rather than transcribed as 0.272 and 106.
    Numbers, the four arithmetic operators, unary minus and ``int()`` only; anything else
    raises and the field is skipped.
    """
    if isinstance(node, ast.Constant):
        return node.value
    if isinstance(node, (ast.List, ast.Tuple)):
        return [_fold(e) for e in node.elts]
    if isinstance(node, ast.Dict):
        return {_fold(k): _fold(v) for k, v in zip(node.keys, node.values)}
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.UAdd, ast.USub)):
        v = _fold(node.operand)
        return v if isinstance(node.op, ast.UAdd) else -v
    if isinstance(node, ast.BinOp):
        a, b = _fold(node.left), _fold(node.right)
        for op, fn in ((ast.Add, lambda x, y: x + y), (ast.Sub, lambda x, y: x - y),
                       (ast.Mult, lambda x, y: x * y), (ast.Div, lambda x, y: x / y)):
            if isinstance(node.op, op):
                return fn(a, b)
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) \
            and node.func.id == "int" and len(node.args) == 1:
        return int(_fold(node.args[0]))
    raise ValueError(f"not a constant expression: {ast.dump(node)[:60]}")


def depth_cfg(root: Path | None = None) -> Dict[str, object]:
    """``CustomDepthCfg`` out of the same read-only upstream file.

    The depth arm has to be the fork's camera or it is not measuring the fork's robot:
    the mount, the field of view, the resolution, the crop, the two noises, the update
    rate and the one-step latency are all things a policy is trained against.  Parsed with
    ``ast`` for the same reason as ``load()`` -- the file imports ``isaaclab`` and cannot be
    imported here, and a copy of these numbers would drift silently.

    ``blur_prob`` and ``erase_prob`` are read but are 0.0 upstream; the caller applies only
    what is non-zero, and a future non-zero value shows up here rather than being ignored.
    """
    src_path = (root or paths.require_upstream()) / CONFIG_REL
    tree = ast.parse(src_path.read_text())
    node = None
    for n in ast.walk(tree):
        if isinstance(n, ast.ClassDef) and n.name == "CustomDepthCfg":
            node = n
    if node is None:
        raise KeyError(f"CustomDepthCfg not found in {src_path}")

    vals: Dict[str, object] = {}
    for stmt in node.body:
        if not isinstance(stmt, ast.Assign) or len(stmt.targets) != 1:
            continue
        tgt = stmt.targets[0]
        try:
            val = _fold(stmt.value)
        except (ValueError, KeyError, TypeError):
            continue          # e.g. processed_resolution, which names other fields
        if isinstance(tgt, ast.Name):
            vals[tgt.id] = val
        elif isinstance(tgt, ast.Tuple):
            # `crop_top, crop_bottom, crop_left, crop_right = 0, 0, 8, 8` -- the crop is
            # written as one line and it is the field this parser most needs.
            names = [e.id for e in tgt.elts if isinstance(e, ast.Name)]
            if len(names) == len(tgt.elts) and isinstance(val, list) \
                    and len(val) == len(names):
                vals.update(dict(zip(names, val)))

    pos = tuple(float(v) for v in vals["position"]["mean"])
    res = tuple(int(v) for v in vals["original_resolution"])
    cl, cr = int(vals["crop_left"]), int(vals["crop_right"])
    ct, cb = int(vals["crop_top"]), int(vals["crop_bottom"])
    return {
        "pos": pos,
        "pitch_rad": float(vals["rotation"]["mean"][1]),
        "horizontal_fov": float(vals["horizontal_fov"]),
        "resolution": res,                                   # (w, h) as rendered
        "processed": (res[0] - cl - cr, res[1] - ct - cb),   # (w, h) after the crop
        "crop_left": cl, "crop_right": cr, "crop_top": ct, "crop_bottom": cb,
        "near_clip": float(vals["near_clip"]),
        "far_clip": float(vals["far_clip"]),
        "bias_noise": float(vals["bias_noise"]),
        "granular_noise": float(vals["granular_noise"]),
        "blackout_noise": float(vals["blackout_noise"]),
        "blur_prob": float(vals["blur_prob"]),
        "erase_prob": float(vals["erase_prob"]),
        "update_interval": int(vals["update_interval"]),
        "delay_steps": int(vals["depth_delay_steps"]),
    }
