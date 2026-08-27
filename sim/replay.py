"""Playing a clip through an Isaac Lab articulation, four ways.

Why more than one way
---------------------
``outputs/skill_clips.md`` finding (1): the sport controller schedules its PD
gains per skill.  WALK and TROT were recorded at kp 40 / kd 1 with zero
feed-forward torque, which is *exactly* the eurekaverse Go2 actuator config, so a
plain position replay is faithful.  RUN was recorded at kp 13/3/2 with ~12 Nm RMS
of calf feed-forward: the leg is nearly free and ``q_des`` is not what produces
the gait.  Replaying RUN as position targets under kp 40 commands a different
controller and will not reproduce it.

So there is no single replay mode.  There are four, in decreasing order of
fidelity and increasing order of "works without further API support":

``TORQUE``      per-step gains from the clip + per-step feed-forward torque.
                What the real robot did.  Needs runtime gain writes.
``FIXED_GAIN``  one gain pair per skill (the clip's median), set on the actuator
                config before the sim starts, + per-step feed-forward torque.
                Fallback (a): the gains are near-piecewise-constant per skill, so
                this loses the within-skill schedule and nothing else.
``FF_ONLY``     stock config gains + per-step feed-forward torque.  Fallback (b)
                for the case where gains cannot be changed at all.
``POSITION``    stock config gains, position targets only.  Correct for WALK and
                TROT; expected to fail for RUN.  This is the baseline, not the goal.

Feasibility of each is argued from the upstream source in
``outputs/gain_feasibility.md`` and probed at runtime by
``scripts/probe_isaac_actuator.py``.  Nothing here assumes the probe passed:
``apply_gains`` reports what it managed to do and the caller records it, so a
result can never silently be from a different mode than the one requested.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Sequence

import numpy as np


class ReplayMode(str, Enum):
    TORQUE = "torque"
    FIXED_GAIN = "fixed-gain"
    FF_ONLY = "ff-only"
    POSITION = "position"

    @property
    def needs_runtime_gains(self) -> bool:
        return self is ReplayMode.TORQUE

    @property
    def needs_effort(self) -> bool:
        return self in (ReplayMode.TORQUE, ReplayMode.FIXED_GAIN, ReplayMode.FF_ONLY)


#: What to fall back to when a mode's requirement is not available.
FALLBACK: Dict[ReplayMode, Optional[ReplayMode]] = {
    ReplayMode.TORQUE: ReplayMode.FIXED_GAIN,
    ReplayMode.FIXED_GAIN: ReplayMode.FF_ONLY,
    ReplayMode.FF_ONLY: ReplayMode.POSITION,
    ReplayMode.POSITION: None,
}


def default_mode_for(position_controlled: bool) -> ReplayMode:
    """POSITION is honest for a position-controlled clip; TORQUE otherwise."""
    return ReplayMode.POSITION if position_controlled else ReplayMode.TORQUE


@dataclass
class Capabilities:
    """What the installed Isaac Lab actually let us do, discovered not assumed."""

    explicit_actuators: bool = False
    runtime_gain_write: bool = False
    effort_target: bool = False
    implicit_gain_write: bool = False           # write_joint_stiffness_to_sim present
    notes: List[str] = field(default_factory=list)

    def supports(self, mode: ReplayMode) -> bool:
        if mode.needs_runtime_gains and not (self.runtime_gain_write or self.implicit_gain_write):
            return False
        if mode.needs_effort and not self.effort_target:
            return False
        return True

    def resolve(self, mode: ReplayMode) -> ReplayMode:
        """Walk down the fallback chain until something is supported."""
        seen = set()
        m: Optional[ReplayMode] = mode
        while m is not None and m not in seen:
            if self.supports(m):
                return m
            seen.add(m)
            self.notes.append(f"{m.value} unsupported -> falling back to {FALLBACK[m]}")
            m = FALLBACK[m]
        return ReplayMode.POSITION


def probe_capabilities(robot, cfg_all_explicit: bool) -> Capabilities:
    """Ask the live articulation what it supports.  Never raises."""
    cap = Capabilities(explicit_actuators=cfg_all_explicit)
    cap.effort_target = hasattr(robot, "set_joint_effort_target")
    cap.implicit_gain_write = hasattr(robot, "write_joint_stiffness_to_sim")
    try:
        acts = list(getattr(robot, "actuators", {}).values())
    except Exception as exc:
        cap.notes.append(f"no actuators attribute: {exc}")
        return cap
    if not acts:
        cap.notes.append("articulation exposes no actuator groups")
        return cap
    a = acts[0]
    if getattr(a, "stiffness", None) is None:
        cap.notes.append(f"{type(a).__name__} has no stiffness (actuator net?); gains are not schedulable")
        return cap
    try:
        original = a.stiffness.clone() if hasattr(a.stiffness, "clone") else a.stiffness
        a.stiffness = original * 1.0            # assignment must survive
        cap.runtime_gain_write = True
        a.stiffness = original
    except Exception as exc:
        cap.notes.append(f"actuator stiffness is not writable: {exc}")
    if cap.runtime_gain_write and not cap.explicit_actuators:
        cap.notes.append("gains are writable but the actuator is implicit: PhysX holds the PD, so the "
                         "write only takes effect through write_joint_stiffness_to_sim")
        cap.runtime_gain_write = bool(cap.implicit_gain_write)
    return cap


def apply_gains(robot, dof_index, kp: np.ndarray, kd: np.ndarray, cap: Capabilities) -> bool:
    """Write per-joint gains for this step.  ``kp``/``kd`` are length-12, clip order.

    For an explicit actuator the write lands in the Python-side PD that runs
    during ``write_data_to_sim()``, so it takes effect on this very step.  For an
    implicit one it has to go through PhysX, which is a different call and a
    different latency; both paths are handled and the caller is told which ran.
    """
    import torch

    ok = False
    for act in robot.actuators.values():
        ids = act.joint_indices
        cols = _clip_cols_for(dof_index, ids, robot)
        if cols is None:
            continue
        try:
            act.stiffness = _broadcast_like(act.stiffness, kp[cols])
            act.damping = _broadcast_like(act.damping, kd[cols])
            ok = True
        except Exception as exc:                                  # pragma: no cover
            cap.notes.append(f"gain write failed on {act}: {exc}")
    if ok and cap.implicit_gain_write and not cap.explicit_actuators:
        try:
            full_kp = torch.as_tensor(kp, device=robot.device, dtype=torch.float32)
            full_kd = torch.as_tensor(kd, device=robot.device, dtype=torch.float32)
            robot.write_joint_stiffness_to_sim(full_kp.expand(robot.num_instances, -1), joint_ids=dof_index)
            robot.write_joint_damping_to_sim(full_kd.expand(robot.num_instances, -1), joint_ids=dof_index)
        except Exception as exc:                                  # pragma: no cover
            cap.notes.append(f"write_joint_stiffness_to_sim failed: {exc}")
            ok = False
    return ok


def _broadcast_like(target, values):
    import torch
    v = torch.as_tensor(np.asarray(values), device=target.device, dtype=target.dtype)
    return v.expand_as(target).clone() if v.shape != target.shape else v.clone()


def _clip_cols_for(dof_index, ids, robot):
    """Which clip columns belong to this actuator group."""
    import torch
    dof = dof_index.tolist() if hasattr(dof_index, "tolist") else list(dof_index)
    if isinstance(ids, slice):
        member = set(range(robot.num_joints)[ids])
    else:
        member = set(ids.tolist() if hasattr(ids, "tolist") else list(ids))
    cols = [i for i, d in enumerate(dof) if d in member]
    return np.asarray(cols, dtype=int) if cols else None


# --------------------------------------------------------------------------- #
# Torque headroom — the other reason RUN and JUMP may not reproduce
# --------------------------------------------------------------------------- #

@dataclass
class Headroom:
    joint_type: str
    peak_logged_nm: float
    p999_logged_nm: float
    limit_nm: float

    @property
    def over(self) -> bool:
        return self.peak_logged_nm > self.limit_nm

    @property
    def ratio(self) -> float:
        return self.peak_logged_nm / self.limit_nm if self.limit_nm else float("nan")


def torque_headroom(tau_logged: np.ndarray, effort_limits: Sequence[float],
                    joints: Sequence[str]) -> List[Headroom]:
    """Compare what the real robot applied against the sim's static effort clip.

    ``IdealPDActuator`` clips torque at ``effort_limit`` with no torque-speed
    curve.  Where the logged torque exceeded that clip the sim robot is simply
    weaker than the one that produced the clip, and any limit calibrated on it is
    a property of the *sim* robot.  That is the right thing to calibrate -- the
    planner runs in sim -- but it must be said out loud rather than discovered
    when a jump comes out short.
    """
    n_j = len(joints)
    out = []
    for k, jn in enumerate(joints):
        m = np.arange(tau_logged.shape[1]) % n_j == k
        lim = float(np.asarray(effort_limits)[m][0])
        a = np.abs(tau_logged[:, m])
        out.append(Headroom(jn, float(a.max()), float(np.percentile(a, 99.9)), lim))
    return out


# --------------------------------------------------------------------------- #
# Ground contact -- reproducing the env's friction, which is a product of two
# --------------------------------------------------------------------------- #

def ground_material_cfg(sim_utils):
    """The terrain material the env builds, verbatim.

    ``legged_robot.py::_create_trimesh`` spawns the terrain with static 1.0 /
    dynamic 1.0 / restitution 0.0 and -- the part that matters -- BOTH combine
    modes set to ``multiply``.  Isaac Lab's default is ``average``, so a harness
    that only overrides the ground's coefficient does not get that coefficient:
    it gets the average of the ground and whatever material the robot's USD
    happens to ship.  The effective friction is a product of two materials and
    only one of them was ever being set.
    """
    return sim_utils.RigidBodyMaterialCfg(
        static_friction=1.0, dynamic_friction=1.0, restitution=0.0,
        friction_combine_mode="multiply", restitution_combine_mode="multiply")


def set_robot_friction(robot, mu: float):
    """Put ``mu`` on every one of the robot's collision shapes.

    This is the half the env randomises: ``_process_rigid_shape_props`` samples one
    coefficient per environment from ``friction_range`` and writes it to all of the
    robot's shapes, which then combines multiplicatively with the terrain's 1.0.
    Replaying a clip needs one fixed floor rather than a per-episode sample, so the
    sampled draw is replaced by a fixed ``mu`` -- but it has to be written in the
    same place, or the terrain override alone does nothing.

    Returns ``(before, after)`` mean static friction so the caller can report what
    the USD was actually shipping, or ``None`` if this build exposes no way to
    write shape materials.
    """
    try:
        import warp as wp
        import torch
        view = robot.root_view
        mats = wp.to_torch(view.get_material_properties())      # (envs, shapes, 3)
        before = float(mats[..., 0].mean().item())
        mats[..., 0] = mu          # static
        mats[..., 1] = mu          # dynamic
        ids = torch.arange(mats.shape[0], dtype=torch.int32)
        view.set_material_properties(wp.from_torch(mats.contiguous(), dtype=wp.float32),
                                     wp.from_torch(ids, dtype=wp.int32))
        after = float(wp.to_torch(view.get_material_properties())[..., 0].mean().item())
        return before, after
    except Exception as exc:                                    # pragma: no cover
        print(f"[replay] WARNING could not write the robot's shape friction ({exc}); "
              f"the effective coefficient is the ground's combined with whatever "
              f"go2.usd ships, which is not the env's")
        return None
