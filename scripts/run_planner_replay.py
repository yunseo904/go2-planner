#!/usr/bin/env python3
"""Drive the rule planner and actually execute what it chooses, on flat ground.

    # no GPU needed -- physics is on the CPU and nothing renders
    scripts/isaac_docker_run.sh scripts/run_planner_replay.py --headless --device cpu
    scripts/isaac_docker_run.sh scripts/run_planner_replay.py --headless --device cpu \
        --schedule pairs --results-csv outputs/planner_replay.csv

    # anywhere, no Isaac Lab
    python3 scripts/run_planner_replay.py --plan        # the schedule and what it should elicit
    python3 scripts/run_planner_replay.py --self-test   # the wiring, with no simulator

What this is for
----------------
Everything until now has replayed ONE clip chosen on the command line.  This
closes the loop the experiment is actually about: the planner picks a skill, the
executor plays that skill's clip, and when the planner changes its mind the gait
changes underneath the robot while it is moving.

It is a WIRING test, not an evaluation.  The thresholds it drives the planner
with are the config's placeholders and the terrain it "sees" is a script, not a
height field -- the question here is whether the planner chooses, whether the
switch reaches the robot, whether the gait after a switch is the gait that was
asked for, and whether it chatters.  Nothing measured here says the planner
decides *well*.

Division of information, which is the point of the whole design:

* the PLANNER sees the (scripted) terrain features and the heading error
* the LOW LEVEL sees ``BaseState`` -- base velocity and yaw rate -- and nothing
  else.  It never receives terrain, depth, or the goal.

RUN and JUMP stay in the interface and are refused by the executor with a reason
(``planner.skills.UNSUPPORTED_REASON``); the robot carries on with the skill it
was already playing rather than falling over or silently substituting one.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from planner.config import DEFAULT
from planner.features import Observation, FEATURE_NAMES
from planner.rules import RulePlanner
from planner.skills import (BaseState, SkillId, SUPPORTED, UNSUPPORTED_REASON,
                            build_library, make_policy)
from sim import diagnose as D
from sim import isaac_cfg as IC
from sim.footcomp import FootPlacement, stance_time_s
from sim.yawmoment import YawMoment
from terrain_toolkit.paths import SKILL_CLIPS_META_JSON

_SIM_APP = None
_RUN_UTC = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


# --------------------------------------------------------------------------- #
# The scripted terrain.  Not a height field -- the planner's inputs, written
# down, so a transition can be demanded rather than waited for.
# --------------------------------------------------------------------------- #
#: ``(seconds, roughness_m, heading_err_deg, what it should elicit)``
#:
#: roughness is the lever: it is the one feature all three straight gaits are
#: ranked on (RUN 0.015, TROT 0.03, WALK inf), and unlike step_up it cannot
#: accidentally trip the jump gate.  heading_err drives TURN, which is off that
#: axis entirely.
SCHEDULES = {
    "pairs": [
        (6.0, 0.000,  0.0, "flat: planner asks for RUN, executor refuses it"),
        (6.0, 0.020,  0.0, "rough past RUN's limit -> TROT"),
        (6.0, 0.050,  0.0, "rougher past TROT's limit -> WALK"),
        (6.0, 0.020,  0.0, "back under TROT's limit -> TROT   [WALK->TROT]"),
        (6.0, 0.020, 25.0, "heading off -> TURN                [TROT->TURN]"),
        (6.0, 0.020,  0.0, "heading recovered -> TROT          [TURN->TROT]"),
        (6.0, 0.050,  0.0, "rough -> WALK                      [TROT->WALK]"),
        (6.0, 0.050, 25.0, "heading off while rough -> TURN    [WALK->TURN]"),
        (6.0, 0.050,  0.0, "heading recovered, still rough     [TURN->WALK]"),
    ],
    "hold": [(40.0, 0.020, 0.0, "TROT held, no switches: the control for everything else")],
    # roughness past TROT's 0.030 limit, so the planner asks for WALK and keeps asking.
    # "hold" alone asks for TROT, which is why an --initial WALK run on it switches at
    # the first tick instead of holding anything.
    "hold_walk": [(40.0, 0.050, 0.0, "WALK held, no switches")],
    # TROT -> TURN direct is the one switch that still fails, and the speed gap is the
    # suspect: 0.64 m/s into a gait whose measured speed is 0.008.  WALK sits between
    # them at 0.23, so this asks whether it can serve as the bridge -- TROT -> WALK ->
    # TURN, each leg of it a switch that already works.
    "bridge": [
        (10.0, 0.020,  0.0, "TROT, established"),
        (6.0,  0.050,  0.0, "rough -> WALK                       [TROT->WALK]"),
        (10.0, 0.050, 25.0, "heading off while rough -> TURN     [WALK->TURN]"),
        (8.0,  0.050,  0.0, "heading recovered -> WALK           [TURN->WALK]"),
    ],
    # the control: the same TURN demand reached directly from TROT
    "direct": [
        (10.0, 0.020,  0.0, "TROT, established"),
        (10.0, 0.020, 25.0, "heading off -> TURN                 [TROT->TURN]"),
        (8.0,  0.020,  0.0, "heading recovered -> TROT           [TURN->TROT]"),
    ],
    "chatter": [   # a feature parked exactly on TROT's limit, to provoke oscillation
        (4.0, 0.020, 0.0, "TROT"),
        (24.0, 0.0301, 0.0, "roughness parked 0.1 mm over TROT's limit"),
        (4.0, 0.020, 0.0, "TROT"),
    ],
}


def schedule_at(sched, t: float):
    """``(roughness_m, heading_err_deg, label, index)`` at time ``t``."""
    acc = 0.0
    for i, (dur, rough, head, label) in enumerate(sched):
        if t < acc + dur:
            return rough, head, label, i
        acc += dur
    dur, rough, head, label = sched[-1]
    return rough, head, label, len(sched) - 1


def total_s(sched) -> float:
    return float(sum(seg[0] for seg in sched))


def make_obs(roughness_m: float) -> Observation:
    """A full-confidence observation carrying one feature that matters.

    Everything else is set to "nothing here": zero steps, no gap, a corridor
    wider than the body.  A blind or stale observation is a different test --
    the planner already refuses to upgrade on those and that path is covered by
    the offline sweep.
    """
    f = {k: 0.0 for k in FEATURE_NAMES}
    f["width_min_m"] = 2.0
    f["slope_max_deg"] = 0.0
    f["slope_mean_deg"] = 0.0
    f["roughness_m"] = float(roughness_m)
    return Observation(features=f, confidence=1.0, lookahead_m=0.5,
                       observed_from_m=0.0, observed_to_m=1.0, n_samples=32)


# --------------------------------------------------------------------------- #
def plan_report(sched_name: str) -> str:
    sched = SCHEDULES[sched_name]
    cfg = DEFAULT
    lib = build_library(cfg)
    L = [f"schedule {sched_name!r}: {len(sched)} segments, {total_s(sched):.0f} s", ""]
    L.append(f"  {'t0':>6s} {'t1':>6s} {'rough':>7s} {'head':>6s}  expects")
    acc = 0.0
    for dur, rough, head, label in sched:
        L.append(f"  {acc:6.1f} {acc+dur:6.1f} {rough:7.3f} {head:6.1f}  {label}")
        acc += dur
    L += ["", "  skill limits the roughness column is being read against:"]
    for sid in (SkillId.RUN, SkillId.TROT, SkillId.WALK):
        r = lib[sid].roughness_max_m
        L.append(f"    {sid.value:5s} roughness_max {r if np.isfinite(r) else float('inf'):.3f} m"
                 f"   executable={sid in SUPPORTED}")
    L += ["", f"  TURN fires at |heading error| > {cfg.skill.HEADING_ERR_TURN_DEG:.1f} deg "
              f"(CALIBRATION_NEEDED placeholder)",
          f"  switch delay {cfg.switch.SWITCH_DELAY:.2f} s, min hold {cfg.switch.MIN_HOLD_S:.2f} s,"
          f" hysteresis {cfg.switch.HYSTERESIS:.2f}",
          "",
          "  NOTE every threshold above is a placeholder. This exercises the wiring;",
          "  it is not evidence that the planner decides correctly."]
    return "\n".join(L)


def self_test() -> int:
    """Run the planner + policies with no simulator: does the wiring hold?"""
    from planner.skills import ClipPolicy, UnsupportedPolicy
    fails = 0
    print("== planner drives the schedule ==")
    for name in ("pairs", "chatter", "hold"):
        sched = SCHEDULES[name]
        pl = RulePlanner(initial=SkillId.WALK)
        seen, refused, dt = [], 0, 1.0 / DEFAULT.feature.TICK_HZ
        t = 0.0
        while t < total_s(sched):
            rough, head, _, _ = schedule_at(sched, t)
            d = pl.step(make_obs(rough), dt, x_m=t * 0.3, heading_err_deg=head)
            if d.switched:
                seen.append(d.active)
            if d.requested not in SUPPORTED:
                refused += 1
            t += dt
        got = " ".join(s.value for s in seen)
        print(f"  {name:8s} switches={pl.switches:2d}  requests for unsupported skills={refused:4d}"
              f"  sequence: {got or '(none)'}")
        if name == "pairs":
            need = {(SkillId.WALK, SkillId.TROT), (SkillId.TROT, SkillId.WALK),
                    (SkillId.TROT, SkillId.TURN), (SkillId.TURN, SkillId.TROT),
                    (SkillId.WALK, SkillId.TURN), (SkillId.TURN, SkillId.WALK)}
            pairs = set(zip([SkillId.WALK] + seen, seen))
            missing = need - pairs
            ok = not missing
            fails += 0 if ok else 1
            print(f"  {'ok  ' if ok else 'FAIL'} all six ordered pairs elicited"
                  + ("" if ok else f"; missing {sorted((a.value, b.value) for a, b in missing)}"))

    print("\n== the executor refuses what it cannot do, and says why ==")
    for sid in (SkillId.RUN, SkillId.JUMP):
        pol = make_policy(sid, clips={})
        c = pol.act(BaseState(), 0.02)
        ok = isinstance(pol, UnsupportedPolicy) and not c.supported and c.q is None
        fails += 0 if ok else 1
        print(f"  {'ok  ' if ok else 'FAIL'} {sid.value}: {c.note[:88]}")

    print("\n== a clip policy emits joint targets, and the correction only moves swing hips ==")
    n = 24
    clip = {"name": "FAKE", "q_des": np.zeros((n, 12), np.float32),
            "contact": np.tile(np.array([1, 0, 0, 1], np.uint8), (n, 1)), "fs": 50.0}
    fp = FootPlacement(t_stance_s=0.372, lever_m=np.full(4, 0.31), hip_x_m=np.array([.19, .19, -.19, -.19]),
                       cap_rad=0.05)
    pol = make_policy(SkillId.TROT, clips={SkillId.TROT: clip}, foot_for=lambda s, c: fp)
    c = pol.act(BaseState(vy=0.3), 0.02)
    q = np.asarray(c.q)
    moved = np.flatnonzero(np.abs(q) > 1e-9)
    want = [3, 6]                       # FR and RL hips: the two legs in swing
    ok = list(moved) == want
    fails += 0 if ok else 1
    print(f"  {'ok  ' if ok else 'FAIL'} moved columns {list(moved)} (want {want}: swing legs' hips only)")

    print(f"\nself-test: {'PASS' if fails == 0 else f'{fails} FAILURES'}")
    return 0 if fails == 0 else 1


# --------------------------------------------------------------------------- #
# Per-skill foot-placement settings.
#
# NOT one setting for all three.  Each is the configuration that skill was
# measured to hold on, and they differ because the skills differ:
#
#   WALK, TROT  yaw target off, cap 0.05 rad.  The straight clips' logged yaw is
#               ~0, and closing a loop on yaw rate made heading WORSE in every
#               variant tried (outputs/heading_candidates.md 3), so the lateral
#               term runs alone.  0.05 is above the 0.03 rad where TROT starts
#               holding and below where anything degrades.
#   TURN        yaw target from the log, averaged over one cycle, cap 0.05.  A
#               turn's foot placement is only well posed against the rotation's
#               own per-leg target (outputs/turn_target.md); with v_y = 0 the
#               correction cancels the turn and the clip falls sooner than
#               untouched.  Cycle-averaged because the instantaneous yaw rate is
#               3.9x noisier than the bias it corrects.
# --------------------------------------------------------------------------- #
FOOT_COMP = {
    SkillId.WALK: dict(yaw_mode="off", cap_rad=0.05),
    SkillId.TROT: dict(yaw_mode="off", cap_rad=0.05),
    SkillId.TURN: dict(yaw_mode="log-cycle", cap_rad=0.05),
}

#: With --heading on, the yaw mode is replaced and each skill gets the heading cap the
#: open-loop steering probe measured it tolerates: WALK took +-0.04 rad with no cost,
#: TROT falls in BOTH directions at +-0.04 and is safe at +-0.02
#: (outputs/heading_candidates.md 2).  TURN is excluded -- it is not trying to hold a
#: heading, it is trying to change one, and its own log yaw rate is the target.
HEADING_CAP = {SkillId.WALK: 0.04, SkillId.TROT: 0.02, SkillId.TURN: 0.0}

#: --yaw-moment caps, N.m of hip feed-forward torque, per skill.  TROT is the only
#: skill this was built for: outputs/trot_straight.md measures its foot-placement
#: heading authority as spent at +-0.02 rad, while WALK already reaches the benchmark
#: budget (0.24 deg/m) without any of this.  Adding a second heading actuator to a
#: skill that does not need one would be a change with nothing to show for it and a
#: control group to spoil, so WALK and TURN are 0 -- OFF -- until something asks.
#: The number itself is set by the probe, not here; 0 means the term never runs.
YAW_MOMENT_CAP_NM = {SkillId.WALK: 0.0, SkillId.TROT: 0.0, SkillId.TURN: 0.0}


def run_isaac(args) -> dict:
    from isaaclab.app import AppLauncher

    global _SIM_APP
    _SIM_APP = AppLauncher(args).app

    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.assets import Articulation
    from isaaclab.sensors import ContactSensor, ContactSensorCfg
    from isaaclab.sim import SimulationCfg, SimulationContext

    from verify_skill_replay import (_log_motion_for, expected_from_meta, level_start,
                                     load_clip, report, rotate_clip)
    from sim.replay import (foot_body_ids, ground_material_cfg, quat_rotate_inv,
                            quat_to_rpy_deg, set_robot_friction, snap)

    ucfg = IC.load()
    sys.path.insert(0, str(Path(ucfg.source).parents[3]))
    from legged_gym.envs.base.legged_robot_config import UNITREE_GO2_CFG

    meta = json.loads(SKILL_CLIPS_META_JSON.read_text())
    sched = SCHEDULES[args.schedule]
    print(plan_report(args.schedule))

    # One SimulationContext takes one dt, and the clips do not share a rate (49.4 to
    # 50.6 Hz -- each carries an integer number of samples per gait cycle, so its rate
    # lands near 50 rather than on it).  So the physics step is the config's sim_dt and
    # every clip is played one frame per control step at the config's control rate; the
    # residual per-clip rate error is reported below and is under 1.5%.  Stepping each
    # clip at its own rate is not available to a run that switches between them.
    phys_dt = float(ucfg.sim_dt)
    decim = int(ucfg.decimation or 4)
    dt = decim * phys_dt
    clips = {sid: load_clip(sid.value, args.rate) for sid in SUPPORTED}
    print(f"[planner] physics {1/phys_dt:.0f} Hz x decimation {decim} -> control {1/dt:.1f} Hz")
    for sid, c in clips.items():
        print(f"[planner]   {sid.value:5s} clip {c['fs']:.2f} Hz -> played at {1/dt:.2f} Hz "
              f"({100*((dt) * c['fs'] - 1):+.2f}% rate error), {len(c['q_des'])} frames/cycle")

    sim = SimulationContext(SimulationCfg(dt=phys_dt, device=args.device))
    mu = ucfg.ground_friction
    ground = sim_utils.GroundPlaneCfg(physics_material=ground_material_cfg(sim_utils))
    ground.func("/World/ground", ground)
    sim_utils.DomeLightCfg(intensity=2000.0).func("/World/light", sim_utils.DomeLightCfg(intensity=2000.0))
    robot = Articulation(UNITREE_GO2_CFG.replace(prim_path="/World/Robot"))
    contacts = ContactSensor(ContactSensorCfg(prim_path="/World/Robot/.*_foot",
                                              history_length=1, track_air_time=True))
    sim.reset()
    if mu is not None:
        set_robot_friction(robot, mu)
        print(f"[planner] ground/robot friction -> {mu:.2f}")

    want = [f"{leg}_{j}_joint" for leg in clips[SkillId.WALK]["leg_order"]
            for j in clips[SkillId.WALK]["joint_order"]]
    idx = [robot.joint_names.index(n) for n in want]
    idx_t = torch.as_tensor(idx, device=sim.device, dtype=torch.long)
    foot_ids, foot_names = foot_body_ids(robot)

    # The hip's effort limit, READ FROM THE ARTICULATION.  --yaw-moment adds torque on
    # top of the PD and IdealPDActuator clips the sum at this value SILENTLY, so the
    # number that bounds the new term has to be the one the actuator will actually
    # enforce.  Refused rather than defaulted: a nominal 23.7 that disagreed with the
    # installed config would make every headroom statement in the run a fiction.
    #
    # READ FROM THE ACTUATOR, not from robot.data.  The first version of this read
    # robot.data.joint_effort_limits and got 1e9, which is the PHYSX limit -- and PhysX
    # is not what clips here.  The actuator is an explicit IdealPDActuator, so the clip
    # that binds is applied in Python, in ActuatorBase._clip_effort, against
    # actuator.effort_limit.  With 1e9 in hand the headroom guard below accepted any cap
    # at all and the saturation counter could never fire: a guard that is always open.
    # Caught by the banner printing "against a 1000000000.00 Nm limit" next to a peak of
    # 11.79 Nm.
    def _hip_effort_limit() -> float:
        want_hip = robot.joint_names[idx[0]]                       # e.g. FL_hip_joint
        for act in robot.actuators.values():
            names = list(getattr(act, "joint_names", []) or [])
            e = getattr(act, "effort_limit", None)
            if e is None or want_hip not in names:
                continue
            arr = np.asarray(snap(e)).reshape(-1)
            v = float(arr[names.index(want_hip)] if arr.size == len(names) else arr.flat[0])
            if np.isfinite(v) and 0.0 < v < 1e4:
                return v
        return float("nan")

    hip_effort_limit_nm = _hip_effort_limit()
    if not np.isfinite(hip_effort_limit_nm):
        # Refused, not defaulted.  --yaw-moment's whole safety argument is "the sum of
        # the PD and the feed-forward stays under the limit the actuator enforces"; with
        # the limit unknown that argument is not available and the run must not pretend
        # otherwise.  Position-only runs are unaffected.
        if args.yaw_moment != "off":
            raise SystemExit(
                "[planner] REFUSING --yaw-moment: could not read the hip's effort limit "
                "off the articulation, so the headroom this term needs cannot be checked. "
                "outputs/isaac_actuator_probe.json measured it at 23.70 Nm on this config; "
                "if that is still right, the read above is what is broken, not the limit.")
        hip_effort_limit_nm = float("nan")
    else:
        print(f"[planner] hip effort limit {hip_effort_limit_nm:.2f} Nm "
              f"(the actuator's own clip, not PhysX's)")
    hip_sat_steps = 0

    # Entry phase per clip, by the same kinematic criterion the single-clip harness
    # uses: the frame whose four feet are closest to coplanar.  A switch lands on
    # that frame, so a mid-run gait change starts from a pose the robot can stand on
    # rather than from wherever frame 0 happens to be.
    #
    # TURN is the exception and it is a MEASURED one.  outputs/turn_entry_phase.md swept
    # all 45 phases of the TURN clip over 9 flat cells in both compensator arms: the
    # frame `level_start` picks (24) sits inside a contiguous ten-frame dead band that
    # fails in BOTH arms, and it is picked precisely because it has the best foot spread
    # in the cycle -- the criterion actively misleads here.  Frame 6 is the only phase
    # that passes 9/9 in both arms with both neighbours doing the same, and it is already
    # in the config as skill.ENTRY_FRAME_TURN with provenance MEASURED.
    #
    # Default stays `rule`, so every earlier result reproduces.  --entry-turn measured is
    # the on/off pair.
    for sid in list(clips):
        k, ph = level_start(clips[sid], robot, sim, idx_t, phys_dt)
        note = ""
        if (args.entry_turn == "measured" and sid is SkillId.TURN
                and DEFAULT.skill.ENTRY_FRAME_TURN >= 0):
            k = int(DEFAULT.skill.ENTRY_FRAME_TURN) % len(clips[sid]["q_des"])
            note = (f"  <- MEASURED (turn_entry_phase.md), overriding the rule's "
                    f"{ph['start_frame']}")
            ph = dict(ph, start_frame=k, spread_at_start=float("nan"))
        # A MEASUREMENT knob, not an intervention: it shifts where in its own cycle a
        # clip is entered and changes no controller.  It exists because a result on one
        # entry phase is one draw from a chaotic plant -- trot_straight.md 4b is what
        # happens when that is believed -- and this rig had no way to take another.
        if args.entry_offset:
            k = (k + int(args.entry_offset)) % len(clips[sid]["q_des"])
            note += f"  (+{int(args.entry_offset)} entry offset -> {k})"
        clips[sid] = rotate_clip(clips[sid], k)
        print(f"[planner]   {sid.value:5s} entry frame {k}, "
              f"foot spread {ph['spread_at_start']*1000:.1f} mm{note}")

    initial = SkillId[args.initial]
    root = robot.data.default_root_state.clone()
    robot.write_root_state_to_sim(root)
    q_stand = robot.data.default_joint_pos.clone()
    q_first = q_stand.clone()
    q_first[:, idx_t] = torch.as_tensor(clips[initial]["q_des"][0], device=sim.device,
                                        dtype=torch.float32)

    def _hold(target, n_ctrl):
        for _ in range(n_ctrl):
            robot.set_joint_position_target(target)
            for _ in range(decim):
                robot.write_data_to_sim(); sim.step()
                robot.update(phys_dt); contacts.update(phys_dt)

    n_settle = max(int(args.settle_s / dt), 1)
    robot.write_joint_state_to_sim(q_stand, torch.zeros_like(q_stand))
    _hold(q_stand, n_settle)
    for k in range(1, n_settle + 1):
        _hold(q_stand * (1 - k / n_settle) + q_first * (k / n_settle), 1)
    print(f"[planner] handover: |v| {torch.linalg.norm(robot.data.root_lin_vel_b[0]).item():.3f} m/s, "
          f"base {robot.data.root_pos_w[0,2].item():.3f} m")

    # Geometry for the foot-placement law, measured from the robot standing on the
    # clip pose: hip-to-foot drop, and each hip's offset in the body frame.
    hip_ids, hip_names = robot.find_bodies(".*_hip")
    h_by = {n.split("_")[0]: i for i, n in zip(hip_ids, hip_names)}
    f_by = {n.split("_")[0]: i for i, n in zip(foot_ids, foot_names)}
    bp, base_p, base_q = (snap(robot.data.body_pos_w[0]), snap(robot.data.root_pos_w[0]),
                          snap(robot.data.root_quat_w[0]))
    legs = clips[initial]["leg_order"]
    lever = np.array([bp[h_by[l], 2] - bp[f_by[l], 2] for l in legs])
    hip_xy = np.array([quat_rotate_inv(base_q[None, :], (bp[h_by[l]] - base_p)[None, :])[0, :2]
                       for l in legs])
    print(f"[planner] hip->foot lever {np.round(lever,3)} m; "
          f"hip x {np.round(hip_xy[:,0],3)} y {np.round(hip_xy[:,1],3)} m")

    def foot_for(sid, clip):
        if args.foot_comp == "off":
            return None
        vy_log, wz_log = _log_motion_for(meta, clip["name"])
        s = dict(FOOT_COMP[sid])
        if args.heading != "off" and sid is not SkillId.TURN:
            s["yaw_mode"] = args.heading
            s["heading_cap_rad"] = HEADING_CAP[sid]
        return FootPlacement(t_stance_s=stance_time_s(clip["contact"], clip["fs"]),
                             lever_m=lever, hip_x_m=hip_xy[:, 0], hip_y_m=hip_xy[:, 1],
                             vy_log=vy_log, wz_log=wz_log,
                             cycle_len=len(clip["q_des"]), **s)

    def yaw_for(sid, clip):
        """The stance-leg yaw couple, or None.  TROT-only unless a cap is forced."""
        if args.yaw_moment == "off":
            return None
        cap = (args.yaw_moment_cap_nm if args.yaw_moment_cap_nm is not None
               else YAW_MOMENT_CAP_NM.get(sid, 0.0))
        if cap <= 0.0 or (args.yaw_moment_skill != "all"
                          and sid.value != args.yaw_moment_skill):
            return None
        return YawMoment(lever_m=lever, hip_x_m=hip_xy[:, 0], hip_y_m=hip_xy[:, 1],
                         bias_nm=args.yaw_moment_nm,
                         gain_nm_per_rad=(args.yaw_moment_gain
                                          if args.yaw_moment == "hold" else 0.0),
                         cap_nm=cap,
                         effort_limit_nm=hip_effort_limit_nm)

    if args.foot_comp != "off":
        print("[planner] *** FOOT PLACEMENT ON, per skill. This CLOSES A LOOP on base "
              "velocity and OVERWRITES the recording; no run here is an open-loop replay. "
              "The archive on disk is unchanged. ***")
        for sid in SUPPORTED:
            vy_log, wz_log = _log_motion_for(meta, clips[sid]["name"])
            print(f"[planner]   {sid.value:5s} {FOOT_COMP[sid]}, T_stance "
                  f"{stance_time_s(clips[sid]['contact'], clips[sid]['fs']):.3f} s, "
                  f"log v_y {vy_log:+.4f} yaw {np.degrees(wz_log):+.2f} deg/s")

    # The commanded heading is the one the robot was handed over at: "keep going the way
    # you were pointed".  A planner would supply this; on flat ground with no goal it is
    # the spawn heading, which is what makes the curvature measurable at all.
    _, _, _yaw0 = quat_to_rpy_deg(snap(robot.data.root_quat_w[0])[None, :])
    yaw_ref = float(_yaw0[0])
    if args.heading != "off":
        print(f"[planner] *** HEADING HOLD --heading {args.heading}: reference {yaw_ref:+.2f} deg "
              f"(the handover heading), caps " + ", ".join(
                  f"{k.value} {v:g}" for k, v in HEADING_CAP.items() if v) +
              ". omega_target = omega_log - psi_err / T_stance; T_stance cancels, so the "
              "heading term carries no constant. ***")

    planner = RulePlanner(cfg=DEFAULT, initial=initial)
    policies = {sid: make_policy(sid, clips=clips, foot_for=foot_for, yaw_for=yaw_for)
                for sid in SkillId}
    _ym_on = [sid for sid in SUPPORTED if getattr(policies[sid], "yaw", None) is not None]
    if _ym_on:
        _y = policies[_ym_on[0]].yaw
        print(f"[planner] *** YAW MOMENT --yaw-moment {args.yaw_moment} on {','.join(x.value for x in _ym_on)}: "
              f"feed-forward hip torque on the STANCE legs, bias {args.yaw_moment_nm:+.3f} Nm, "
              f"gain {args.yaw_moment_gain:+.2f} Nm/rad, cap {_y.cap_nm:.2f} Nm against a "
              f"{hip_effort_limit_nm:.2f} Nm hip limit. This is a DYNAMIC intervention: it "
              f"moves no foot, it adds torque to the legs already on the ground, and the "
              f"recording's own contact channel says which those are. Geometry: "
              f"{abs(_y.couple_nm_per_nm(np.zeros(4, bool))):.3f} Nm of yaw couple per Nm "
              f"with four down, {abs(_y.couple_nm_per_nm(np.array([0,1,1,0], bool))):.3f} "
              f"on a diagonal pair. Report paired with --yaw-moment off. ***")
    else:
        print("[planner] yaw moment OFF (the default): no feed-forward torque is commanded")
    playing = initial
    policies[playing].reset()

    tick_every = max(int(round((1.0 / DEFAULT.feature.TICK_HZ) / dt)), 1)
    n_steps = int(total_s(sched) / dt)
    print(f"[planner] {n_steps} control steps = {total_s(sched):.0f} s; planner ticks every "
          f"{tick_every} steps ({DEFAULT.feature.TICK_HZ:.0f} Hz)")

    yaw_any = bool(_ym_on)
    rec = {k: [] for k in ("contact_f", "root_lin_vel_b", "root_ang_vel_b", "root_pos_w",
                           "root_quat_w", "played", "planned", "foot_u", "swing")}
    rec["tau_hip"] = []
    rec["tau_ff"] = []
    events, refused, last_q, entry_note = [], {}, None, ""
    blend_left, blend_steps, blend_from = 0, 0, None
    last_u, gate_defer = None, 0
    cn = [str(x).split("_")[0] for x in getattr(contacts, "body_names", [])]
    c_by = {n: i for i, n in enumerate(cn)}
    # Sensor body order is NOT the articulation's and NOT the clip's; resolve by name
    # or not at all (harness_findings.md 5).
    sensor_col = (np.array([c_by[l] for l in legs], dtype=int)
                  if all(l in c_by for l in legs) else None)
    if args.switch_entry == "contact" and sensor_col is None:
        raise SystemExit(f"--switch-entry contact needs the contact sensor's body names "
                         f"to identify legs; it reports {cn}")
    terminated_s, term_reason = None, ""
    x_m = 0.0

    for i in range(n_steps):
        t = i * dt
        if i % tick_every == 0:
            rough, head, label, seg_i = schedule_at(sched, t)
            speed_now = float(np.hypot(*snap(robot.data.root_lin_vel_b[0])[:2])) \
                if args.speed_gate == "on" else float("nan")
            dec = planner.step(make_obs(rough), tick_every * dt, x_m=x_m,
                               heading_err_deg=head, speed_m_s=speed_now)
            if dec.active is not playing and args.switch_gate == "settled" \
                    and dec.active in SUPPORTED and last_u is not None \
                    and float(np.abs(last_u).max()) >= args.foot_clip_rad - 1e-6:
                # Do not hand a new gait a body the current one cannot hold.
                #
                # The instrumented run says the correction was ALREADY pinned at its cap
                # for three steps before the seam: WALK was mid lateral excursion, v_y
                # climbing 0.044 -> 0.132 m/s, and the switch landed on that.  A capped
                # correction is one that has stopped being a function of the velocity it
                # is supposed to answer, so it is exactly the moment not to change gait.
                #
                # No new constant: the test is the cap the foot placement already uses.
                # Unsaturated moments recur every stride, so this defers rather than
                # forbids -- which is the difference between this and the speed band.
                gate_defer += 1
            elif dec.active is not playing:
                if dec.active in SUPPORTED:
                    prev, playing = playing, dec.active
                    policies[playing].reset()
                    if args.switch_entry == "contact":
                        # Land in the new gait at a phase whose stance/swing assignment
                        # AGREES WITH THE FEET THAT ARE ACTUALLY LOADED right now, and
                        # among those frames take the closest pose.  Proprioception
                        # only -- the sensor's own contact forces, resolved to legs by
                        # name -- so the low level still sees no terrain.
                        #
                        # This is the entry criterion the other two get wrong in
                        # opposite ways: `level` lands on the clip's standable frame,
                        # which is right from a settle and arbitrary mid-stride, and
                        # `nearest` minimises the commanded step while saying nothing
                        # about which legs are in the air.  Measured, the smaller step
                        # did WORSE (TROT survived 1.48 s against 2.46 s), which is what
                        # said the seam size was not the thing that matters.
                        f_now = snap(contacts.data.net_forces_w[0].norm(dim=-1))
                        down = f_now[sensor_col] > args.contact_threshold_n
                        cl = np.asarray(clips[playing]["contact"], dtype=bool)
                        agree = (cl == down[None, :]).sum(axis=1)
                        cand = np.flatnonzero(agree == agree.max())
                        qs = policies[playing]._q
                        d = np.abs(qs[cand] - last_q[None, :]).max(axis=1)
                        k = int(cand[int(np.argmin(d))])
                        policies[playing]._start = k
                        policies[playing]._i = 0
                        entry_note = (f"phase-matched: {agree.max()}/4 legs agree, "
                                      f"frame {k}")
                    elif args.switch_entry == "nearest" and last_q is not None:
                        # Enter the new clip at the frame whose commanded pose is
                        # closest to the one being held.  Parameter-free, and it is
                        # the executor's job rather than the planner's: the planner
                        # decided WHICH gait, this decides WHERE in it to land so the
                        # PD is not asked to cross a step it cannot follow.  The
                        # alternative (`level`) enters at the clip's own standable
                        # frame, which is right from a settle and wrong mid-stride.
                        qs = policies[playing]._q
                        k = int(np.argmin(np.abs(qs - last_q[None, :]).max(axis=1)))
                        policies[playing]._start = k
                        policies[playing]._i = 0
                    blend_steps = int(round(args.switch_blend_s / dt))
                    blend_left = blend_steps
                    blend_from = last_q.copy() if last_q is not None else None
                    if blend_from is None:
                        blend_left = 0
                    q_new = policies[playing]._q[policies[playing].frame]
                    jump = float(np.abs(q_new - last_q).max()) if last_q is not None else float("nan")
                    events.append({"t_s": t, "kind": "switch", "from": prev.value,
                                   "to": playing.value, "cmd_jump_rad": jump,
                                   "reason": dec.reason, "segment": seg_i})
                    print(f"[planner] {t:6.2f}s SWITCH {prev.value} -> {playing.value} "
                          f"(commanded pose jumps {jump:.3f} rad, {entry_note}) "
                          f":: {dec.reason[:50]}")
                    entry_note = ""
                else:
                    if dec.active not in refused:
                        refused[dec.active] = 0
                        events.append({"t_s": t, "kind": "refused", "from": playing.value,
                                       "to": dec.active.value, "cmd_jump_rad": float("nan"),
                                       "reason": UNSUPPORTED_REASON.get(dec.active, ""),
                                       "segment": seg_i})
                        print(f"[planner] {t:6.2f}s REFUSED {dec.active.value}: "
                              f"{UNSUPPORTED_REASON.get(dec.active,'')[:70]} -- holding "
                              f"{playing.value}")
                    refused[dec.active] += 1

        vb = robot.data.root_lin_vel_b[0]
        wb = robot.data.root_ang_vel_b[0]
        # Heading error against the heading the robot was handed over at.  Wrapped to
        # (-pi, pi] so a run that turns past 180 deg does not command a correction the
        # long way round.  Proprioception: the base's own yaw, no terrain, no goal.
        _, _, yaw_now = quat_to_rpy_deg(snap(robot.data.root_quat_w[0])[None, :])
        psi_err = float(np.radians((float(yaw_now[0]) - yaw_ref + 180.0) % 360.0 - 180.0))
        cmd = policies[playing].act(
            BaseState(vx=float(vb[0].item()), vy=float(vb[1].item()), wz=float(wb[2].item()),
                      psi_err=psi_err if args.heading != "off" else 0.0), dt)
        q = np.asarray(cmd.q, dtype=np.float32)
        if blend_left > 0:
            # Ramp from the pose held at the switch to the new clip's stream, while the
            # new clip's PHASE keeps advancing -- the gait's rhythm is not paused, only
            # the amplitude of the handover.  This is the same shape as the cold-start
            # settle, which ramps stand -> clip over 0.5 s and is the one entry that is
            # known to work; the switch is the same problem without the ramp.
            a = 1.0 - blend_left / max(blend_steps, 1)
            q = (1.0 - a) * blend_from + a * q
            blend_left -= 1
        last_q = q
        tgt = robot.data.default_joint_pos.clone()
        tgt[:, idx_t] = torch.as_tensor(q, device=sim.device, dtype=torch.float32)
        robot.set_joint_position_target(tgt)
        # The feed-forward torque goes through a DIFFERENT call from the position
        # target, and IdealPDActuator ADDS it to the PD before clipping the sum at
        # effort_limit -- measured, not assumed: outputs/isaac_actuator_probe.json
        # q3_effort_is_additive true, q3_effort_replaces_pd false, 5 Nm in -> 5.0 Nm out.
        # It is written EVERY step, including the all-zero steps: an effort target is
        # sticky, so skipping the write when the term is off would leave the last
        # non-zero torque applied for the rest of the run.
        if yaw_any:
            eff = torch.zeros_like(tgt)
            eff[:, idx_t] = torch.as_tensor(
                np.asarray(cmd.tau_ff, dtype=np.float32)
                if cmd.tau_ff is not None else np.zeros(12, np.float32),
                device=sim.device, dtype=torch.float32)
            robot.set_joint_effort_target(eff)
        for _ in range(decim):
            robot.write_data_to_sim(); sim.step()
            robot.update(phys_dt); contacts.update(phys_dt)
        if yaw_any:
            # What the actuator ACTUALLY applied, against the limit it clips at.  A run
            # whose hips are pinned at the clip is not a run with more heading
            # authority -- it is one whose gait is being starved to pay for the couple,
            # and that has to be visible in the results row rather than inferred.
            _tau_hip = np.abs(snap(robot.data.applied_torque[0, idx_t])[0::3])
            hip_sat_steps += int((_tau_hip >= hip_effort_limit_nm - 1e-3).any())
            rec["tau_hip"].append(_tau_hip.max())

        rec["contact_f"].append(snap(contacts.data.net_forces_w[0].norm(dim=-1)))
        rec["root_lin_vel_b"].append(snap(robot.data.root_lin_vel_b[0]))
        rec["root_ang_vel_b"].append(snap(robot.data.root_ang_vel_b[0]))
        rec["root_pos_w"].append(snap(robot.data.root_pos_w[0]))
        rec["root_quat_w"].append(snap(robot.data.root_quat_w[0]))
        last_u = np.asarray(getattr(policies[playing], "last_u", np.zeros(12, np.float32)))
        rec["foot_u"].append(np.asarray(getattr(policies[playing], "last_u",
                                                 np.zeros(12, np.float32))).copy())
        rec["swing"].append(np.asarray(getattr(policies[playing], "last_swing",
                                               np.zeros(4, bool)), dtype=bool).copy())
        rec["tau_ff"].append(np.asarray(getattr(policies[playing], "last_tau_ff",
                                                np.zeros(12, np.float32))).copy())
        rec["played"].append(list(SkillId).index(playing))
        rec["planned"].append(list(SkillId).index(planner.active))
        x_m = float(robot.data.root_pos_w[0, 0].item())

        _r, _p, _ = quat_to_rpy_deg(snap(robot.data.root_quat_w[0])[None, :])
        if terminated_s is None:
            why = ("height" if robot.data.root_pos_w[0, 2].item() < args.fall_height_m
                   else "roll" if abs(float(_r[0])) > args.fall_attitude_deg
                   else "pitch" if abs(float(_p[0])) > args.fall_attitude_deg else None)
            if why:
                terminated_s, term_reason = t, why
                print(f"[planner] TERMINATED at {t:.2f} s on {why}")
                break

    if yaw_any:
        n_ctrl = max(len(rec["root_pos_w"]), 1)
        print(f"[planner] yaw moment: peak hip torque "
              f"{max(rec['tau_hip']) if rec['tau_hip'] else 0.0:.2f} Nm against a "
              f"{hip_effort_limit_nm:.2f} Nm limit; the actuator clipped a hip on "
              f"{hip_sat_steps}/{n_ctrl} control steps ({100*hip_sat_steps/n_ctrl:.1f}%)")
        for sid in SUPPORTED:
            y = getattr(policies[sid], "yaw", None)
            if y is not None:
                print(f"[planner]   {sid.value:5s} commanded |c| max {y.max_abs_nm:.3f} Nm, "
                      f"at its cap on {100*y.cap_hit_frac:.1f}% of stance-leg steps")
    a = {k: np.asarray(v) for k, v in rec.items() if len(v)}
    return summarise(args, a, dt, sched, planner, events, refused, meta, clips,
                     terminated_s, term_reason, gate_defer,
                     yaw_extra={"yaw_moment": args.yaw_moment,
                                "yaw_moment_nm": args.yaw_moment_nm,
                                "yaw_moment_gain": args.yaw_moment_gain,
                                "yaw_moment_cap_nm": (args.yaw_moment_cap_nm
                                                      if args.yaw_moment_cap_nm is not None
                                                      else float("nan")),
                                "yaw_moment_skill": args.yaw_moment_skill,
                                "hip_effort_limit_nm": hip_effort_limit_nm,
                                "hip_sat_frac": hip_sat_steps / max(len(rec["root_pos_w"]), 1),
                                "tau_hip_peak_nm": (max(rec["tau_hip"])
                                                    if rec["tau_hip"] else 0.0)})


def summarise(args, a, dt, sched, planner, events, refused, meta, clips,
              terminated_s, term_reason, gate_defer=0, yaw_extra=None) -> dict:
    """Per-segment gait numbers against the clip that was actually playing."""
    from verify_skill_replay import expected_from_meta, report
    played = a["played"]
    n = len(played)
    order = list(SkillId)
    bounds = [0] + [i for i in range(1, n) if played[i] != played[i - 1]] + [n]
    print("\n-- what was played, and was it the gait that was asked for --")
    print(f"   {'t0':>6s} {'t1':>6s} {'skill':6s} {'cycles':>7s} {'stride':>14s} "
          f"{'vx':>14s} {'yaw deg/s':>11s} {'deg/m':>7s}  verdict")
    segs = []
    for lo, hi in zip(bounds[:-1], bounds[1:]):
        if hi - lo < 8:
            continue
        sid = order[int(played[lo])]
        g = D.gait_from_force(a["contact_f"][lo:hi], dt)
        exp = expected_from_meta(meta, sid.value)
        seg = {"role": sid.value, "clip": sid.value, "t0_s": lo * dt, "t1_s": hi * dt,
               "stride_hz": g.get("stride_hz", np.nan), "duty": g.get("duty", np.nan),
               "vx_mean": float(a["root_lin_vel_b"][lo:hi, 0].mean()),
               "vy_mean": float(a["root_lin_vel_b"][lo:hi, 1].mean()),
               "yaw_rate_deg_s": float(np.degrees(a["root_ang_vel_b"][lo:hi, 2].mean()))}
        # deg of yaw per METRE travelled: the benchmark's own budget unit (0.565 deg/m).
        # Computed here rather than left to whoever reads the CSV, because "yaw rate"
        # and "curvature" are exactly the pair CLAUDE.md 6.5 warns about -- a gait that
        # slows down improves one while the other is unchanged.
        seg["curv_deg_per_m"] = (abs(seg["yaw_rate_deg_s"]) / abs(seg["vx_mean"])
                                 if abs(seg["vx_mean"]) > 1e-6 else float("nan"))
        f = D.gait_reproduced(seg, exp)
        verdict = "ok" if all(x.severity == "ok" for x in f) else f[0].severity.upper()
        cyc = (hi - lo) * dt * (exp["stride_hz"] if np.isfinite(exp["stride_hz"]) else np.nan)
        print(f"   {lo*dt:6.2f} {hi*dt:6.2f} {sid.value:6s} {cyc:7.1f} "
              f"{seg['stride_hz']:6.2f}/{exp['stride_hz']:5.2f} Hz "
              f"{seg['vx_mean']:6.3f}/{exp['vx_mean']:5.3f} {seg['yaw_rate_deg_s']:11.2f} "
              f"{seg['curv_deg_per_m']:7.2f}  "
              f"{verdict}  {f[0].symptom[:40] if f else ''}")
        seg["verdict"] = verdict
        segs.append(seg)

    switches = [e for e in events if e["kind"] == "switch"]
    gaps = np.diff([e["t_s"] for e in switches]) if len(switches) > 1 else np.array([])
    print(f"\n   planner switches {planner.switches}, executed {len(switches)}, "
          f"refused-as-unsupported {sum(refused.values())} ticks over "
          f"{len(refused)} skill(s)")
    if gaps.size:
        print(f"   time between executed switches: min {gaps.min():.2f} s, median "
              f"{np.median(gaps):.2f} s  (MIN_HOLD_S is {DEFAULT.switch.MIN_HOLD_S:.2f})")
    jumps = ", ".join("{}->{} {:.3f}".format(e["from"], e["to"], e["cmd_jump_rad"])
                      for e in switches)
    print(f"   commanded pose jump at a switch: {jumps or '(none)'}")

    row = {"schedule": args.schedule, "run_utc": _RUN_UTC, "argv": " ".join(sys.argv[1:]),
           "foot_comp": args.foot_comp, "initial": args.initial,
           "switch_entry": args.switch_entry, "switch_blend_s": args.switch_blend_s,
           "speed_gate": args.speed_gate, "speed_refusals": planner.speed_refusals,
           "switch_gate": args.switch_gate, "gate_deferrals": gate_defer,
           "heading": args.heading, "entry_turn": args.entry_turn,
           "planner_switches": planner.switches, "executed_switches": len(switches),
           "refused_ticks": sum(refused.values()),
           "refused_skills": "|".join(sorted(s.value for s in refused)),
           "segments": len(segs), "n_steps": n, "dt": dt,
           "seg_ok": sum(1 for s in segs if s["verdict"] == "ok"),
           "seg_not_ok": sum(1 for s in segs if s["verdict"] != "ok"),
           "min_switch_gap_s": float(gaps.min()) if gaps.size else "",
           "max_cmd_jump_rad": max((e["cmd_jump_rad"] for e in switches), default=""),
           "terminated_s": terminated_s if terminated_s is not None else "",
           "term_reason": term_reason,
           "pairs": "|".join(f"{e['from']}>{e['to']}" for e in switches)}
    # The longest segment's gait numbers, promoted into the row.  A --schedule hold run
    # has exactly one, and every A/B in outputs/trot_yaw_moment.md is a comparison of
    # these four columns -- reading them back out of a trace for each comparison is how
    # the curvature in one table stopped matching the curvature in another.
    if segs:
        _m = max(segs, key=lambda s_: s_["t1_s"] - s_["t0_s"])
        row.update({"main_skill": _m["role"], "curv_deg_per_m": _m["curv_deg_per_m"],
                    "yaw_rate_deg_s": _m["yaw_rate_deg_s"], "vx_mean": _m["vx_mean"],
                    "vy_mean": _m["vy_mean"], "stride_hz": _m["stride_hz"],
                    "duty": _m["duty"], "main_verdict": _m["verdict"]})
    # Every intervention this run had on, stamped into the row.  A results file whose
    # rows do not say which arm they are is a results file that gets read wrong once.
    if yaw_extra:
        row.update(yaw_extra)
    if segs:
        _m = max(segs, key=lambda s_: s_["t1_s"] - s_["t0_s"])
        print(f"\n   {_m['role']} curvature {_m['curv_deg_per_m']:.2f} deg/m "
              f"(benchmark budget 0.565); yaw {_m['yaw_rate_deg_s']:+.2f} deg/s, "
              f"vx {_m['vx_mean']:.3f} m/s, stride {_m['stride_hz']:.2f} Hz, "
              f"survived {terminated_s if terminated_s is not None else 'the whole run'}")
    if args.results_csv:
        report([row], args.results_csv)
    if args.trace_npz:
        Path(args.trace_npz).parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(args.trace_npz, dt=dt, skill_order=np.array([s.value for s in SkillId]),
                            **a)
        print(f"[planner] trace -> {args.trace_npz}")
    return row


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--schedule", choices=sorted(SCHEDULES), default="pairs")
    ap.add_argument("--initial", choices=[s.value for s in SkillId], default="WALK")
    ap.add_argument("--rate", choices=("hi", "lo"), default="lo")
    ap.add_argument("--foot-comp", choices=("off", "on"), default="on")
    ap.add_argument("--contact-threshold-n", type=float, default=30.0)
    ap.add_argument("--switch-blend-s", type=float, default=0.0,
                    help="ramp the commanded pose from the one held at the switch to the "
                         "new clip's stream over this long, with the clip's phase still "
                         "advancing. 0 (default) is the sharp seam the harness measured.")
    ap.add_argument("--heading", choices=("off", "heading", "heading-only"), default="off",
                    help="hold the handover heading with differential lateral foot "
                         "placement. heading: the full substitution omega_target = "
                         "omega_log - psi_err/T_stance, which keeps the yaw-RATE term. "
                         "heading-only: the heading half alone, because every rate loop "
                         "measured made heading worse. Caps are per skill, from the "
                         "open-loop steering probe.")
    # ----------------------------------------------------------------- yaw moment
    # The --flag contract this project uses for an intervention: default OFF, a banner
    # when it is on, its settings stamped into every results row, and an on/off pair
    # required before anything is reported.  outputs/interventions.md is the register.
    ap.add_argument("--yaw-moment", choices=("off", "probe", "hold"), default="off",
                    help="off (default): nothing is commanded but position, and the run "
                         "is bit-identical to one from before this flag existed. probe: a "
                         "CONSTANT feed-forward hip torque on the stance legs, open loop, "
                         "for measuring the actuator's gain the way "
                         "outputs/heading_candidates.md 2 measured foot placement's -- "
                         "amplitude from --yaw-moment-nm, no feedback of any kind. hold: "
                         "close the loop, c = --yaw-moment-gain * heading error, on top of "
                         "any --yaw-moment-nm bias. This is a DYNAMIC actuator: it moves no "
                         "foot and its bound is the friction cone and the hip's effort "
                         "limit, not the trot's footfall margin, which is what every route "
                         "in outputs/trot_straight.md ran out of.")
    ap.add_argument("--yaw-moment-nm", type=float, default=0.0,
                    help="constant hip torque amplitude, N.m. Positive is +front/-rear, "
                         "which the geometry says gives a NEGATIVE yaw moment -- a sign "
                         "derived in sim/yawmoment.py and deliberately not trusted: run "
                         "both signs. Not clipped by anything but --yaw-moment-cap-nm, "
                         "because the bias IS the amplitude being measured.")
    ap.add_argument("--yaw-moment-gain", type=float, default=0.0,
                    help="N.m of hip torque per radian of heading error, for --yaw-moment "
                         "hold. Set it from the probe's measured gain, not by feel.")
    ap.add_argument("--yaw-moment-cap-nm", type=float, default=None,
                    help="magnitude bound on the commanded torque. Default: the per-skill "
                         "YAW_MOMENT_CAP_NM table (all zero, i.e. OFF, until the probe "
                         "fills it in). Refused above half the hip's effort limit -- the "
                         "PD that plays the clip needs that headroom, and the actuator "
                         "clips the sum silently.")
    ap.add_argument("--yaw-moment-skill", default="TROT",
                    help="which skill carries the term (default TROT, the only one whose "
                         "heading authority is spent; WALK already meets the budget). "
                         "'all' puts it on every supported skill.")
    ap.add_argument("--entry-offset", type=int, default=0,
                    help="shift EVERY clip's entry frame by this many frames from whatever "
                         "the rule chose. 0 (default) is the rule's own answer and every "
                         "earlier result. A measurement knob: one entry phase is one draw "
                         "from a chaotic plant, and a claim that only holds at one phase is "
                         "not a claim (outputs/trot_straight.md 4b).")
    ap.add_argument("--entry-turn", choices=("rule", "measured"), default="rule",
                    help="where a TURN replay starts in its cycle. rule (default, and what "
                         "every earlier result used): verify_skill_replay.level_start, the "
                         "most-coplanar frame. measured: skill.ENTRY_FRAME_TURN from the "
                         "config, frame 6, which outputs/turn_entry_phase.md found is the "
                         "only phase of 45 that turns 9/9 in both compensator arms. The "
                         "rule picks frame 24, inside a ten-frame band that fails in both. "
                         "Affects TURN only.")
    ap.add_argument("--switch-gate", choices=("off", "settled"), default="off",
                    help="settled: defer a switch while the foot-placement correction is "
                         "pinned at its cap, i.e. while the current gait is in an excursion "
                         "it cannot answer. Uses the existing cap as its test, adds no "
                         "constant, and defers rather than forbids.")
    ap.add_argument("--foot-clip-rad", type=float, default=0.05,
                    help="the foot-placement cap, also the --switch-gate settled test")
    ap.add_argument("--speed-gate", choices=("off", "on"), default="off",
                    help="on: the planner refuses a switch while the body's speed is "
                         "further than SPEED_MATCH_MAX from the incoming skill's measured "
                         "speed. What the band admits is the measurement.")
    ap.add_argument("--switch-entry", choices=("contact", "nearest", "level"), default="contact",
                    help="where in the new clip a mid-run switch lands. nearest (default): "
                         "contact (default): the frame whose stance/swing assignment agrees "
                         "with the feet that are actually loaded, closest pose among those. "
                         "nearest: the frame closest to the pose currently commanded, which "
                         "minimises the step the PD has to follow and says nothing about "
                         "phase. level: the clip's own most-coplanar frame, right from a "
                         "settle and arbitrary mid-stride.")
    ap.add_argument("--settle-s", type=float, default=0.5)
    ap.add_argument("--fall-height-m", type=float, default=0.15)
    ap.add_argument("--fall-attitude-deg", type=float, default=60.0)
    ap.add_argument("--results-csv", default=None)
    ap.add_argument("--trace-npz", default=None)
    ap.add_argument("--plan", action="store_true", help="print the schedule, no sim")
    ap.add_argument("--self-test", action="store_true", help="wiring only, no sim")
    try:
        from isaaclab.app import AppLauncher
        AppLauncher.add_app_launcher_args(ap)
        ap.set_defaults(device="cpu")
    except Exception:
        ap.add_argument("--headless", action="store_true")
        ap.add_argument("--device", default="cpu")
    args = ap.parse_args()

    if args.plan:
        print(plan_report(args.schedule))
        return 0
    if args.self_test:
        return self_test()
    try:
        run_isaac(args)
    finally:
        if _SIM_APP is not None:
            exc = sys.exc_info()[1]
            if exc is not None:
                import traceback
                print("[planner] EXCEPTION -- printed before the app teardown takes it:",
                      file=sys.stderr)
                traceback.print_exc(); sys.stderr.flush(); sys.stdout.flush()
            _SIM_APP.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
