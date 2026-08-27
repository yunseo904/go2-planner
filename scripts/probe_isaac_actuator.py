#!/usr/bin/env python3
"""Settle, in about a minute, what the installed Isaac Lab actually permits.

    python scripts/probe_isaac_actuator.py --headless

``outputs/gain_feasibility.md`` splits its claims into two lists: what was read
out of the upstream source on a machine with no Isaac Lab, and what is asserted
from the Isaac Lab API and has to be checked.  This script checks the second
list.  Run it before anything else on day 1 -- every later decision about replay
mode hangs on the answers, and guessing them costs a day.

It spawns one Go2 on a flat floor, pokes the actuator, and prints a table.  No
clip, no terrain, no policy.

Questions
---------
1. Are the Go2 actuators explicit (Python-side PD) or implicit (PhysX-side)?
2. Does assigning ``actuator.stiffness`` change the torque the next step applies?
3. Does ``set_joint_effort_target`` add a feed-forward torque on top of the PD
   term, or replace it, or get ignored?
4. Does ``write_joint_stiffness_to_sim`` exist, and does it error on an explicit
   actuator?
5. Is the applied torque clipped at ``effort_limit`` with no torque-speed curve?

Every answer is measured from ``robot.data.applied_torque``, not inferred from
the presence of a method.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, default=ROOT / "outputs" / "isaac_actuator_probe.json")
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(ap)
    # AppLauncher owns --device on Isaac Lab 6.x and rejects a duplicate.
    # Its default is cuda:0; day 1 runs the physics on CPU, so re-assert that.
    ap.set_defaults(device="cpu")
    args = ap.parse_args()

    app = AppLauncher(args).app

    import numpy as np
    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.assets import Articulation
    from isaaclab.sim import SimulationCfg, SimulationContext

    sys.path.insert(0, str(ROOT / "extreme-parkour"))
    from sim import isaac_cfg as IC

    cfg = IC.load()
    print(f"upstream config : {cfg.source}")
    print(f"control rate    : {cfg.control_hz} Hz  (dt {cfg.sim_dt} x decimation {cfg.decimation})")
    for a in cfg.actuators:
        print(f"  {a.name:10s} {a.cls:20s} kp={a.stiffness} kd={a.damping} "
              f"effort={a.effort_limit} explicit(by class name)={a.explicit}")

    # Import the real ArticulationCfg rather than rebuilding it, so the probe
    # tests the object the training code uses.
    sys.path.insert(0, str(Path(cfg.source).parents[3]))
    from legged_gym.envs.base.legged_robot_config import UNITREE_GO2_CFG

    sim = SimulationContext(SimulationCfg(dt=cfg.sim_dt or 0.005, device=args.device))
    sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
    robot = Articulation(UNITREE_GO2_CFG.replace(prim_path="/World/Robot"))
    sim.reset()

    R: dict = {"upstream_config": str(cfg.source), "control_hz": cfg.control_hz}
    dt = cfg.sim_dt or 0.005

    def settle(steps=50):
        for _ in range(steps):
            robot.set_joint_position_target(robot.data.default_joint_pos)
            robot.write_data_to_sim()
            sim.step()
            robot.update(dt)

    # -- Q1 actuator class ------------------------------------------------
    acts = robot.actuators
    R["actuator_classes"] = {k: type(v).__name__ for k, v in acts.items()}
    R["q1_explicit"] = all("Implicit" not in type(v).__name__ for v in acts.values())
    a0 = next(iter(acts.values()))
    R["stiffness_shape"] = list(getattr(a0.stiffness, "shape", []))
    print(f"\nQ1 actuator classes : {R['actuator_classes']}  explicit={R['q1_explicit']}")
    print(f"   stiffness tensor : shape {R['stiffness_shape']}")

    # -- Q2 does a runtime gain change move the torque? --------------------
    settle()
    offset = 0.15
    target = robot.data.default_joint_pos + offset
    def torque_at(scale):
        for act in acts.values():
            act.stiffness = act.stiffness * 0 + (cfg.actuators[0].stiffness or 40.0) * scale
        robot.write_joint_state_to_sim(robot.data.default_joint_pos, torch.zeros_like(robot.data.joint_vel))
        robot.set_joint_position_target(target)
        robot.write_data_to_sim()
        sim.step()
        robot.update(dt)
        return robot.data.applied_torque[0].clone()

    t1, t2 = torque_at(1.0), torque_at(2.0)
    ratio = float((t2.abs().sum() / t1.abs().sum()).item()) if t1.abs().sum() > 0 else float("nan")
    R["q2_runtime_gain_write"] = bool(abs(ratio - 2.0) < 0.25)
    R["q2_torque_ratio_for_2x_gain"] = ratio
    print(f"\nQ2 doubling stiffness scaled |torque| by {ratio:.3f} (want ~2.0) "
          f"-> runtime gain write {'WORKS' if R['q2_runtime_gain_write'] else 'DOES NOT WORK'}")

    for act in acts.values():                       # restore
        act.stiffness = act.stiffness * 0 + (cfg.actuators[0].stiffness or 40.0)

    # -- Q3 feed-forward torque -------------------------------------------
    R["q3_has_set_joint_effort_target"] = hasattr(robot, "set_joint_effort_target")
    if R["q3_has_set_joint_effort_target"]:
        settle()
        robot.write_joint_state_to_sim(robot.data.default_joint_pos, torch.zeros_like(robot.data.joint_vel))
        robot.set_joint_position_target(target)
        robot.write_data_to_sim(); sim.step(); robot.update(dt)
        base = robot.data.applied_torque[0].clone()

        ff = torch.zeros_like(robot.data.joint_pos)
        ff[:] = 5.0
        robot.write_joint_state_to_sim(robot.data.default_joint_pos, torch.zeros_like(robot.data.joint_vel))
        robot.set_joint_position_target(target)
        robot.set_joint_effort_target(ff)
        robot.write_data_to_sim(); sim.step(); robot.update(dt)
        withff = robot.data.applied_torque[0].clone()

        delta = float((withff - base).mean().item())
        R["q3_effort_delta_for_5nm"] = delta
        R["q3_effort_is_additive"] = bool(abs(delta - 5.0) < 1.0)
        R["q3_effort_replaces_pd"] = bool(abs(float(withff.mean().item()) - 5.0) < 1.0 and not R["q3_effort_is_additive"])
        verdict = ("ADDITIVE (PD + tau_ff)" if R["q3_effort_is_additive"] else
                   "REPLACES the PD term" if R["q3_effort_replaces_pd"] else "IGNORED or clipped")
        print(f"\nQ3 +5 Nm effort target moved applied_torque by {delta:+.3f} Nm -> {verdict}")
    else:
        print("\nQ3 set_joint_effort_target ABSENT -- feed-forward torque has no path")

    # -- Q4 implicit-path writer ------------------------------------------
    R["q4_has_write_joint_stiffness_to_sim"] = hasattr(robot, "write_joint_stiffness_to_sim")
    if R["q4_has_write_joint_stiffness_to_sim"]:
        try:
            robot.write_joint_stiffness_to_sim(torch.full_like(robot.data.joint_pos, 40.0))
            R["q4_write_result"] = "accepted"
        except Exception as exc:
            R["q4_write_result"] = f"raised: {type(exc).__name__}: {exc}"
        print(f"\nQ4 write_joint_stiffness_to_sim present; on this articulation it {R['q4_write_result']}")
        if R["q1_explicit"]:
            print("   (explicit actuator: this path writes PhysX gains that the Python PD does not read; "
                  "it is NOT the way to schedule gains here)")
    else:
        print("\nQ4 write_joint_stiffness_to_sim ABSENT")

    # -- Q5 effort clip ----------------------------------------------------
    settle()
    robot.write_joint_state_to_sim(robot.data.default_joint_pos, torch.zeros_like(robot.data.joint_vel))
    robot.set_joint_position_target(robot.data.default_joint_pos + 5.0)   # absurd error -> saturate
    robot.write_data_to_sim(); sim.step(); robot.update(dt)
    sat = robot.data.applied_torque[0].abs()
    R["q5_saturated_torque_per_joint"] = [round(float(v), 3) for v in sat]
    R["q5_joint_names"] = list(robot.joint_names)
    print("\nQ5 saturated |torque| per joint (expect the configured effort_limit):")
    for n, v in zip(robot.joint_names, sat):
        print(f"   {n:20s} {float(v):8.2f} Nm")

    # Everything below has to happen BEFORE app.close(): SimulationApp.close() tears the
    # process down, so a write placed after it never runs -- the script exits 0 having
    # produced nothing. Measured on Isaac Sim 6.0.1-rc.7.
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(R, indent=2, default=str) + "\n")
    print(f"\nwrote {args.out}")

    print("\n== what this means for the replay mode ==")
    if R.get("q2_runtime_gain_write") and R.get("q3_effort_is_additive"):
        print("  TORQUE mode is available: per-step gains + feed-forward. Use it for RUN and JUMP.")
    elif R.get("q3_effort_is_additive"):
        print("  TORQUE is out; FIXED_GAIN is available (per-skill gains set before the sim starts,")
        print("  feed-forward per step). The within-skill gain schedule is lost; nothing else is.")
    elif R.get("q3_has_set_joint_effort_target"):
        print("  Feed-forward exists but is not additive. Read Q3 above before choosing a mode.")
    else:
        print("  Only POSITION is available. WALK and TROT are still faithful (kp 40 / kd 1 / tau_ff 0")
        print("  matches the config exactly); RUN and JUMP need the parameter-gait fallback (c).")

    app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
