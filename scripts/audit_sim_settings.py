#!/usr/bin/env python3
"""Compare every simulation setting the harness runs under against the env config.

Three defects so far -- physics rate, contact threshold, ground friction -- were the
same shape: the harness took an Isaac Lab default where the env sets something else,
and the discrepancy read back as a locomotion result.  This walks the rest of the
surface so the remaining ones are found by enumeration rather than one at a time.

Isaac Lab's own defaults are read from the live dataclasses, not from memory, and the
env's values from the upstream config's AST -- so neither side is a recollection.

    scripts/isaac_docker_run.sh scripts/audit_sim_settings.py --headless
"""
from __future__ import annotations

import argparse
import ast
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim import isaac_cfg as IC


def upstream_literals(src: Path, names) -> dict:
    """Pull `name = <literal>` out of the env source, wherever it is nested."""
    tree = ast.parse(src.read_text())
    found: dict = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            if node.targets[0].id in names:
                try:
                    found.setdefault(node.targets[0].id, ast.literal_eval(node.value))
                except (ValueError, SyntaxError):
                    pass
        if isinstance(node, ast.keyword) and node.arg in names:
            try:
                found.setdefault(node.arg, ast.literal_eval(node.value))
            except (ValueError, SyntaxError):
                pass
    return found


KEYS = {
    "dt", "decimation", "gravity", "render_interval", "solver_type",
    "max_position_iteration_count", "max_velocity_iteration_count",
    "bounce_threshold_velocity", "static_friction", "dynamic_friction",
    "restitution", "friction_combine_mode", "restitution_combine_mode",
    "friction_range", "solver_position_iteration_count",
    "solver_velocity_iteration_count", "enabled_self_collisions",
    "max_linear_velocity", "max_angular_velocity",
    "soft_joint_pos_limit_factor", "action_scale",
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--json-out", default=None)
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(ap)
    ap.set_defaults(device="cpu")
    args = ap.parse_args()

    app = AppLauncher(args).app
    try:
        import isaaclab.sim as sim_utils
        from isaaclab.sim import SimulationCfg
        # Isaac Lab 3.0 moved PhysxCfg out of isaaclab.sim; the env config imports it
        # from isaaclab_physx.physics, so follow the env rather than guess.
        from isaaclab_physx.physics import PhysxCfg

        ucfg = IC.load()
        env = upstream_literals(ucfg.source, KEYS)
        # The terrain material is built in legged_robot.py, not in the config file.
        robot_py = ucfg.source.parent / "legged_robot.py"
        if robot_py.exists():
            for k, v in upstream_literals(robot_py, KEYS).items():
                env.setdefault(k, v)

        # Isaac Lab's defaults, read live from the dataclasses.
        d_sim, d_px = SimulationCfg(), PhysxCfg()
        d_mat = sim_utils.RigidBodyMaterialCfg()
        default = {
            "dt": d_sim.dt, "gravity": d_sim.gravity, "render_interval": d_sim.render_interval,
            "solver_type": d_px.solver_type,
            "max_position_iteration_count": d_px.max_position_iteration_count,
            "max_velocity_iteration_count": d_px.max_velocity_iteration_count,
            "bounce_threshold_velocity": d_px.bounce_threshold_velocity,
            "static_friction": d_mat.static_friction, "dynamic_friction": d_mat.dynamic_friction,
            "restitution": d_mat.restitution,
            "friction_combine_mode": d_mat.friction_combine_mode,
            "restitution_combine_mode": d_mat.restitution_combine_mode,
        }

        rows, mismatches = [], []
        for key in sorted(set(env) | set(default)):
            e, d = env.get(key, "--"), default.get(key, "--")
            same = (e == d)
            if isinstance(e, (int, float)) and isinstance(d, (int, float)):
                same = abs(float(e) - float(d)) < 1e-9
            flag = "same" if same else ("not-a-sim-default" if d == "--" else "DIFFERS")
            rows.append((key, e, d, flag))
            if flag == "DIFFERS":
                mismatches.append(key)

        w = max(len(r[0]) for r in rows)
        print(f"\n{'setting'.ljust(w)}  {'env config':>26}  {'Isaac default':>16}  verdict")
        print("-" * (w + 52))
        for k, e, d, f in rows:
            print(f"{k.ljust(w)}  {str(e):>26}  {str(d):>16}  {f}")
        print(f"\n{len(mismatches)} setting(s) where the env departs from the Isaac default: "
              f"{', '.join(mismatches)}")
        print("Any of those the harness does not set explicitly is a silent revert to the default.")

        out = {"upstream": str(ucfg.source), "env": env, "isaac_default": default,
               "differs": mismatches}
        if args.json_out:
            Path(args.json_out).write_text(json.dumps(out, indent=2, default=str))
            print(f"wrote {args.json_out}")
    except BaseException:
        # app.close() TERMINATES the process, so a bare finally: app.close() swallows
        # the traceback whole and the script exits looking like a clean no-op.  Print
        # first, close second.  (Same defect as harness_findings.md 2/3.)
        import traceback
        traceback.print_exc()
        sys.stdout.flush(); sys.stderr.flush()
        app.close()
        return 1
    sys.stdout.flush()
    app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
