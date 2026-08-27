#!/usr/bin/env python3
"""Run the frozen calibration probes and turn the outcome into config values.

    # on the sim machine
    python scripts/run_calibration.py --headless
    python scripts/run_calibration.py --headless --params skill.STEP_JUMP_MAX
    python scripts/run_calibration.py --headless --reps 5 --max-probes 20

    # anywhere, no Isaac Lab needed
    python scripts/run_calibration.py --plan            # what would run, and how long
    python scripts/run_calibration.py --self-test       # exercise the protocol arithmetic
    python scripts/run_calibration.py --from-results outputs/calibration_results.csv

Protocol
--------
Every level of a probe family is driven ``--reps`` times (5 by default) with the
skill that parameter belongs to.  A repeat succeeds if the robot reaches the
probe's second goal -- the one just past the obstacle -- upright and inside the
time budget.  The reported limit is the top of the unbroken run of levels that
succeeded on **every** repeat, minus one level of margin.

Why every repeat and not a majority: this number becomes a planner *guarantee*.
A limit that works four times in five is a limit the planner will walk off.

Why one probe per terrain: the probes are one obstacle each, not a staircase, so
a failure at 0.14 m cannot contaminate the 0.16 m trial (``terrain_toolkit/
calibrate.py``).  The runs are independent by construction.

Why this is not on the benchmark: CLAUDE.md section 2 forbids tuning thresholds on
performance data.  These probes share no code and no RNG with the benchmark
archive, so a value measured here cannot have leaked from the terrain it will be
evaluated on.

Status: the Isaac Lab path in this file has **not been executed** -- this machine
has no Isaac Lab.  The protocol arithmetic, the mesh conversion and the config
emission have been, via ``--self-test``.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sim import heightfield as HF
from sim import isaac_cfg as IC
from sim.replay import ReplayMode, apply_gains, default_mode_for, probe_capabilities
from terrain_toolkit.calibrate import CALIBRATION_MAP, STEP_DOWN_NOTE
from terrain_toolkit.paths import (
    CALIBRATION_NPZ,
    SKILL_CLIPS_META_JSON,
    CALIBRATION_REPORT_MD,
    CALIBRATION_RESULTS_CSV,
    SKILL_CLIPS_NPZ,
)

#: Which clip drives which parameter's sweep.  Mirrors ``CALIBRATION_MAP``.
PARAM_SKILL = {
    "skill.STEP_WALK_MAX": "WALK",
    "skill.STEP_TROT_MAX": "TROT",
    "skill.STEP_RUN_MAX": "RUN",
    "skill.STEP_JUMP_MAX": "JUMP",
    "robot.FOOT_SPAN_X": "WALK",
}

#: Seconds a repeat gets to reach goal 2 before it counts as a failure.
TIME_BUDGET_S = 20.0
#: Base below this is a fall.
FALL_HEIGHT_M = 0.15
#: Within this of the goal counts as reached.
GOAL_RADIUS_M = 0.35


def load_probes() -> dict:
    z = np.load(CALIBRATION_NPZ, allow_pickle=False)
    return {
        "hf": z["height_fields"],
        "goals": z["goals"],
        "families": [str(x) for x in z["families"]],
        "names": [str(x) for x in z["names"]],
        "params_m": z["params_m"],
        "levels": z["levels"],
        "horizontal_scale": float(z["horizontal_scale"]),
        "vertical_scale": float(z["vertical_scale"]),
        "spawn": (float(z["spawn_x"]), float(z["spawn_y"])),
    }


def planned_runs(probes: dict, params: list, reps: int, max_probes: int | None) -> list:
    """``[(parameter, skill, family, probe index, level m)]`` in run order."""
    runs = []
    for p in params:
        family = CALIBRATION_MAP[p][0]
        if family is None:
            continue
        idx = [i for i, f in enumerate(probes["families"]) if f == family]
        if max_probes:
            idx = idx[:max_probes]
        for i in idx:
            runs.append((p, PARAM_SKILL[p], family, i, float(probes["params_m"][i])))
    return runs


# --------------------------------------------------------------------------- #
# Isaac Lab  (UNVERIFIED — see the module docstring)
# --------------------------------------------------------------------------- #

# SimulationApp.close() tears the process down, so anything after it never runs.
# Both scripts originally closed inside the Isaac helper, before the metrics were
# computed and returned -- the run exited 0 having produced no verdict and no CSV.
# The app is held here and closed once, in main(), after the results are written.
_SIM_APP = None


def run_isaac(args, probes: dict, runs: list) -> list:
    from isaaclab.app import AppLauncher

    global _SIM_APP
    app_launcher = AppLauncher(args)
    simulation_app = _SIM_APP = app_launcher.app

    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.assets import Articulation
    from isaaclab.sim import SimulationCfg, SimulationContext
    from isaaclab.terrains import TerrainImporter, TerrainImporterCfg

    # The SAME articulation config the training and the replay verification use.
    # Calibrating against a differently-configured robot would measure a robot the
    # evaluation never instantiates.
    ucfg = IC.load()
    sys.path.insert(0, str(Path(ucfg.source).parents[3]))
    from legged_gym.envs.base.legged_robot_config import UNITREE_GO2_CFG

    sys.path.insert(0, str(ROOT / "scripts"))
    from verify_skill_replay import load_clip

    import json as _json
    meta = _json.loads(SKILL_CLIPS_META_JSON.read_text())
    clips = {name: load_clip(name, args.rate) for name in sorted({r[1] for r in runs})}
    # Same defect as verify_skill_replay.py: one sim.step() per clip sample runs PhysX
    # at ~50 Hz instead of the config's 200 Hz and the robot collapses on flat ground.
    # See that file for the measured A/B.
    #
    # A SimulationContext takes its dt once, but the clips do not share a rate (49.37 to
    # 50.58 Hz -- each cyclic clip carries an integer number of samples per gait cycle,
    # so its rate lands near 50 rather than on it). So the physics step is the config's
    # sim_dt and the substep count is resolved PER CLIP against it. Residual control-rate
    # error is under 1.3% per clip; it does not accumulate across episodes because each
    # episode restarts, and it is far below the pass/fail granularity of a distance probe.
    phys_dt = float(ucfg.sim_dt)
    decim_for = {name: max(1, int(round((1.0 / c["fs"]) / phys_dt))) for name, c in clips.items()}
    sim = SimulationContext(SimulationCfg(dt=phys_dt, device=args.device))
    for name, c in clips.items():
        d = decim_for[name]
        print(f"[cal] {name}: clip {c['fs']:.2f} Hz -> {d} x {phys_dt:.4f} s "
              f"= {1.0/(d*phys_dt):.2f} Hz control ({100*((d*phys_dt)*c['fs']-1):+.2f}% rate error)")

    sign = np.ones(12, dtype=np.float32)
    if args.hip_sign == "flip":
        sign[0::3] = -1.0
        print("[cal] hip columns negated (--hip-sign flip)")

    rows = []
    for (param, skill, family, pi, level) in runs:
        clip = clips[skill]
        decim = decim_for[skill]
        dt = decim * phys_dt                       # control period actually stepped
        hf = probes["hf"][pi]
        verts, faces = HF.to_trimesh(hf, probes["horizontal_scale"], probes["vertical_scale"])
        importer = TerrainImporter(TerrainImporterCfg(prim_path="/World/ground", terrain_type="plane"))
        try:
            import trimesh
            importer.import_mesh("probe", trimesh.Trimesh(vertices=verts, faces=faces))
        except Exception as exc:                                   # pragma: no cover
            raise SystemExit(f"could not import the probe mesh: {exc}")

        robot_cfg = UNITREE_GO2_CFG.replace(prim_path="/World/Robot")
        requested = (ReplayMode(args.mode) if args.mode
                     else default_mode_for(meta["clips"][skill]["gains"]["position_controlled"]))
        if requested is ReplayMode.FIXED_GAIN:
            for act in robot_cfg.actuators.values():
                act.stiffness = float(np.median(clip["kp"]))
                act.damping = float(np.median(clip["kd"]))
        robot = Articulation(robot_cfg)
        sim.reset()
        cap = probe_capabilities(robot, ucfg.all_explicit)
        mode = cap.resolve(requested)
        want = [f"{leg}_{j}_joint" for leg in clip["leg_order"] for j in clip["joint_order"]]
        idx_t = torch.as_tensor([robot.joint_names.index(j) for j in want],
                                device=sim.device, dtype=torch.long)

        sx, sy = probes["spawn"]
        sz = HF.height_at(hf, probes["horizontal_scale"], probes["vertical_scale"], sx, sy) + args.spawn_clearance_m
        goal = probes["goals"][pi][1]

        for rep in range(args.reps):
            root = robot.data.default_root_state.clone()
            root[:, 0:3] = torch.as_tensor([sx, sy, sz], device=sim.device)
            robot.write_root_state_to_sim(root)
            robot.reset()

            n, reached, fell, t_s = clip["q_des"].shape[0], False, False, 0.0
            step, saturated = 0, 0
            _, _, lim_l = ucfg.ordered(clip["leg_order"], clip["joint_order"])
            lim = np.asarray([l if l is not None else np.inf for l in lim_l], dtype=float)
            while t_s < args.time_budget_s and not reached and not fell:
                k = step % n
                if mode is ReplayMode.TORQUE:
                    apply_gains(robot, idx_t, clip["kp"][k], clip["kd"][k], cap)
                tgt = robot.data.default_joint_pos.clone()
                tgt[:, idx_t] = torch.as_tensor(clip["q_des"][k] * sign, device=sim.device)
                robot.set_joint_position_target(tgt)
                if mode.needs_effort:
                    eff = torch.zeros_like(tgt)
                    eff[:, idx_t] = torch.as_tensor(clip["tau_ff"][k] * sign, device=sim.device)
                    robot.set_joint_effort_target(eff)
                for _ in range(decim):
                    robot.write_data_to_sim()
                    sim.step()
                    robot.update(phys_dt)
                pos = robot.data.root_pos_w[0].cpu().numpy()
                saturated += int((np.abs(robot.data.applied_torque[0, idx_t].cpu().numpy()) >= lim - 1e-3).any())
                reached = bool(np.hypot(pos[0] - goal[0], pos[1] - goal[1]) < args.goal_radius_m)
                fell = bool(pos[2] < args.fall_height_m)
                step += 1
                t_s += dt
                if clip["kind"] == "oneshot" and step >= n:
                    # a one-shot clip is played once; hold the last pose out the clock
                    step = n - 1

            rows.append({"parameter": param, "skill": skill, "family": family,
                         "probe": probes["names"][pi], "level_m": level, "rep": rep,
                         "passed": int(reached), "fell": int(fell), "t_s": round(t_s, 3),
                         "x_final_m": round(float(pos[0]), 3), "mode": mode.value,
                         "hip_sign": args.hip_sign,
                         "torque_saturated_frac": round(saturated / max(step, 1), 4)})
            print(f"[cal] {param:22s} {skill:5s} {probes['names'][pi]:16s} rep {rep+1}/{args.reps} "
                  f"{'PASS' if reached else 'fail'} x={pos[0]:.2f} t={t_s:.1f}s")

    return rows


# --------------------------------------------------------------------------- #
# Results -> thresholds -> config
# --------------------------------------------------------------------------- #

def thresholds_from_rows(rows: list) -> list:
    by_param: dict = {}
    for r in rows:
        by_param.setdefault(r["parameter"], []).append(r)
    out = []
    for param, rs in sorted(by_param.items()):
        levels = sorted({float(r["level_m"]) for r in rs})
        reps = max(int(r["rep"]) for r in rs) + 1
        mat = np.zeros((len(levels), reps), dtype=bool)
        for r in rs:
            mat[levels.index(float(r["level_m"])), int(r["rep"])] = bool(int(r["passed"]))
        out.append(HF.threshold_from_matrix(levels, mat, param, rs[0]["family"], rs[0]["skill"]))
    return out


def write_report(ths: list, rows: list, args) -> None:
    reps = max((t.n_reps for t in ths), default=0)
    L = ["# Calibration results", "",
         f"`{CALIBRATION_NPZ.name}` probes, {reps} repeats per level.",
         "Protocol: top of the unbroken all-pass run, minus one level.", "",
         "| parameter | skill | family | value | highest all-pass | first failure | monotone |",
         "|---|---|---|---|---|---|---|"]
    for t in ths:
        v = "—" if t.value_m is None else f"**{t.value_m:.3f} m**"
        h = "—" if t.highest_all_pass_m is None else f"{t.highest_all_pass_m:.2f} m"
        f = "—" if t.first_failure_m is None else f"{t.first_failure_m:.2f} m"
        L.append(f"| `{t.parameter}` | {t.skill} | {t.family} | {v} | {h} | {f} | "
                 f"{'yes' if t.monotone else '**no**'} |")
    L += ["", "## Raw pass matrix", ""]
    for t in ths:
        L += [f"### `{t.parameter}` ({t.skill} on {t.family})", "",
              "| level | passed / reps |", "|---|---|"]
        L += [f"| {lv:.2f} m | {np_:d} / {nr:d} |" for lv, np_, nr in t.raw]
        if t.note:
            L += ["", f"> {t.note}"]
        L.append("")
    uncovered = [k for k, v in CALIBRATION_MAP.items() if v[0] is None]
    L += ["## Not covered by these probes", "",
          "These stay `CALIBRATION_NEEDED` after this run — they need a ramp probe and a "
          "roughness probe that are deliberately not in the frozen archive:", ""]
    L += [f"- `{k}` — {CALIBRATION_MAP[k][1]}" for k in uncovered]
    L += ["", "## step_down", "", STEP_DOWN_NOTE, "",
          "## Config patch", "", "```python", HF.config_block(ths), "```", ""]
    CALIBRATION_REPORT_MD.parent.mkdir(parents=True, exist_ok=True)
    CALIBRATION_REPORT_MD.write_text("\n".join(L))
    print(f"[cal] wrote {CALIBRATION_REPORT_MD}")


def self_test() -> int:
    fails = 0
    print("== mesh conversion ==")
    probes = load_probes()
    for name in ("step_up_0p10", "gap_0p30"):
        i = probes["names"].index(name)
        hf = probes["hf"][i]
        v, f = HF.to_trimesh(hf, probes["horizontal_scale"], probes["vertical_scale"])
        nx, ny = hf.shape
        ok = v.shape == (nx * ny, 3) and f.shape == (2 * (nx - 1) * (ny - 1), 3)
        ok &= np.isclose(v[:, 2].max(), hf.max() * probes["vertical_scale"])
        ok &= np.isclose(v[:, 2].min(), hf.min() * probes["vertical_scale"])
        fails += 0 if ok else 1
        print(f"  {'ok  ' if ok else 'FAIL'} {name:14s} {v.shape[0]} verts, {f.shape[0]} faces, "
              f"z in [{v[:,2].min():+.3f}, {v[:,2].max():+.3f}] m")
    i = probes["names"].index("step_up_0p10")
    h_before = HF.height_at(probes["hf"][i], probes["horizontal_scale"], probes["vertical_scale"], 3.0, 2.0)
    h_after = HF.height_at(probes["hf"][i], probes["horizontal_scale"], probes["vertical_scale"], 5.0, 2.0)
    ok = np.isclose(h_after - h_before, 0.10, atol=1e-6)
    fails += 0 if ok else 1
    print(f"  {'ok  ' if ok else 'FAIL'} step_up_0p10 rise across the obstacle = {h_after-h_before:.3f} m")

    print("\n== protocol ==")
    levels = [0.02 * k for k in range(1, 8)]
    cases = [
        ("clean cut after 0.08", [[1]*5, [1]*5, [1]*5, [1]*5, [0]*5, [0]*5, [0]*5], 0.06, 0.08, True),
        ("one flaky repeat at 0.08", [[1]*5, [1]*5, [1]*5, [1,1,1,1,0], [0]*5, [0]*5, [0]*5], 0.04, 0.06, True),
        ("non-monotone", [[1]*5, [1]*5, [0]*5, [1]*5, [0]*5, [0]*5, [0]*5], 0.02, 0.08, False),
        ("fails at the bottom", [[0]*5]*7, None, None, True),
        ("clears everything", [[1]*5]*7, 0.12, 0.14, True),
    ]
    for label, mat, want_v, want_h, want_mono in cases:
        t = HF.threshold_from_matrix(levels, np.asarray(mat, dtype=bool), "skill.X", "step_up", "WALK")
        ok = (t.value_m is None and want_v is None or
              (t.value_m is not None and want_v is not None and np.isclose(t.value_m, want_v)))
        ok &= (t.highest_all_pass_m is None and want_h is None or
               (t.highest_all_pass_m is not None and want_h is not None and np.isclose(t.highest_all_pass_m, want_h)))
        ok &= t.monotone == want_mono
        fails += 0 if ok else 1
        v = "None" if t.value_m is None else f"{t.value_m:.2f}"
        h = "None" if t.highest_all_pass_m is None else f"{t.highest_all_pass_m:.2f}"
        print(f"  {'ok  ' if ok else 'FAIL'} {label:26s} value={v:>5s} (want {want_v}) "
              f"highest={h:>5s} monotone={t.monotone}")

    print("\n== config emission ==")
    t = HF.threshold_from_matrix(levels, np.asarray(cases[0][1], dtype=bool),
                                 "skill.STEP_WALK_MAX", "step_up", "WALK")
    block = HF.config_block([t])
    ok = "STEP_WALK_MAX: float = 0.060" in block and "Provenance.MEASURED" in block
    fails += 0 if ok else 1
    print(f"  {'ok  ' if ok else 'FAIL'} emits a value and a MEASURED provenance line")
    t2 = HF.threshold_from_matrix(levels, np.asarray(cases[2][1], dtype=bool),
                                  "skill.STEP_WALK_MAX", "step_up", "WALK")
    ok = "CALIBRATION_NEEDED" in HF.config_block([t2])
    fails += 0 if ok else 1
    print(f"  {'ok  ' if ok else 'FAIL'} a non-monotone sweep is NOT promoted to MEASURED")

    print(f"\nself-test: {'PASS' if fails == 0 else f'{fails} FAILURES'}")
    return 0 if fails == 0 else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--params", nargs="*", default=None, help="subset of the calibration map")
    ap.add_argument("--reps", type=int, default=5)
    ap.add_argument("--rate", choices=("hi", "lo"), default="lo")
    ap.add_argument("--max-probes", type=int, default=None, help="cap levels per family (smoke runs)")
    ap.add_argument("--mode", choices=[m.value for m in ReplayMode], default=None,
                    help="replay mode; defaults per clip. Must match what verify_skill_replay.py "
                         "validated, or the calibration measures a different controller.")
    ap.add_argument("--hip-sign", choices=("keep", "flip"), default="keep",
                    help="must match the convention settled in step 2 of outputs/server_day1.md")
    ap.add_argument("--time-budget-s", type=float, default=TIME_BUDGET_S)
    ap.add_argument("--goal-radius-m", type=float, default=GOAL_RADIUS_M)
    ap.add_argument("--fall-height-m", type=float, default=FALL_HEIGHT_M)
    ap.add_argument("--spawn-clearance-m", type=float, default=0.40)
    ap.add_argument("--plan", action="store_true", help="print the run plan and exit")
    ap.add_argument("--self-test", action="store_true", help="no Isaac Lab needed")
    ap.add_argument("--from-results", type=Path, default=None,
                    help="recompute thresholds and the config patch from an existing CSV")
    try:
        from isaaclab.app import AppLauncher
        AppLauncher.add_app_launcher_args(ap)
        # AppLauncher owns --device on Isaac Lab 6.x and rejects a duplicate.
        # Its default is cuda:0; day 1 runs the physics on CPU, so re-assert that.
        ap.set_defaults(device="cpu")
    except Exception:
        ap.add_argument("--headless", action="store_true")
        ap.add_argument("--device", default="cpu")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    if args.from_results:
        with open(args.from_results) as fh:
            rows = list(csv.DictReader(fh))
        ths = thresholds_from_rows(rows)
        write_report(ths, rows, args)
        print(HF.config_block(ths))
        return 0

    probes = load_probes()
    params = args.params or [k for k, v in CALIBRATION_MAP.items() if v[0] is not None]
    unknown = [p for p in params if p not in CALIBRATION_MAP]
    if unknown:
        raise SystemExit(f"unknown parameters: {unknown}")
    runs = planned_runs(probes, params, args.reps, args.max_probes)

    if args.plan:
        print(f"{len(runs)} probe configurations x {args.reps} repeats = {len(runs)*args.reps} episodes")
        print(f"time budget {args.time_budget_s:.0f} s each -> "
              f"{len(runs)*args.reps*args.time_budget_s/3600:.1f} h of simulated time (wall clock is lower)")
        print()
        for p in params:
            fam = CALIBRATION_MAP[p][0]
            lv = [r[4] for r in runs if r[0] == p]
            print(f"  {p:22s} {PARAM_SKILL[p]:5s} on {fam:9s} "
                  f"{len(lv)} levels {min(lv):.2f}-{max(lv):.2f} m -- {CALIBRATION_MAP[p][1]}")
        print()
        for p, v in CALIBRATION_MAP.items():
            if v[0] is None:
                print(f"  {p:22s} NOT COVERED -- {v[2]}")
        if not SKILL_CLIPS_NPZ.is_file():
            print(f"\n!! no clip archive at {SKILL_CLIPS_NPZ}; run scripts/extract_skill_clips.py first")
        return 0

    if not SKILL_CLIPS_NPZ.is_file():
        raise SystemExit(f"no clip archive at {SKILL_CLIPS_NPZ}; run scripts/extract_skill_clips.py")

    try:
        rows = run_isaac(args, probes, runs)
        CALIBRATION_RESULTS_CSV.parent.mkdir(parents=True, exist_ok=True)
        with open(CALIBRATION_RESULTS_CSV, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)
        print(f"[cal] wrote {CALIBRATION_RESULTS_CSV}")
        ths = thresholds_from_rows(rows)
        write_report(ths, rows, args)
        print(HF.config_block(ths))
    finally:
        if _SIM_APP is not None:
            _SIM_APP.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
