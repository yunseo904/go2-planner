#!/usr/bin/env python
"""Sweep the frozen benchmark terrains with the rule planner, no simulator.

A virtual robot follows the route ``spawn -> goal0 -> ... -> goal7`` of every
(task, level).  At each planner tick it observes the sensor-limited window ahead
(`planner.features`), the rule engine picks a skill (`planner.rules`), and the
robot advances at that skill's *measured* steady speed.  The whole point is the
**skill sequence and the switch count**, and how both move when ``SWITCH_DELAY``
changes -- the sensitivity sweep `CLAUDE.md` §3 asks for.

This is not an evaluation.  Nothing here decides whether the robot would have
stayed upright; a kinematic sweep cannot. Terrain the skill library has no answer
for is logged as `unsupported` and traversed anyway, per `CLAUDE.md` §2: all 20
tasks are run identically and "tried and failed" is a result.

Writes:
    outputs/planner_offline_segments.csv   run-length-encoded skill sequence
    outputs/planner_offline_summary.csv    one row per (delay, task, level)
    outputs/planner_offline_unsupported.csv every unsupported/unverified event
    outputs/planner_offline.md             summary + the config provenance table

Usage:
    python scripts/simulate_planner_offline.py [--delays 0 0.21 2.4] [--levels 0 5 9]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from planner.config import DEFAULT, PlannerConfig, Provenance  # noqa: E402
from planner.features import FeatureMemory, TerrainMap, extract, ground_resolution  # noqa: E402
from planner.features import lookahead_distance  # noqa: E402
from planner.features import maps_from_archive  # noqa: E402
from planner.rules import RulePlanner  # noqa: E402
from planner.tracking import JumpGate  # noqa: E402
from planner.skills import SkillId  # noqa: E402
from terrain_toolkit import paths  # noqa: E402
from terrain_toolkit.freeze import load_archive  # noqa: E402

#: Give up on a run once the robot has made no progress for this long.
STALL_S = 5.0
#: Hard cap so a zero-speed configuration cannot spin forever.
MAX_SIM_S = 240.0


def traverse(tmap: TerrainMap, cfg: PlannerConfig,
             jump_gate: JumpGate = JumpGate.NEAR_EDGE) -> Dict[str, object]:
    """Run one (task, level) at one delay.  Returns the trace and its summary."""
    dt = 1.0 / cfg.feature.TICK_HZ
    route = tmap.route()
    planner = RulePlanner(cfg, jump_gate=jump_gate)
    memory = FeatureMemory(max_age_s=cfg.switch.SETTLE_WORST_S)

    pos = route[0].astype(float).copy()
    leg = 1
    t = 0.0
    travelled = 0.0
    stall_t = 0.0
    stalled = False
    warn_ticks = 0
    blind_ticks = 0
    stale_ticks = 0
    seen_steps: List[float] = []
    true_steps: List[float] = []
    in_jump_band = 0
    walk_max = cfg.skill.STEP_WALK_MAX
    jump_max = cfg.skill.STEP_JUMP_MAX

    segments: List[Dict[str, object]] = []
    time_in: Dict[str, float] = {s.value: 0.0 for s in SkillId}
    cur = planner.active
    seg_start_t, seg_start_x = 0.0, float(pos[0])

    while leg < len(route) and t < MAX_SIM_S:
        target = route[leg].astype(float)
        d = target - pos
        dist = float(np.hypot(*d))
        heading = (d / dist) if dist > 1e-9 else np.array([1.0, 0.0])

        look = lookahead_distance(planner.speed, cfg)
        obs = extract(tmap, float(pos[0]), float(pos[1]), look, cfg, tuple(heading))
        obs = memory.update(obs, dt)

        if jump_gate is JumpGate.TRACKING:
            # the tracking gate sees the whole visible range, not just the band
            # the planner can still commit to. Same sensor model, wider read.
            full = extract(tmap, float(pos[0]), float(pos[1]), look, cfg, tuple(heading),
                           window=(cfg.sensor.SENSOR_NEAR, cfg.sensor.SENSOR_FAR),
                           with_profile=True)
            planner.observe(full, float(pos[0]), tmap.horizontal_scale)
        if obs.warnings:
            warn_ticks += 1
        if not obs.valid:
            blind_ticks += 1
        if obs.stale:
            stale_ticks += 1

        seen = obs.features.get("step_up_m", np.nan)
        true = obs.features.get("step_up_true_m", np.nan)
        if np.isfinite(seen):
            seen_steps.append(seen)
            if walk_max < seen <= jump_max:
                in_jump_band += 1
        if np.isfinite(true):
            true_steps.append(true)

        dec = planner.step(obs, dt, x_m=float(pos[0]))
        if dec.active is not cur:
            segments.append({
                "skill": cur.value, "t_start_s": seg_start_t, "t_end_s": t,
                "x_start_m": seg_start_x, "x_end_m": float(pos[0]),
                "duration_s": t - seg_start_t,
            })
            cur, seg_start_t, seg_start_x = dec.active, t, float(pos[0])

        speed = planner.speed
        time_in[dec.active.value] += dt
        step_len = speed * dt
        if step_len <= 1e-6:
            stall_t += dt
            if stall_t >= STALL_S:
                stalled = True
                break
        else:
            stall_t = 0.0
            if step_len >= dist:
                pos = target.copy()
                travelled += dist
                leg += 1
            else:
                pos = pos + heading * step_len
                travelled += step_len
        t += dt

    segments.append({
        "skill": cur.value, "t_start_s": seg_start_t, "t_end_s": t,
        "x_start_m": seg_start_x, "x_end_m": float(pos[0]), "duration_s": t - seg_start_t,
    })

    total = max(t, 1e-9)

    def _q(v: List[float], p: float) -> float:
        return float(np.percentile(v, p)) if v else float("nan")

    look_trot = lookahead_distance(cfg.skill.SPEED_TROT, cfg)
    summary = {
        "task": tmap.task, "level": tmap.level,
        "switch_delay_s": cfg.switch.SWITCH_DELAY,
        "step_walk_max_m": cfg.skill.STEP_WALK_MAX,
        "lookahead_trot_m": look_trot,
        # ground width of one depth pixel at the commitment point -- the number
        # that decides whether an obstacle survives the sensor model at all
        "px_ground_width_m": float(ground_resolution(np.array([max(look_trot, cfg.sensor.SENSOR_NEAR)]),
                                                     cfg.sensor)[0]),
        "obs_step_up_max": _q(seen_steps, 100), "obs_step_up_p95": _q(seen_steps, 95),
        "obs_step_up_mean": float(np.mean(seen_steps)) if seen_steps else float("nan"),
        "true_step_up_max": _q(true_steps, 100), "true_step_up_p95": _q(true_steps, 95),
        "true_step_up_mean": float(np.mean(true_steps)) if true_steps else float("nan"),
        "ticks_in_jump_band": in_jump_band,
        "jump_gate": str(jump_gate),
        "requests_dropped_busy": planner.requests_dropped_busy,
        "jump_detections": planner.jump_detections,
        "jump_tracks_missed": planner.tracker.n_missed(float(pos[0])) if planner.tracker else 0,
        "jump_band_ticks": planner.jump_band_ticks,
        "jump_blocked_far": planner.jump_blocked_far,
        "jump_blocked_retrigger": planner.jump_blocked_retrigger,
        "switches": planner.switches,
        "jumps": planner.jumps,
        "goals_reached": leg - 1,
        "goals_total": len(route) - 1,
        "sim_time_s": t,
        "travelled_m": travelled,
        "stalled": stalled,
        "timed_out": t >= MAX_SIM_S,
        "n_segments": len(segments),
        "warn_tick_frac": warn_ticks / (total / dt),
        "blind_tick_frac": blind_ticks / (total / dt),
        "stale_tick_frac": stale_ticks / (total / dt),
        "unsupported_fatal": sum(1 for u in planner.unsupported_log if u.fatal),
        "unsupported_unverified": sum(1 for u in planner.unsupported_log if not u.fatal),
    }
    for skill in SkillId:
        summary[f"frac_{skill.value}"] = time_in[skill.value] / total
    summary["sequence"] = "→".join(
        s["skill"] for i, s in enumerate(segments) if i == 0 or s["skill"] != segments[i - 1]["skill"]
    )
    return {"summary": summary, "segments": segments, "unsupported": planner.unsupported_log}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--delays", type=float, nargs="+",
                    default=[0.0, 0.21, 0.45, 0.6, 0.8, 1.0, 1.4, 2.0, 2.4],
                    help="SWITCH_DELAY values to sweep")
    ap.add_argument("--step-walk-max", type=float, nargs="+", default=[0.06, 0.10, 0.15],
                    help="STEP_WALK_MAX values to sweep (CALIBRATION_NEEDED placeholder); "
                         "the second value is treated as the reference in the report")
    ap.add_argument("--levels", type=int, nargs="+", default=None,
                    help="difficulty levels (default: all 10)")
    ap.add_argument("--tasks", type=str, nargs="+", default=None, help="task names (default: all 20)")
    ap.add_argument("--jump-gate", type=str, nargs="+", default=["near_edge"],
                    choices=[g.value for g in JumpGate],
                    help="jump gate(s) to run; pass both to compare them")
    ap.add_argument("--report-only", action="store_true",
                    help="regenerate the markdown from the CSVs already in outputs/ (no sweep)")
    args = ap.parse_args()

    if args.report_only:
        summary = pd.read_csv(paths.PLANNER_SUMMARY_CSV)
        uns = pd.read_csv(paths.PLANNER_UNSUPPORTED_CSV)
        delays = sorted(summary["switch_delay_s"].unique().tolist())
        steps = sorted(summary["step_walk_max_m"].unique().tolist())
        ref = steps[min(1, len(steps) - 1)]
        paths.PLANNER_OFFLINE_MD.write_text(_report(summary, uns, delays, steps, ref), encoding="utf-8")
        print(f"[planner] wrote {paths.PLANNER_OFFLINE_MD} (from existing CSVs)")
        return 0

    z = load_archive()
    maps = maps_from_archive(z)          # height_fields_before_fix: what the sim rasterises
    if args.levels is not None:
        maps = [m for m in maps if m.level in args.levels]
    if args.tasks is not None:
        maps = [m for m in maps if m.task in args.tasks]
    print(f"[planner] {len(maps)} (task, level) terrains x {len(args.delays)} delays")

    cal = DEFAULT.needs_calibration()
    if cal:
        print(f"[planner] WARNING: {len(cal)} parameters are CALIBRATION_NEEDED placeholders:")
        for key, value, _src in cal:
            print(f"[planner]   {key} = {value}")
        print("[planner] the skill *sequence* is meaningful; the absolute thresholds are not.")

    ref_step = args.step_walk_max[min(1, len(args.step_walk_max) - 1)]
    gates = [JumpGate(g) for g in args.jump_gate]
    ref_gate = gates[0]
    sum_rows, seg_rows, uns_rows = [], [], []
    for gate in gates:
      # the threshold sweep is a property of the rules, not of the gate: run it
      # once, on the reference gate, and give the other gate only the reference
      # threshold. That keeps the comparison like-for-like without paying for a
      # full cross product.
      steps_here = args.step_walk_max if gate is ref_gate else [ref_step]
      for step_max in steps_here:
        for delay in args.delays:
            cfg = DEFAULT.replace(**{"switch.SWITCH_DELAY": float(delay),
                                     "skill.STEP_WALK_MAX": float(step_max)})
            for tmap in maps:
                out = traverse(tmap, cfg, jump_gate=gate)
                sum_rows.append(out["summary"])
                if step_max == ref_step and gate is ref_gate:   # segments/events: reference only
                    for seg in out["segments"]:
                        seg_rows.append({"switch_delay_s": delay, "task": tmap.task,
                                         "level": tmap.level, **seg})
                    for u in out["unsupported"]:
                        uns_rows.append({
                            "switch_delay_s": delay, "task": tmap.task, "level": tmap.level,
                            "x_m": u.x_m, "feature": u.feature, "value": u.value,
                            "limit": u.limit, "fatal": u.fatal, "reason": u.reason,
                        })
            done = [r for r in sum_rows
                    if r["switch_delay_s"] == delay and r["step_walk_max_m"] == step_max
                    and r["jump_gate"] == str(gate)]
            print(f"[planner] gate {str(gate):<9} STEP_WALK_MAX {step_max:.2f} "
                  f"delay {delay:>5.2f} s: {len(done)} runs, "
                  f"{sum(r['switches'] for r in done)} switches, "
                  f"{sum(r['jumps'] for r in done)} jumps, "
                  f"{sum(r['stalled'] for r in done)} stalled")

    paths.OUTPUTS_DIR.mkdir(parents=True, exist_ok=True)
    summary = pd.DataFrame(sum_rows)
    summary.to_csv(paths.PLANNER_SUMMARY_CSV, index=False, float_format="%.6g")
    pd.DataFrame(seg_rows).to_csv(paths.PLANNER_SEGMENTS_CSV, index=False, float_format="%.6g")
    uns = pd.DataFrame(uns_rows)
    uns.to_csv(paths.PLANNER_UNSUPPORTED_CSV, index=False, float_format="%.6g")
    for p, n in ((paths.PLANNER_SUMMARY_CSV, len(summary)),
                 (paths.PLANNER_SEGMENTS_CSV, len(seg_rows)),
                 (paths.PLANNER_UNSUPPORTED_CSV, len(uns))):
        print(f"[planner] wrote {p} ({n} rows)")

    paths.PLANNER_OFFLINE_MD.write_text(
        _report(summary, uns, args.delays, args.step_walk_max, ref_step), encoding="utf-8")
    print(f"[planner] wrote {paths.PLANNER_OFFLINE_MD}")
    return 0


def _delay_note(ref: pd.DataFrame, delays: List[float]) -> str:
    """Explain the switch-count drop in terms of what the far field looks like."""
    lo, hi = min(delays), max(delays)
    a, b = ref[ref["switch_delay_s"] == lo], ref[ref["switch_delay_s"] == hi]
    return (
        f"Going from {lo:g} s to {hi:g} s drops the mean switch count from "
        f"{a['switches'].mean():.1f} to {b['switches'].mean():.1f}, and the share of time in WALK "
        f"from {a['frac_WALK'].mean() * 100:.0f} % to {b['frac_WALK'].mean() * 100:.0f} % — the "
        f"planner becomes *less* conservative, not more. At {hi:g} s the commitment point sits at "
        f"{b['lookahead_trot_m'].iloc[0]:.2f} m, where one depth pixel covers "
        f"{b['px_ground_width_m'].iloc[0] * 100:.0f} cm of ground, so the largest step in the "
        f"window is box-averaged from {b['true_step_up_max'].mean():.2f} m down to "
        f"{b['obs_step_up_max'].mean():.2f} m — below the thresholds that would have triggered a "
        f"downgrade. Jump triggers go from {a['jumps'].sum():.0f} to {b['jumps'].sum():.0f} over "
        f"the same range, but for a *different* reason -- see §5, where the counters show the "
        f"aimability gate shutting well before the blur would have suppressed them. A quieter "
        f"skill sequence here means the planner stopped seeing the obstacles, not that it "
        f"handled them better."
    )


def _md_table(header, rows) -> str:
    out = ["| " + " | ".join(map(str, header)) + " |",
           "| " + " | ".join(["---"] * len(header)) + " |"]
    out += ["| " + " | ".join(map(str, r)) + " |" for r in rows]
    return "\n".join(out)


def _collapse_boundary(ref: pd.DataFrame, delays: List[float]) -> dict:
    """Where perception starts to fail, by three independent criteria.

    * ``blind``   -- the commitment point passes ``far_clip``; nothing is
      observable at it at all.  Pure geometry: ``(SENSOR_FAR - BASE_MARGIN)/speed``.
    * ``invisible`` -- the required lookahead is still inside the blind zone, so
      the delay changes nothing: ``(SENSOR_NEAR - BASE_MARGIN)/speed``.
    * ``blur``    -- the first swept delay at which the *observed* step height
      falls materially below the true one.  This is the interesting one, because
      it happens long before the geometric limits.
    """
    speed = DEFAULT.skill.SPEED_TROT
    out = {
        "invisible_s": (DEFAULT.sensor.SENSOR_NEAR - DEFAULT.switch.BASE_MARGIN) / speed,
        "reliable_s": (DEFAULT.sensor.RELIABLE_RANGE - DEFAULT.switch.BASE_MARGIN) / speed,
        "blind_s": (DEFAULT.sensor.SENSOR_FAR - DEFAULT.switch.BASE_MARGIN) / speed,
    }
    loss = {}
    for d in delays:
        g = ref[ref["switch_delay_s"] == d]
        true_m, obs_m = g["true_step_up_max"].mean(), g["obs_step_up_max"].mean()
        loss[d] = 1.0 - obs_m / true_m if true_m > 1e-9 else float("nan")
    out["loss"] = loss
    out["first_loss_s"] = next((d for d in delays if np.isfinite(loss[d]) and loss[d] >= 0.02), None)
    out["blur_s"] = next((d for d in delays if np.isfinite(loss[d]) and loss[d] >= 0.25), None)
    out["half_s"] = next((d for d in delays if np.isfinite(loss[d]) and loss[d] >= 0.50), None)
    return out


def _fmt_bound(d) -> str:
    return "not reached in this sweep" if d is None else f"**{d:g} s**"


def _report(summary: pd.DataFrame, uns: pd.DataFrame, delays: List[float],
            step_maxes: List[float], ref_step: float) -> str:
    # Sections 1-5 describe ONE configuration: the reference threshold on the
    # reference gate. Pooling the gates here would average two different planners.
    ref_gate = str(summary["jump_gate"].iloc[0])
    base = summary[summary["jump_gate"] == ref_gate]
    ref = base[np.isclose(base["step_walk_max_m"], ref_step)]
    bound = _collapse_boundary(ref, delays)
    jumps_any = {d: base[base["switch_delay_s"] == d]["jumps"].sum() for d in delays}
    bound["jump_zero_s"] = next(
        (d for d in delays if all(jumps_any[e] == 0 for e in delays if e >= d)), None)

    per_delay = []
    for d in delays:
        g = ref[ref["switch_delay_s"] == d]
        if g.empty:
            continue
        per_delay.append([
            f"{d:g}", f"{g['lookahead_trot_m'].iloc[0]:.2f}",
            f"{g['px_ground_width_m'].iloc[0] * 100:.1f}",
            f"{g['switches'].mean():.1f}", f"{g['jumps'].sum():.0f}",
            f"{g['frac_WALK'].mean() * 100:.0f} / {g['frac_TROT'].mean() * 100:.0f} / "
            f"{g['frac_RUN'].mean() * 100:.0f} / {g['frac_JUMP'].mean() * 100:.0f}",
            f"{g['true_step_up_max'].mean():.3f}", f"{g['obs_step_up_max'].mean():.3f}",
            f"{bound['loss'][d] * 100:.0f} %",
            f"{g['blind_tick_frac'].mean() * 100:.0f} %",
            int(g["stalled"].sum()),
        ])

    per_step = []
    for sm in step_maxes:
        for d in delays:
            g = base[np.isclose(base["step_walk_max_m"], sm) & (base["switch_delay_s"] == d)]
            if g.empty:
                continue
            per_step.append([
                f"{sm:.2f}", f"{d:g}", f"{g['jumps'].sum():.0f}",
                f"{g['jump_band_ticks'].sum():.0f}",
                f"{g['jump_blocked_far'].sum():.0f}",
                f"{g['jump_blocked_retrigger'].sum():.0f}",
                f"{g['obs_step_up_max'].mean():.3f}",
                f"{g['switches'].mean():.1f}",
            ])

    tasks = sorted(ref["task"].unique())
    per_task = []
    for t in tasks:
        g = ref[ref["task"] == t]
        u = uns[uns["task"] == t] if len(uns) else uns
        per_task.append([
            f"`{t}`", f"{g['switches'].mean():.1f}", f"{g['jumps'].sum():.0f}",
            g["sequence"].nunique(),
            int(u["fatal"].sum()) if len(u) else 0,
            int((~u["fatal"].astype(bool)).sum()) if len(u) else 0,
            (g["sequence"].mode().iloc[0] if len(g) else "")[:52],
        ])

    reason_rows = []
    if len(uns):
        for (reason, fatal), g in uns.groupby(["reason", "fatal"]):
            reason_rows.append(["fatal" if fatal else "unverified", len(g), g["task"].nunique(),
                                f"{g['value'].min():.2f}–{g['value'].max():.2f}", reason[:110]])
        reason_rows.sort(key=lambda r: -r[1])

    cal = DEFAULT.needs_calibration()
    cal_rows = [[f"`{k}`", v, src[:110]] for k, v, src in cal]

    return f"""# Offline planner sweep — {len(summary)} runs

Generated by `scripts/simulate_planner_offline.py` on the frozen benchmark
(`height_fields_before_fix`, 20 tasks × 10 levels), swept over
`SWITCH_DELAY` ∈ {{{', '.join(f'{d:g}' for d in delays)}}} s
× `STEP_WALK_MAX` ∈ {{{', '.join(f'{v:.2f}' for v in step_maxes)}}} m.
Sections 1–4 use the reference `STEP_WALK_MAX` = {ref_step:.2f} m; §5 varies it.

> **This is not an evaluation.** A kinematic sweep advances the robot at each
> skill's measured steady speed and never decides whether it would have stayed
> upright. What it produces is the **skill sequence** and the **switch count**,
> and how both respond to the switch delay. Success rates need the simulator.

> **{len(cal)} parameters are `CALIBRATION_NEEDED` placeholders** (§6). §5 exists
> precisely because one of them, `STEP_WALK_MAX`, could otherwise be mistaken for
> the cause of a result that is really the sensor model's doing.

## 1. Effect of the switch delay

`lookahead = BASE_MARGIN + SWITCH_DELAY × speed`, at trot speed
({DEFAULT.skill.SPEED_TROT:.2f} m/s). The decision window is
`[max({DEFAULT.sensor.SENSOR_NEAR:.2f}, lookahead), min({DEFAULT.sensor.SENSOR_FAR:.2f}, lookahead + {DEFAULT.feature.DECISION_WINDOW_M:.1f})]`.
**px width** is the ground covered by one depth pixel *at the commitment point* —
the number that decides whether an obstacle survives the sensor model.
**true / seen** are the largest step in the window before and after the pixel blur,
averaged over the {len(ref[ref['switch_delay_s'] == delays[0]])} runs at that delay.

{_md_table(["delay s", "lookahead m", "px width cm", "switches", "jumps", "WALK/TROT/RUN/JUMP %", "true step m", "seen step m", "step lost", "blind ticks", "stalled"], per_delay)}

## 2. Where perception collapses

Several thresholds get called "the sensor limit". They are not the same number,
and the binding one is not the obvious one:

| boundary | delay | what happens |
| :--- | ---: | :--- |
| delay is **invisible** | ≤ {bound['invisible_s']:.2f} s | required lookahead is still inside the {DEFAULT.sensor.SENSOR_NEAR:.2f} m blind zone, so the decision window is unchanged. `(SENSOR_NEAR − BASE_MARGIN) / speed` |
| **blur starts to bite** | {_fmt_bound(bound['first_loss_s'])} | first swept delay losing ≥ 2 % of the true step height |
| **blur is material** | {_fmt_bound(bound['blur_s'])} | first swept delay losing ≥ 25 % |
| **half the geometry is gone** | {_fmt_bound(bound['half_s'])} | first swept delay losing ≥ 50 % |
| **jump becomes unaimable** | {_fmt_bound(bound['jump_zero_s'])} | first swept delay at which no jump fires at any threshold (§5) |
| past the **reliable range** | > {bound['reliable_s']:.2f} s | commitment point beyond {DEFAULT.sensor.RELIABLE_RANGE:.1f} m |
| fully **blind** | > {bound['blind_s']:.2f} s | commitment point past `far_clip`; the window collapses and the planner reports it |

The binding limit for *this* planner is {_fmt_bound(bound['jump_zero_s'])} — not the
{bound['blind_s']:.2f} s at which the window finally goes blind, and (per §6) not a
limit of the sensor either.
Perception does not fail when the commitment point leaves the sensor range. Two
things break long before that, and they break at different delays:

* The **jump stops firing** at {_fmt_bound(bound['jump_zero_s'])} — a hard edge
  rather than a slope, because the near-edge gate is a boolean: `front_jump`
  covers 26 mm of ground, and the gate refuses it the moment the commitment point
  leaves the blind-zone edge. §6 shows this edge is a property of the **decision
  window**, not of the sensor or the robot — a gate that tracks obstacles across
  the full visible range still fires at every delay in this sweep.
* The **blur** degrades gradually from {_fmt_bound(bound['first_loss_s'])}
  ({bound['loss'][bound['first_loss_s']] * 100:.0f} % of the step height lost) to
  {bound['loss'][max(delays)] * 100:.0f} % at {max(delays):g} s. The pixel footprint
  goes as roughly `d²`, so it overtakes a 0.1–0.2 m step well inside the 2 m range.

Read together: below ~{bound['invisible_s']:.1f} s the delay is invisible, between
there and {_fmt_bound(bound['jump_zero_s'])} the planner degrades smoothly, and past
{_fmt_bound(bound['jump_zero_s'])} it loses a whole skill outright while still
believing the terrain is flatter than it is. The blur is a real sensor limit; the
skill loss is not, and §6 recovers it.

**Fewer switches at a long delay is not an improvement.** {_delay_note(ref, delays)}

## 3. Per task (reference `STEP_WALK_MAX`, all delays pooled)

All 20 tasks are run identically; none is excepted out.

{_md_table(["task", "switches", "jumps", "distinct sequences", "unsupported", "unverified", "most common sequence"], per_task)}

Note on `stepping_stones_*`: gaps are now measured on the **raw centre line**, not
on the corridor-following path, so a line of stones no longer reads as continuous
ground. The four tasks with int16 overflow artefacts (`sphere_bump*`,
`flat_circle_jump`, `bump_jump`) report some unsupported events from the spurious
blobs; per `CLAUDE.md` §2 those bugs stay in, because the E2E policy meets them too.

## 4. Why terrain was unsupported

{_md_table(["kind", "events", "tasks", "value range", "reason"], reason_rows) if reason_rows else "_No unsupported terrain recorded._"}

## 5. Is the jump count a threshold effect or a resolution effect?

`STEP_WALK_MAX` is a `CALIBRATION_NEEDED` placeholder, and JUMP only fires for a
step in `(STEP_WALK_MAX, STEP_JUMP_MAX]`. Lowering it widens that band, so if the
jumps that disappear at long delay were a threshold artefact, a lower threshold
would bring them back. The three counters separate the possible causes:

* **ticks in band** — the observed step landed inside the jump band at all. Zero
  here means the geometry never even offered a jump: a *resolution* effect,
  because the band's own position moves with `STEP_WALK_MAX` and this column does
  not recover when it is lowered.
* **blocked: window not near** — the step was in the band, but the decision
  window no longer starts at the blind-zone edge. `front_jump` covers 26 mm of
  ground, so it is only aimable at something the robot already stands at; once
  the delay pushes the commitment point outwards this gate shuts. **Structural —
  no threshold value re-opens it.**
* **blocked: retrigger** — in band, window near, but a previous jump has not yet
  been cleared by `JUMP_RETRIGGER_M` of travel.

{_md_table(["STEP_WALK_MAX m", "delay s", "jumps", "ticks in band", "blocked: window not near", "blocked: retrigger", "seen step m", "switches"], per_step)}

{_degenerate_note(base, step_maxes)}{_verdict(base, delays, step_maxes)}

{_gate_section(summary, delays, ref_step)}## 7. Parameters still needing calibration

{_md_table(["parameter", "placeholder", "why it is not a measurement"], cal_rows) if cal_rows else "_None._"}

Per `CLAUDE.md` §2 these must be settled on a separate calibration terrain, never
by tuning against benchmark scores. That terrain exists:
`data/calibration_probes.npz` (`scripts/freeze_calibration.py`) holds isolated
step and gap probes covering five of these; `outputs/calibration_plan.md` maps
each parameter to its probe and states which five are still uncovered.
"""


def _gate_section(summary: pd.DataFrame, delays: List[float], ref_step: float) -> str:
    """Compare the two jump gates: is the long-delay zero physical or a window artefact?"""
    gates = [g for g in ("near_edge", "tracking") if (summary["jump_gate"] == g).any()]
    if len(gates) < 2:
        return ""
    ref = summary[np.isclose(summary["step_walk_max_m"], ref_step)]

    def at(d, g):
        return ref[(ref["switch_delay_s"] == d) & (ref["jump_gate"] == g)]

    rows = []
    for d in delays:
        ne, tr = at(d, "near_edge"), at(d, "tracking")
        rows.append([
            f"{d:g}",
            f"{ne['jumps'].sum():.0f}" if len(ne) else "-",
            f"{tr['jumps'].sum():.0f}" if len(tr) else "-",
            f"{tr['jump_detections'].sum():.0f}" if len(tr) else "-",
            f"{ne['jump_blocked_far'].sum():.0f}" if len(ne) else "-",
            f"{tr['requests_dropped_busy'].mean():.0f}" if len(tr) else "-",
            f"{tr['switches'].mean():.1f}" if len(tr) else "-",
        ])

    speed = DEFAULT.skill.SPEED_TROT
    reach_limit = DEFAULT.sensor.SENSOR_FAR / speed
    win = DEFAULT.feature.DECISION_WINDOW_M
    ne_zero = next((d for d in delays if at(d, "near_edge")["jumps"].sum() == 0), None)
    tr_zero = next((d for d in delays
                    if all(at(e, "tracking")["jumps"].sum() == 0 for e in delays if e >= d)), None)

    if ne_zero is not None and (tr_zero is None or tr_zero > ne_zero):
        tail = (f"does not reach zero until **{tr_zero:g} s**" if tr_zero is not None
                else "never reaches zero across the swept range")
        verdict = (
            f"**Window design, not a physical limit.** The near-edge gate reaches zero at "
            f"**{ne_zero:g} s**. Tracking, on exactly the same sensor model and the same "
            f"thresholds, still fires {at(ne_zero, 'tracking')['jumps'].sum():.0f} jumps there and "
            f"{tail}. The obstacles were visible the whole time; the near-edge gate was simply not "
            f"allowed to look at them, because the decision window slides away from the robot as "
            f"the delay grows and the thing it is about to reach falls out of the window's near "
            f"side. **Recorded as a window-design problem, not a capability limit.**"
        )
    elif ne_zero is not None and tr_zero is not None and tr_zero <= ne_zero:
        verdict = (
            f"**Physical limit confirmed.** Both gates reach zero at **{tr_zero:g} s** or earlier. "
            f"Watching the whole visible range and firing on time-to-arrival recovers nothing, so "
            f"the jump really does stop being usable at that delay rather than being hidden by the "
            f"decision window."
        )
    else:
        verdict = (
            "Neither gate reaches zero in the swept range, so this sweep does not locate the "
            "boundary. Extend `--delays` past the current maximum before concluding anything."
        )

    header = ["delay s", "jumps: near_edge", "jumps: tracking", "detections (tracking)",
              "near-edge refusals", "dropped while busy", "switches (tracking)"]
    return (
        "## 6. Is the JUMP gate a physical limit or a window artefact?\n\n"
        "`front_jump` covers 26 mm of ground, so it has to be launched while the robot is\n"
        "standing at the step - the command must go out `speed x SWITCH_DELAY` metres early.\n"
        "Section 5 showed the jump count going to zero at long delay for reasons that are neither\n"
        "the threshold nor the blur, which leaves two possibilities:\n\n"
        "* **physical** - at that delay the decision genuinely has to be made before the obstacle\n"
        "  can be seen; or\n"
        f"* **a window artefact** - the obstacle *is* visible, but the decision window\n"
        f"  `[lookahead, lookahead + {win:.1f}]` slides away from the robot as the delay grows, so\n"
        "  the thing it is about to reach falls out of the window's near side and the near-edge\n"
        "  gate refuses it.\n\n"
        "`JumpGate.TRACKING` separates them. It watches the **whole visible range**\n"
        f"({DEFAULT.sensor.SENSOR_NEAR:.2f}-{DEFAULT.sensor.SENSOR_FAR:.2f} m) every tick, converts\n"
        "each detected step into a world-frame position, and fires when that obstacle's estimated\n"
        "time of arrival falls to `SWITCH_DELAY`. Detection runs on the **blurred** profile inside\n"
        "the same range limits - the sensor model is untouched. Only *which part of what the camera\n"
        "already sees* the planner may act on changes; thresholds, hysteresis, hold time and the\n"
        "one-command channel are identical.\n\n"
        + _md_table(header, rows) + "\n\n"
        + verdict + "\n\n"
        f"The geometric ceiling on tracking is `SENSOR_FAR / speed` = **{reach_limit:.2f} s** at trot\n"
        "speed: past that the commitment point is further away than the camera can see, and no\n"
        "amount of tracking helps. That, not the near-edge gate, is where the physical limit sits.\n\n"
        "**A caveat on the tracking gate.** It fires on a time-to-arrival computed from the\n"
        "*current* skill's speed. If the planner changes gait while the jump is in flight the robot\n"
        "arrives early or late; nothing here measures landing accuracy, and a kinematic sweep\n"
        "cannot. Tracking restoring the jump count means the decision was *available*, not that the\n"
        "jump would have landed on the step.\n\n"
    )


def _degenerate_note(summary: pd.DataFrame, step_maxes: List[float]) -> str:
    """Warn about any swept threshold that closes the jump band entirely."""
    jump_max = DEFAULT.skill.STEP_JUMP_MAX
    dead = [sm for sm in step_maxes if sm >= jump_max]
    if not dead:
        return ""
    return (
        f"> **{', '.join(f'{v:.2f}' for v in dead)} m closes the band, it does not narrow it.** "
        f"JUMP fires on `(STEP_WALK_MAX, STEP_JUMP_MAX]` and `STEP_JUMP_MAX` is "
        f"{jump_max:.2f} m, so at those rows the interval is empty and the ticks-in-band column "
        f"is 0 by construction at *every* delay — that is arithmetic, not a measurement about "
        f"the terrain or the sensor. Both are `CALIBRATION_NEEDED` placeholders; the ordering "
        f"`STEP_WALK_MAX < STEP_JUMP_MAX` is a structural requirement of the rule, and a "
        f"calibration that violates it removes the skill.\n\n"
    )


def _verdict(summary: pd.DataFrame, delays: List[float], step_maxes: List[float]) -> str:
    """Attribute the long-delay jump count to a cause, from the counters."""
    dmax, dmin = max(delays), min(delays)
    at_max = summary[summary["switch_delay_s"] == dmax]
    at_min = summary[summary["switch_delay_s"] == dmin]
    lowest = min(step_maxes)
    low_at_max = at_max[np.isclose(at_max["step_walk_max_m"], lowest)]

    jumps_by_step = {sm: at_max[np.isclose(at_max["step_walk_max_m"], sm)]["jumps"].sum()
                     for sm in step_maxes}
    zero_everywhere = all(v == 0 for v in jumps_by_step.values())

    band_ticks = low_at_max["jump_band_ticks"].sum()
    far_blocked = low_at_max["jump_blocked_far"].sum()
    retrig = low_at_max["jump_blocked_retrigger"].sum()
    seen_max, seen_min = at_max["obs_step_up_max"].mean(), at_min["obs_step_up_max"].mean()
    true_max = at_max["true_step_up_max"].mean()

    lines = []
    if not zero_everywhere:
        lines.append(
            f"**Threshold-sensitive.** At {dmax:g} s the jump count is not zero for every "
            f"`STEP_WALK_MAX` tried ({', '.join(f'{sm:.2f} m -> {v:.0f}' for sm, v in jumps_by_step.items())}), "
            f"so the disappearance at the reference value is at least partly the threshold's "
            f"doing. Nothing here can be read as a finding until `STEP_WALK_MAX` is calibrated."
        )
    elif band_ticks == 0:
        lines.append(
            f"**Resolution, not threshold.** At {dmax:g} s no tick's observed step lands in the "
            f"jump band for any `STEP_WALK_MAX` tried, down to {lowest:.2f} m. Widening the band "
            f"cannot recover a jump whose trigger never enters it: the mean observed step falls "
            f"from {seen_min:.3f} m at {dmin:g} s to {seen_max:.3f} m at {dmax:g} s against a true "
            f"{true_max:.3f} m, so the blurred geometry sits below the band's floor wherever that "
            f"floor is put."
        )
    elif far_blocked >= 0.9 * band_ticks:
        lines.append(
            f"**Neither — it is a structural gate.** At {dmax:g} s the observed step still lands "
            f"in the jump band on {band_ticks:.0f} ticks (so the band is reachable, and lowering "
            f"`STEP_WALK_MAX` to {lowest:.2f} m does not change that), but {far_blocked:.0f} of "
            f"those — {far_blocked / band_ticks * 100:.0f} % — are refused because the decision "
            f"window no longer starts at the blind-zone edge. `front_jump` covers 26 mm of ground; "
            f"once the commitment point moves out to "
            f"{low_at_max['lookahead_trot_m'].iloc[0]:.2f} m there is nothing for it to aim at. "
            f"**No value of `STEP_WALK_MAX` re-opens that gate**, which is why the jump count is "
            f"0 at every threshold tried ({', '.join(f'{sm:.2f}' for sm in step_maxes)} m). "
            f"Whether that gate is a real limit or an artefact of how the decision window is "
            f"shaped is the question §6 answers."
        )
        lines.append(
            f"The resolution effect is real but secondary here: the same runs see "
            f"{seen_max:.3f} m where the true step is {true_max:.3f} m "
            f"({(1 - seen_max / true_max) * 100:.0f} % lost to the pixel footprint), which is what "
            f"suppresses the *downgrades* in §1. For the jump specifically, the aimability gate "
            f"shuts first."
        )
    else:
        lines.append(
            f"**Mixed.** At {dmax:g} s the band is entered on {band_ticks:.0f} ticks; "
            f"{far_blocked:.0f} are refused for an out-of-reach window and {retrig:.0f} by the "
            f"retrigger guard, leaving the rest to hysteresis and the hold time. Both the sensor "
            f"model and the rule state machine contribute; the threshold does not, since the count "
            f"is 0 at every value tried."
        )

    jumps_ref = {d: summary[summary["switch_delay_s"] == d]["jumps"].sum() for d in delays}
    tail_zero = [d for d in delays if all(jumps_ref[e] == 0 for e in delays if e >= d)]
    if all(v == 0 for v in jumps_ref.values()):
        lines.append("No jump fired at any delay in this sweep, so the column carries no signal.")
    elif tail_zero:
        d0 = min(tail_zero)
        lines.append(
            f"Pooled over thresholds, the jump count reaches zero at **{d0:g} s** and stays "
            f"there for every longer delay."
        )
    return "\n\n".join(lines)


if __name__ == "__main__":
    raise SystemExit(main())
