#!/usr/bin/env python3
"""What the foot placement does across a skill switch.

    python3 scripts/diagnose_switch.py outputs/traces/planner/sw_base.npz

Reads a trace from ``run_planner_replay.py`` -- no simulator, no GPU -- and asks
the three questions a switch raises about the correction that made TROT hold:

1. what the base is doing at the seam (v_y, yaw rate), which is the correction's
   only input;
2. whether the correction is responding or SATURATING -- a capped correction is
   one that has stopped being a function of the velocity it is supposed to answer;
3. what changes about the law itself at the seam. The gain is
   ``T_stance/2 + k`` and ``T_stance`` is a property of the CLIP, so it steps
   discontinuously: WALK 0.484 s -> TROT 0.372 s means the gain drops 23% at the
   instant the body is furthest from either gait's steady state.

On history: for WALK and TROT the law is memoryless -- ``yaw_mode`` is off, so the
correction is a static function of the instantaneous v_y and there is nothing to
reset. Only TURN carries state (the one-cycle yaw ring), and switching into TURN
does clear it, so its yaw target is averaged over a partial window for one cycle.
That is reported rather than assumed.
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sim.replay import quat_to_rpy_deg                                   # noqa: E402

T_STANCE = {"WALK": 0.484, "TROT": 0.372, "TURN": 0.600}


def report(path: str, cap_rad: float = 0.05, window_s: float = 1.5) -> None:
    z = np.load(path, allow_pickle=False)
    dt = float(z["dt"])
    order = [str(x) for x in z["skill_order"]]
    played = z["played"]
    u = z["foot_u"] if "foot_u" in z.files else None
    v, w = z["root_lin_vel_b"], z["root_ang_vel_b"]
    r, p, _ = quat_to_rpy_deg(z["root_quat_w"])
    sw = [i for i in range(1, len(played)) if played[i] != played[i - 1]]

    print("=" * 78)
    print(f"{Path(path).name}: {len(played)} steps = {len(played)*dt:.2f} s, "
          f"{len(sw)} switch(es)")
    if u is None:
        print("  (no foot_u in this trace -- re-run with the instrumented runner)")
        return
    for s in sw:
        a, b = order[int(played[s - 1])], order[int(played[s])]
        k = int(round(window_s / dt))
        pre, post = slice(max(0, s - k), s), slice(s, min(len(played), s + k))
        hip = np.abs(u[:, 0::3])
        print(f"\n-- {a} -> {b} at {s*dt:.2f} s "
              f"(gain {0.5*T_STANCE.get(a, np.nan):.3f} -> {0.5*T_STANCE.get(b, np.nan):.3f} s, "
              f"{100*(T_STANCE.get(b,np.nan)/T_STANCE.get(a,np.nan)-1):+.0f}%) --")
        print(f"   {'window':>8s} {'|v_y| mean':>11s} {'|v_y| max':>10s} {'yaw mean':>9s} "
              f"{'yaw sd':>7s} {'|roll| max':>11s}")
        for label, sl in (("before", pre), ("after", post)):
            print(f"   {label:>8s} {np.abs(v[sl,1]).mean():11.4f} {np.abs(v[sl,1]).max():10.4f} "
                  f"{np.degrees(w[sl,2]).mean():9.2f} {np.degrees(w[sl,2]).std():7.2f} "
                  f"{np.abs(r[sl]).max():11.2f}")
        print(f"   {'window':>8s} {'hip corr RMS':>13s} {'max':>8s} {'at cap':>8s} "
              f"{'swing samples':>14s}")
        for label, sl in (("before", pre), ("after", post)):
            hs = hip[sl]
            live = hs > 1e-9
            atcap = (hs >= cap_rad - 1e-6)
            print(f"   {label:>8s} {np.sqrt((hs[live]**2).mean()) if live.any() else 0:13.4f} "
                  f"{hs.max():8.4f} {100*atcap.sum()/max(live.sum(),1):7.1f}% "
                  f"{int(live.sum()):14d}")
        # how long after the seam does the correction stay pinned?
        pinned = (hip[s:] >= cap_rad - 1e-6).any(axis=1)
        if pinned.any():
            run = 0
            for q in pinned:
                if q:
                    run += 1
                else:
                    break
            first_free = int(np.argmin(pinned)) if not pinned.all() else len(pinned)
            print(f"   correction is pinned at the {cap_rad:g} rad cap for the first "
                  f"{first_free} steps ({first_free*dt:.2f} s) after the seam")
        else:
            print(f"   correction never reaches the {cap_rad:g} rad cap after the seam")


if __name__ == "__main__":
    for a in sys.argv[1:] or ["outputs/traces/planner/sw_base.npz"]:
        report(a)
