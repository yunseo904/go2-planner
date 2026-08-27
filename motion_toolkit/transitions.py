"""Skill-transition cost from ``events.jsonl`` + the measured motion window.

Three things are measured, and they are deliberately kept apart because they
have different causes:

``call_s``      ``skill_send`` -> ``skill_done``.  This is the *script* round
                trip: ``run.sh`` needs ~2.4 s to start (MANIFEST note), so even
                a fire-and-forget call such as ``speed_level`` costs ~2.45 s.
                For ``move`` it additionally contains the commanded duration.
``lag_s``       ``skill_send`` -> first sample of the motion segment that the
                skill produced.  This is the real dead time a planner has to
                budget: the command has not reached the robot before it.
``settle_s``    end of vigorous motion -> body at rest (see
                :func:`motion_toolkit.profile._settle_time`).

Segments are attributed to skills by "last ``skill_send`` at or before the
segment start"; ``speed_level`` is excluded because it never moves the robot.
A skill that produced no segment at all (``balance_stand`` when the robot is
already standing) is reported with ``lag_s = NaN`` and ``moved = False``.
"""

from __future__ import annotations

from collections import Counter
from typing import Dict, List

import numpy as np
import pandas as pd

from .profile import PARAM_SKILLS, _settle_time
from .session import Session
from .window import detect_motion


def transition_rows(sess: Session) -> List[Dict[str, object]]:
    """One row per ``skill_send`` of one session."""
    t0 = sess.t0_monotonic
    sends = sess.skill_sends()
    if not sends or t0 is None:
        return []
    done_list = sess.skill_dones()
    win = detect_motion(sess)
    t = sess.t

    seq = [e.skill or "?" for e in sends]
    rows: List[Dict[str, object]] = []
    for k, ev in enumerate(sends):
        send_t = ev.t_mono - t0
        # first skill_done for the same skill after this send
        done = next((d for d in done_list if d.skill == ev.skill and d.t_mono >= ev.t_mono), None)
        call_s = (done.t_mono - ev.t_mono) if done else np.nan

        # segments attributed to this send: those starting before the next
        # motion-producing send
        nxt = next(
            (e.t_mono - t0 for e in sends[k + 1:] if e.skill not in PARAM_SKILLS),
            float("inf"),
        )
        own = [(a, b) for a, b in win.segments if send_t <= t[a] < nxt]
        moved = bool(own) and ev.skill not in PARAM_SKILLS
        lag = float(t[own[0][0]] - send_t) if moved else np.nan
        motion_s = float(sum(b - a for a, b in own) / sess.fs) if own else 0.0
        if moved:
            end = own[-1][1]
            hot = np.flatnonzero(win.speed[:end] > win.enter_level)
            settle = _settle_time(
                sess, int(hot[-1]) if hot.size else end - 1, win.speed, win.exit_level
            )
        else:
            settle = np.nan

        rows.append(
            {
                "group": sess.group,
                "session": sess.path.name,
                "index": k,
                "skill": ev.skill,
                "prev_skill": seq[k - 1] if k else "(session_start)",
                "param": "" if ev.param in (None, {}) else str(ev.param),
                "send_s": float(send_t),
                "call_s": float(call_s),
                "cmd_duration_s": float((ev.param or {}).get("duration", np.nan)),
                "lag_s": lag,
                "motion_s": motion_s,
                "n_segments": len(own),
                "settle_s": settle,
                "moved": moved,
                "rc": done.rc if done else None,
            }
        )
    return rows


def transition_table(sessions: List[Session]) -> pd.DataFrame:
    rows: List[Dict[str, object]] = []
    for s in sessions:
        rows.extend(transition_rows(s))
    return pd.DataFrame(rows)


def predecessor_table(df: pd.DataFrame) -> pd.DataFrame:
    """For every skill, which skill_send immediately precedes it, and how often."""
    out = []
    for skill, sub in df.groupby("skill"):
        counts = Counter(sub["prev_skill"])
        n = len(sub)
        for prev, c in counts.most_common():
            out.append(
                {
                    "skill": skill,
                    "prev_skill": prev,
                    "count": c,
                    "n_sends": n,
                    "share": c / n,
                    "mandatory": c == n and n > 1,
                }
            )
    return pd.DataFrame(out).sort_values(["skill", "count"], ascending=[True, False])


def summarize(df: pd.DataFrame, by: str = "skill") -> pd.DataFrame:
    """Per-skill distribution of the three transition costs."""

    def q(x, p):
        x = np.asarray(x, dtype=float)
        x = x[np.isfinite(x)]
        return float(np.percentile(x, p)) if x.size else np.nan

    rows = []
    for skill, sub in df.groupby(by):
        rows.append(
            {
                by: skill,
                "n": len(sub),
                "n_moved": int(sub["moved"].sum()),
                "call_mean_s": q(sub["call_s"], 50),
                "call_min_s": q(sub["call_s"], 0),
                "call_max_s": q(sub["call_s"], 100),
                "call_overhead_s": q(sub["call_s"] - sub["cmd_duration_s"].fillna(0.0), 50),
                "lag_mean_s": float(np.nanmean(sub["lag_s"])) if sub["lag_s"].notna().any() else np.nan,
                "lag_std_s": float(np.nanstd(sub["lag_s"])) if sub["lag_s"].notna().any() else np.nan,
                "lag_min_s": q(sub["lag_s"], 0),
                "lag_max_s": q(sub["lag_s"], 100),
                "motion_mean_s": float(np.nanmean(sub["motion_s"])),
                "settle_mean_s": float(np.nanmean(sub["settle_s"])) if sub["settle_s"].notna().any() else np.nan,
                "settle_max_s": q(sub["settle_s"], 100),
            }
        )
    return pd.DataFrame(rows).sort_values("skill").reset_index(drop=True)
