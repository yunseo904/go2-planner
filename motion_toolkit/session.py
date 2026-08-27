"""Loading of one curated Go2 log session.

A session directory holds ``data.npz`` (one 1-D float array per column),
``meta.json`` (column list, ``t0_monotonic``, leg/joint order), ``MANIFEST.json``
(sealing note, recording info) and — for the API-driven sessions — an
``events.jsonl`` skill timeline.  The three 2026-08-04 gamepad sessions have no
``events.jsonl`` and no ``duration_s``.

Everything here is read-only; the curated tree is never written to.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

#: Native leg order used by every column name in the log.
LEGS: List[str] = ["FR", "FL", "RR", "RL"]
JOINTS: List[str] = ["hip", "thigh", "calf"]

#: ``PosStopF`` sentinel written into ``*_q_des`` when the joint is not
#: position-controlled (2.146e9 in the Unitree low-level command).
POS_STOP_F: float = 2.146e9

#: Columns that are all-zero on this firmware and must not be used.
DEAD_COLUMN_PREFIXES = ("foot_pos_", "foot_vel_")


@dataclass
class Event:
    t_mono: float
    event: str
    skill: Optional[str] = None
    param: Optional[dict] = None
    rc: Optional[int] = None
    name: Optional[str] = None

    @property
    def label(self) -> str:
        return self.skill or self.name or self.event


@dataclass
class Session:
    """One curated log session, loaded lazily-enough for 36 of them to fit in RAM."""

    path: Path
    group: str
    name: str
    meta: dict
    manifest: dict
    events: List[Event]
    _npz: np.lib.npyio.NpzFile = field(repr=False)
    _cache: Dict[str, np.ndarray] = field(default_factory=dict, repr=False)

    # -- columns ---------------------------------------------------------
    def col(self, key: str) -> np.ndarray:
        """One column as float64.  Raises for the known-dead foot_* columns."""
        if key.startswith(DEAD_COLUMN_PREFIXES):
            raise KeyError(f"{key} is all-zero on this firmware (see INDEX.md caveat 1)")
        if key not in self._cache:
            self._cache[key] = np.asarray(self._npz[key], dtype=np.float64)
        return self._cache[key]

    def stack(self, keys: List[str]) -> np.ndarray:
        """``(n_samples, len(keys))`` float64 matrix."""
        return np.stack([self.col(k) for k in keys], axis=1)

    @property
    def t(self) -> np.ndarray:
        """Sample time in seconds, 0 at the first recorded sample."""
        return self.col("t")

    @property
    def n(self) -> int:
        return len(self.t)

    @property
    def fs(self) -> float:
        """Sample rate from the median sample interval (~420 Hz)."""
        dt = np.diff(self.t)
        dt = dt[np.isfinite(dt) & (dt > 0)]
        return float(1.0 / np.median(dt)) if dt.size else float("nan")

    @property
    def duration_s(self) -> float:
        return float(self.t[-1] - self.t[0])

    @property
    def t0_monotonic(self) -> Optional[float]:
        """Monotonic clock of sample 0.  ``None`` for the pre-1.0.0 recorder."""
        v = self.meta.get("t0_monotonic")
        return float(v) if v is not None else None

    def event_time(self, ev: Event) -> float:
        """Event time on the sample time axis (NaN if the session has no t0)."""
        t0 = self.t0_monotonic
        return float("nan") if t0 is None else ev.t_mono - t0

    # -- convenience joint/foot matrices ---------------------------------
    def joint_matrix(self, suffix: str) -> np.ndarray:
        """``(n, 12)`` matrix of ``<LEG>_<JOINT>_<suffix>`` in LEGS x JOINTS order."""
        return self.stack([f"{l}_{j}_{suffix}" for l in LEGS for j in JOINTS])

    def foot_force(self) -> np.ndarray:
        """``(n, 4)`` foot force in LEGS order.  Clips at ~210 N (INDEX caveat 2)."""
        return self.stack([f"foot_force_{l}" for l in LEGS])

    # -- skill timeline ---------------------------------------------------
    def skill_sends(self) -> List[Event]:
        return [e for e in self.events if e.event == "skill_send"]

    def skill_dones(self) -> List[Event]:
        return [e for e in self.events if e.event == "skill_done"]

    def skill_sequence(self) -> List[str]:
        return [e.skill or "?" for e in self.skill_sends()]


def _read_json(p: Path) -> dict:
    return json.loads(p.read_text())


def load_session(session_dir: Path, group: str = "") -> Session:
    d = Path(session_dir)
    meta = _read_json(d / "meta.json")
    manifest = _read_json(d / "MANIFEST.json") if (d / "MANIFEST.json").is_file() else {}
    events: List[Event] = []
    ev_path = d / "events.jsonl"
    if ev_path.is_file():
        for line in ev_path.read_text().splitlines():
            line = line.strip()
            if not line:
                continue
            raw = json.loads(line)
            events.append(
                Event(
                    t_mono=float(raw["t_mono"]),
                    event=str(raw["event"]),
                    skill=raw.get("skill"),
                    param=raw.get("param"),
                    rc=raw.get("rc"),
                    name=raw.get("name"),
                )
            )
    npz = np.load(d / "data.npz", allow_pickle=False)
    return Session(
        path=d,
        group=group or d.parent.name,
        name=meta.get("name") or d.name,
        meta=meta,
        manifest=manifest,
        events=events,
        _npz=npz,
    )


def iter_sessions(curated_root: Path) -> List[Session]:
    """Every ``<group>/<session>/`` under the curated root, sorted."""
    root = Path(curated_root)
    out: List[Session] = []
    for group_dir in sorted(p for p in root.iterdir() if p.is_dir()):
        for sess_dir in sorted(p for p in group_dir.iterdir() if p.is_dir()):
            if (sess_dir / "data.npz").is_file():
                out.append(load_session(sess_dir, group=group_dir.name))
    return out
