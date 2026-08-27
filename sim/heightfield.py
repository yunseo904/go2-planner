"""Height field -> triangle mesh, and the calibration protocol's arithmetic.

Both halves are pure numpy so they can be tested without a simulator: the mesh
conversion against a known probe, and the threshold rule against a synthetic
pass/fail matrix.  The Isaac Lab code in ``scripts/run_calibration.py`` is a thin
shell around them.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


def to_trimesh(hf: np.ndarray, horizontal_scale: float, vertical_scale: float) -> Tuple[np.ndarray, np.ndarray]:
    """``(vertices, faces)`` for an ``(nx, ny)`` raw int16 height field.

    Vertices sit on the grid; each cell becomes two triangles.  Heights are the
    raw values times ``vertical_scale``, which is how the frozen archive stores
    them (``terrain_toolkit/calibrate.py``) -- no re-quantisation here, so a
    0.02 m step in the archive is a 0.02 m step in the mesh.
    """
    nx, ny = hf.shape
    x = np.arange(nx, dtype=np.float64) * horizontal_scale
    y = np.arange(ny, dtype=np.float64) * horizontal_scale
    xx, yy = np.meshgrid(x, y, indexing="ij")
    verts = np.stack([xx.ravel(), yy.ravel(), (hf.astype(np.float64) * vertical_scale).ravel()], axis=1)

    i, j = np.meshgrid(np.arange(nx - 1), np.arange(ny - 1), indexing="ij")
    v00 = (i * ny + j).ravel()
    v10, v01, v11 = v00 + ny, v00 + 1, v00 + ny + 1
    faces = np.concatenate([
        np.stack([v00, v10, v11], axis=1),
        np.stack([v00, v11, v01], axis=1),
    ], axis=0).astype(np.int32)
    return verts, faces


def height_at(hf: np.ndarray, horizontal_scale: float, vertical_scale: float,
              x_m: float, y_m: float) -> float:
    """Nearest-cell terrain height in metres, for spawn placement."""
    i = int(np.clip(round(x_m / horizontal_scale), 0, hf.shape[0] - 1))
    j = int(np.clip(round(y_m / horizontal_scale), 0, hf.shape[1] - 1))
    return float(hf[i, j]) * vertical_scale


# --------------------------------------------------------------------------- #
# The protocol
# --------------------------------------------------------------------------- #

@dataclass
class Threshold:
    """One calibrated limit and everything needed to argue with it."""

    parameter: str
    family: str
    skill: str
    value_m: Optional[float]
    highest_all_pass_m: Optional[float]
    step_m: float
    n_reps: int
    monotone: bool
    first_failure_m: Optional[float]
    raw: List[Tuple[float, int, int]]      # (level, n_pass, n_reps)
    note: str = ""


def threshold_from_matrix(
    levels_m: Sequence[float],
    passed: np.ndarray,
    parameter: str = "",
    family: str = "",
    skill: str = "",
) -> Threshold:
    """Largest level cleared on **every** repeat, backed off one level.

    Two values come out, and the difference between them is the finding:

    ``highest_all_pass_m``  the largest level where all repeats succeeded,
                            whatever happened below it.
    ``value_m``             the top of the *unbroken* run of all-pass levels
                            from the bottom, minus one level.

    They differ exactly when the results are non-monotone -- a fail at 0.14 m
    and a pass at 0.16 m.  That is not noise to be smoothed over: it means the
    outcome depends on something other than the height, and the conservative
    prefix value is the only one that can be defended.  ``monotone`` says which
    case you are in.
    """
    levels = np.asarray(levels_m, dtype=float)
    p = np.asarray(passed, dtype=bool)
    if p.ndim == 1:
        p = p[:, None]
    if p.shape[0] != levels.size:
        raise ValueError(f"{p.shape[0]} level rows for {levels.size} levels")
    order = np.argsort(levels)
    levels, p = levels[order], p[order]
    all_pass = p.all(axis=1)
    step = float(np.median(np.diff(levels))) if levels.size > 1 else float("nan")

    prefix = int(np.argmin(all_pass)) if not all_pass.all() else levels.size
    highest = float(levels[all_pass][-1]) if all_pass.any() else None
    monotone = bool(highest is None or prefix == 0 or np.isclose(levels[prefix - 1], highest))
    first_fail = float(levels[prefix]) if prefix < levels.size else None

    if prefix == 0:
        value, note = None, "failed at the smallest level; nothing to back off from"
    else:
        value = float(levels[prefix - 1] - step)
        if value < 0:
            value, note = 0.0, "cleared only the smallest level; the limit is below one step"
        else:
            note = ""
    if not monotone:
        note = (note + " " if note else "") + (
            f"NON-MONOTONE: failed at {first_fail:.2f} m but passed every repeat at "
            f"{highest:.2f} m. Something other than the level is deciding the outcome; the "
            f"conservative value is reported and the raw matrix must be looked at.")
    return Threshold(parameter, family, skill, value, highest, step, int(p.shape[1]),
                     monotone, first_fail, [(float(l), int(r.sum()), int(r.size)) for l, r in zip(levels, p)],
                     note)


def config_block(thresholds: Sequence[Threshold]) -> str:
    """The calibrated values as an editable ``planner/config.py`` patch.

    Emitted as a patch rather than written into the file: promoting a placeholder
    to ``MEASURED`` is a claim about an experiment, and it should be made by a
    person who saw the run, not by the script that ran it.
    """
    L = ["# --- calibrated from data/calibration_probes.npz -----------------------",
         "# Paste into planner/config.py and flip the matching Provenance entries",
         "# from CALIBRATION_NEEDED to MEASURED. Do NOT flip one whose value below",
         "# is None or whose note says NON-MONOTONE.",
         ""]
    for th in thresholds:
        field = th.parameter.split(".")[-1]
        if th.value_m is None:
            L.append(f"# {field}: NOT SETTLED -- {th.note}")
            continue
        L.append(f"{field}: float = {th.value_m:.3f}"
                 f"   # {th.skill} on {th.family}, {th.n_reps} reps, "
                 f"highest all-pass {th.highest_all_pass_m:.2f} m - one {th.step_m:.2f} m level"
                 + ("" if th.monotone else "   # NON-MONOTONE, see report"))
    L.append("")
    for th in thresholds:
        if th.value_m is None:
            continue
        prov = "MEASURED" if th.monotone else "CALIBRATION_NEEDED"
        L.append(f'_p("{th.parameter}", Provenance.{prov}, "m",')
        L.append(f'   "{th.skill} cleared {th.highest_all_pass_m:.2f} m on all {th.n_reps} repeats of the '
                 f'{th.family} probe; backed off one {th.step_m:.2f} m level")')
    return "\n".join(L)
