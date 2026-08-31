#!/usr/bin/env python3
"""Reduce an --score inplace sweep against its own flat control.

    python3 scripts/reduce_turn_inplace.py outputs/turn_inplace_flat.csv \
                                           outputs/turn_inplace_obstacle.csv

The protocol's own rule -- a level passes only if EVERY repeat passed -- cannot be
applied to TURN unmodified.  The repeats vary the clip entry phase, and TURN fails 3 of
its 5 entry phases **on flat ground**, so every level would read 0/5 for a reason that
has nothing to do with terrain.  That is the same shape of error `lip_failure.md` 4 found
in STEP_TROT_MAX: a score that comes out identical at 0.02 m and 0.30 m is not measuring
the obstacle.

So the flat run is the control for its own phase.  Phases that fail on flat are dropped
from the denominator, and a level passes if every surviving phase still turns.
"""
from __future__ import annotations

import collections
import csv
import sys
from pathlib import Path

import numpy as np


def phases_that_work(flat_csv: Path) -> list:
    rows = list(csv.DictReader(open(flat_csv)))
    by = collections.defaultdict(list)
    for x in rows:
        by[int(x["rep"])].append(int(x["reached"]))
    return sorted(r for r, v in by.items() if all(v))


def main() -> int:
    if len(sys.argv) < 3:
        raise SystemExit(__doc__)
    flat, sweep = Path(sys.argv[1]), Path(sys.argv[2])
    good = phases_that_work(flat)
    fl = list(csv.DictReader(open(flat)))
    ent = sorted({x["entry_frame"] for x in fl if int(x["rep"]) in good})
    fyaw = [float(x["yaw_ok_deg"]) for x in fl if int(x["rep"]) in good]
    fdr = [float(x["drift_max_m"]) for x in fl if int(x["rep"]) in good]
    print(f"flat control ({flat.name}): {len(good)}/5 entry phases turn "
          f"(frames {', '.join(ent)}); on those, yaw {min(fyaw):.1f}-{max(fyaw):.1f} deg, "
          f"drift {min(fdr):.2f}-{max(fdr):.2f} m")
    if not good:
        print("  no phase turns on flat: nothing downstream can be attributed to terrain")
        return 1

    by = collections.defaultdict(dict)
    for x in csv.DictReader(open(sweep)):
        by[(x["family"], float(x["level_m"]))][int(x["rep"])] = x
    print(f"\n{'family':10s} {'level':>7s} {'pass':>7s} {'yaw completed':>18s} "
          f"{'drift m':>13s}  settle")
    passing = collections.defaultdict(list)
    for fam, lvl in sorted(by, key=lambda t: (t[0], t[1])):
        v = [by[(fam, lvl)][r] for r in good if r in by[(fam, lvl)]]
        if not v:
            continue
        n = sum(int(x["reached"]) for x in v)
        y = [float(x["yaw_ok_deg"]) for x in v]
        d = [float(x["drift_max_m"]) for x in v]
        st = all(int(x.get("settle_ok", 1)) for x in by[(fam, lvl)].values())
        if n == len(v):
            passing[fam].append(lvl)
        print(f"{fam:10s} {lvl:7.3f} {n:3d}/{len(v):<3d} {min(y):7.1f}-{max(y):<7.1f} "
              f"{min(d):5.2f}-{max(d):5.2f}  {'ok' if st else 'NO STANCE'}"
              f"{'   PASS' if n == len(v) else ''}")
    print()
    for fam in sorted({f for f, _ in by}):
        ls = passing.get(fam)
        if not ls:
            floor = min(l for f, l in by if f == fam)
            print(f"{fam:10s}: nothing passes. The limit is BELOW the ladder's floor "
                  f"({floor:g}) -- it is not measurable on this archive.")
        else:
            run = [ls[0]]
            for a, b in zip(ls, ls[1:]):
                if b == ls[ls.index(a) + 1] and ls.index(b) == ls.index(a) + 1:
                    run.append(b)
            print(f"{fam:10s}: passes {ls}; top of the unbroken run = {max(run):g}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
