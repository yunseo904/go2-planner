#!/usr/bin/env python3
"""Re-extraction gate (CLAUDE.md 1): rebuild the clips and prove the archive reproduces.

    python scripts/gate_reextract.py            # run the gate
    python scripts/gate_reextract.py --dry-run  # check readiness, build nothing

Runs ``scripts/extract_skill_clips.py`` **unmodified** and asks one question: does
the rebuilt ``data/skill_clips.npz`` still hash to the frozen ``npz_sha256``?

Two reasons this wrapper exists rather than just running the extractor:

1. ``extract_skill_clips.py`` overwrites ``data/skill_clips.sha256`` with the digest
   of what it just wrote, so its own ``--verify`` compares the new file against the
   new hash and *always* passes after a rebuild.  It cannot detect a change.  The
   frozen digest is therefore captured here before the rebuild and compared after.
2. A rebuild overwrites the frozen archive in place.  Everything is snapshotted and
   restored, so a mismatching rebuild cannot destroy the reference; the rebuilt copy
   is kept beside it for inspection.

On mismatch the archive is compared array by array, because a whole-file hash cannot
tell "the extraction changed" from "the zip container was framed differently".
"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from terrain_toolkit.paths import (  # noqa: E402
    CURATED_ROOT,
    PROJECT_ROOT,
    SKILL_CLIPS_MD,
    SKILL_CLIPS_META_JSON,
    SKILL_CLIPS_NPZ,
    SKILL_CLIPS_SHA256,
)

EXTRACTOR = PROJECT_ROOT / "scripts" / "extract_skill_clips.py"
GUARDED = (SKILL_CLIPS_NPZ, SKILL_CLIPS_SHA256, SKILL_CLIPS_META_JSON, SKILL_CLIPS_MD)


def sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for block in iter(lambda: fh.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def compare_contents(ref: Path, new: Path) -> list[str]:
    """Array-by-array diff, so a container difference is distinguishable from a data one."""
    a = np.load(ref, allow_pickle=False)
    b = np.load(new, allow_pickle=False)
    ka, kb = set(a.files), set(b.files)
    lines = []
    for k in sorted(ka - kb):
        lines.append(f"  only in frozen : {k}")
    for k in sorted(kb - ka):
        lines.append(f"  only in rebuilt: {k}")
    n_same = 0
    for k in sorted(ka & kb):
        x, y = a[k], b[k]
        if x.shape != y.shape:
            lines.append(f"  {k}: shape {x.shape} -> {y.shape}")
            continue
        if x.dtype != y.dtype:
            lines.append(f"  {k}: dtype {x.dtype} -> {y.dtype}")
            continue
        if x.dtype.kind in "fc":
            d = np.abs(np.nan_to_num(x, nan=0.0) - np.nan_to_num(y, nan=0.0))
            nan_moved = int((np.isnan(x) != np.isnan(y)).sum())
            if d.max() > 0 or nan_moved:
                lines.append(f"  {k}: max|diff| {d.max():.3e}  mean {d.mean():.3e}"
                             f"{f'  NaN pattern moved in {nan_moved} cells' if nan_moved else ''}")
            else:
                n_same += 1
        elif not np.array_equal(x, y):
            lines.append(f"  {k}: {int((x != y).sum())} of {x.size} cells differ")
        else:
            n_same += 1
    lines.append(f"  ({n_same} of {len(ka & kb)} arrays bit-identical)")
    return lines


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry-run", action="store_true", help="report readiness and exit")
    args = ap.parse_args()

    if not SKILL_CLIPS_NPZ.is_file() or not SKILL_CLIPS_SHA256.is_file():
        print("[gate] FAIL: no frozen archive to compare against", file=sys.stderr)
        return 2
    frozen_digest = SKILL_CLIPS_SHA256.read_text().split()[0]
    on_disk = sha256_of(SKILL_CLIPS_NPZ)
    meta_digest = json.loads(SKILL_CLIPS_META_JSON.read_text())["npz_sha256"]
    print(f"[gate] frozen sha256   {frozen_digest}")
    print(f"[gate] archive on disk {on_disk}  {'OK' if on_disk == frozen_digest else 'MISMATCH'}")
    print(f"[gate] meta npz_sha256 {meta_digest}  {'OK' if meta_digest == frozen_digest else 'MISMATCH'}")
    if on_disk != frozen_digest or meta_digest != frozen_digest:
        print("[gate] FAIL: the reference is already inconsistent; resolve that first", file=sys.stderr)
        return 2

    curated_ok = (CURATED_ROOT / "INDEX.md").is_file()
    print(f"[gate] curated root    {CURATED_ROOT}  {'present' if curated_ok else 'MISSING'}")
    print(f"[gate] numpy           {np.__version__}  (zip mtimes are zeroed; hash tracks content)")
    if args.dry_run:
        print("[gate] dry run: " + ("ready to run the gate" if curated_ok
                                    else "blocked -- rsync curated/ to this machine first"))
        return 0 if curated_ok else 1
    if not curated_ok:
        print(f"[gate] FAIL: curated logs not at {CURATED_ROOT}; "
              f"rsync them or set $GO2_CURATED_ROOT", file=sys.stderr)
        return 1

    snap = Path(tempfile.mkdtemp(prefix="gate_reextract_"))
    for p in GUARDED:
        if p.is_file():
            shutil.copy2(p, snap / p.name)
    print(f"[gate] snapshot of the frozen files -> {snap}")

    rebuilt_dir = PROJECT_ROOT / "outputs" / "reextract_gate"
    rebuilt_dir.mkdir(parents=True, exist_ok=True)
    try:
        print(f"[gate] running {EXTRACTOR.name} unmodified ...\n" + "-" * 72)
        r = subprocess.run([sys.executable, str(EXTRACTOR)], cwd=str(PROJECT_ROOT))
        print("-" * 72)
        if r.returncode != 0:
            print(f"[gate] FAIL: extractor exited {r.returncode}", file=sys.stderr)
            return 3
        new_digest = sha256_of(SKILL_CLIPS_NPZ)
        for p in GUARDED:
            if p.is_file():
                shutil.copy2(p, rebuilt_dir / p.name)
        print(f"[gate] rebuilt sha256  {new_digest}")
        if new_digest == frozen_digest:
            print(f"[gate] PASS: reproduced {frozen_digest[:8]}… exactly. "
                  f"Clip definitions may now be changed.")
            return 0
        print(f"[gate] FAIL: rebuilt {new_digest[:8]}… != frozen {frozen_digest[:8]}…")
        print("[gate] array-by-array comparison (frozen -> rebuilt):")
        for line in compare_contents(snap / SKILL_CLIPS_NPZ.name, SKILL_CLIPS_NPZ):
            print(line)
        print("[gate] regard the cause as unknown until it is explained. "
              "Do not change any clip definition first.")
        return 1
    finally:
        for p in GUARDED:
            src = snap / p.name
            if src.is_file():
                shutil.copy2(src, p)
        print(f"[gate] frozen files restored from the snapshot; "
              f"rebuilt copies kept in {rebuilt_dir}")
        post = sha256_of(SKILL_CLIPS_NPZ)
        print(f"[gate] archive now     {post}  "
              f"{'OK' if post == frozen_digest else 'RESTORE FAILED'}")


if __name__ == "__main__":
    raise SystemExit(main())
