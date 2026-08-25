#!/usr/bin/env python
"""Freeze the 20 benchmark tasks x 10 difficulty levels to data/benchmark_frozen.npz.

Usage:
    python scripts/freeze_benchmark.py            # generate, fix, save, hash
    python scripts/freeze_benchmark.py --verify   # also regenerate once more and compare
    python scripts/freeze_benchmark.py --check    # only verify the existing npz against its sha256

No Isaac / torch imports; only numpy.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from terrain_toolkit import paths  # noqa: E402
from terrain_toolkit.freeze import freeze, load_archive, sha256_file  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--verify", action="store_true", help="regenerate twice and assert identical content")
    ap.add_argument("--check", action="store_true", help="verify existing npz against recorded sha256 and exit")
    args = ap.parse_args()

    if args.check:
        expected = paths.FROZEN_SHA256.read_text().split()[0]
        actual = sha256_file(paths.FROZEN_NPZ)
        ok = expected == actual
        print(f"{'OK' if ok else 'MISMATCH'}: {paths.FROZEN_NPZ.name} {actual}")
        z = load_archive(verify=False)
        print(f"content_sha256 {z['content_sha256']}  tasks={len(z['task_names'])} levels={len(z['difficulties'])}")
        return 0 if ok else 1

    freeze(verify_determinism=args.verify)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
