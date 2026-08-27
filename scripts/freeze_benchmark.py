#!/usr/bin/env python
"""Freeze the 20 benchmark tasks x 10 difficulty levels to data/benchmark_frozen.npz.

Usage:
    python scripts/freeze_benchmark.py --check    # verify the existing npz; READ-ONLY, use this
    python scripts/freeze_benchmark.py            # generate, fix, save, hash  (refuses to overwrite)
    python scripts/freeze_benchmark.py --verify   # regenerate TWICE and compare  (refuses to overwrite)
    python scripts/freeze_benchmark.py --force    # overwrite an existing archive; CLAUDE.md s2 forbids
                                                  # regenerating the benchmark, so this needs a reason

--verify is NOT a read-only check: it regenerates. --check is the read-only one.

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
    ap.add_argument("--check", action="store_true", help="verify existing npz against recorded sha256 and exit (read-only)")
    ap.add_argument("--force", action="store_true", help="allow overwriting an existing frozen archive")
    args = ap.parse_args()

    if args.check:
        expected = paths.FROZEN_SHA256.read_text().split()[0]
        actual = sha256_file(paths.FROZEN_NPZ)
        ok = expected == actual
        print(f"{'OK' if ok else 'MISMATCH'}: {paths.FROZEN_NPZ.name} {actual}")
        z = load_archive(verify=False)
        print(f"content_sha256 {z['content_sha256']}  tasks={len(z['task_names'])} levels={len(z['difficulties'])}")
        return 0 if ok else 1

    if paths.FROZEN_NPZ.is_file() and not args.force:
        print(f"refusing to overwrite {paths.FROZEN_NPZ}: it is the frozen benchmark and CLAUDE.md "
              f"section 2 says do not regenerate it.\n"
              f"  to VERIFY it, use --check (read-only)\n"
              f"  to regenerate anyway, pass --force", file=sys.stderr)
        return 2

    freeze(verify_determinism=args.verify)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
