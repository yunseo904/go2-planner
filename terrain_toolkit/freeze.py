"""Generate every benchmark terrain (tasks x difficulty levels), apply
``fix_terrain`` and freeze the result to ``data/benchmark_frozen.npz``.

Archive layout (all arrays indexed ``[task, level, ...]``):

    height_fields            int16   (T, L, X, Y)  post-fix, units of ``vertical_scale``
    height_fields_before_fix int16   (T, L, X, Y)  straight out of ``set_terrain``
    goals                    float64 (T, L, G, 2)  metres, post-fix, (x, y) in the sub-terrain frame
    goals_before_fix         float64 (T, L, G, 2)
    set_idx                  int64   (T, L)        return value of ``set_terrain``
    seeds                    int64   (T, L)
    difficulties             float64 (L,)          ``row / (num_rows - 1)``
    variations               float64 (T,)          ``col / num_cols``
    task_names               <U      (T,)          dispatcher order
    fix_descs                <U      (T, L)        what ``fix_terrain`` changed ("" if nothing)
    difficulty_scaling       float64 (T, 2)        (min_d, max_d) used by ``scale_difficulty``
    scaled_difficulties      float64 (T, L)        difficulty after ``scale_difficulty``
    + scalar metadata (see ``META_KEYS``)

x is the forward/travel axis (``terrain.width`` cells upstream), y is lateral.
The robot spawns at ``(spawn_x, spawn_y)`` metres.
"""

from __future__ import annotations

import hashlib
import json
import subprocess
import warnings
from pathlib import Path
from typing import Dict, Optional

import numpy as np

from . import paths
from .fix_terrain import fix_terrain
from .stub import (BenchmarkTerrainCfg, dispatcher_task_names, defined_task_names,
                   generate, load_benchmark_module)

META_KEYS = ("horizontal_scale", "vertical_scale", "terrain_length_m", "terrain_width_m",
             "x_cells", "y_cells", "num_goals", "num_rows", "num_cols", "spawn_x", "spawn_y",
             "upstream_commit", "upstream_module_sha256", "content_sha256")


def sha256_file(path: Path, chunk: int = 1 << 20) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(chunk), b""):
            h.update(block)
    return h.hexdigest()


def content_sha256(arrays: Dict[str, np.ndarray]) -> str:
    """Hash of array contents (independent of the npz container bytes)."""
    h = hashlib.sha256()
    for k in sorted(arrays):
        a = np.ascontiguousarray(arrays[k])
        h.update(k.encode())
        h.update(str(a.dtype).encode())
        h.update(str(a.shape).encode())
        h.update(a.tobytes())
    return h.hexdigest()


def upstream_git_info(root: Path = paths.UPSTREAM_ROOT) -> Dict[str, str]:
    def git(*args) -> str:
        return subprocess.run(["git", "-C", str(root), *args], check=True,
                              capture_output=True, text=True).stdout.strip()

    info = {"commit": git("rev-parse", "HEAD")}
    try:
        info["describe"] = git("describe", "--always", "--dirty", "--tags")
    except subprocess.CalledProcessError:
        info["describe"] = info["commit"][:12]
    info["dirty"] = "yes" if git("status", "--porcelain", "--untracked-files=no") else "no"
    rel = paths.BENCHMARK_MODULE_PATH.relative_to(root)
    info["benchmark_module"] = str(rel)
    info["benchmark_module_blob"] = git("rev-parse", f"HEAD:{rel.as_posix()}")
    info["benchmark_module_sha256"] = sha256_file(paths.BENCHMARK_MODULE_PATH)
    return info


def build_archive(cfg: BenchmarkTerrainCfg = BenchmarkTerrainCfg(), verbose: bool = True) -> Dict[str, np.ndarray]:
    module = load_benchmark_module()
    task_names = dispatcher_task_names(module)
    T, L = cfg.num_cols, cfg.num_rows
    assert len(task_names) == T, f"dispatcher has {len(task_names)} tasks but cfg.num_cols={T}"
    extra = sorted(set(defined_task_names(module)) - set(task_names))
    if verbose and extra:
        print(f"[freeze] NOTE: defined but not dispatched: {extra}")

    X, Y, G = cfg.x_cells, cfg.y_cells, cfg.num_goals
    hf = np.zeros((T, L, X, Y), dtype=np.int16)
    hf0 = np.zeros_like(hf)
    goals = np.zeros((T, L, G, 2), dtype=np.float64)
    goals0 = np.zeros_like(goals)
    set_idx = np.zeros((T, L), dtype=np.int64)
    seeds = np.zeros((T, L), dtype=np.int64)
    fix_descs = np.full((T, L), "", dtype=object)
    scaling = np.array([module.difficulty_scaling[t] for t in task_names], dtype=np.float64)
    difficulties = np.array([cfg.difficulty(r) for r in range(L)])
    variations = np.array([cfg.variation(c) for c in range(T)])

    with warnings.catch_warnings():
        # upstream sphere_bump* terrains overflow int16 inside np.ogrid -> sqrt(NaN) warnings; faithful, so silence
        warnings.simplefilter("ignore", RuntimeWarning)
        for c in range(T):
            for r in range(L):
                t = generate(module, cfg, c, r)
                assert t.idx == c
                hf0[c, r] = t.height_field_raw
                goals0[c, r] = t.goals
                seeds[c, r] = cfg.seed(cfg.variation(c), cfg.difficulty(r))
                set_idx[c, r] = t.idx
                fix_descs[c, r] = fix_terrain(t)
                hf[c, r] = t.height_field_raw
                goals[c, r] = t.goals
            if verbose:
                n_fixed = sum(1 for r in range(L) if fix_descs[c, r])
                print(f"[freeze] {c:2d} {task_names[c]:<36s} fixed {n_fixed:2d}/{L}")

    arrays: Dict[str, np.ndarray] = {
        "height_fields": hf,
        "height_fields_before_fix": hf0,
        "goals": goals,
        "goals_before_fix": goals0,
        "set_idx": set_idx,
        "seeds": seeds,
        "difficulties": difficulties,
        "variations": variations,
        "task_names": np.array(task_names),
        "fix_descs": fix_descs.astype(str),
        "difficulty_scaling": scaling,
        "scaled_difficulties": difficulties[None, :] * (scaling[:, 1:2] - scaling[:, 0:1]) + scaling[:, 0:1],
    }
    git = upstream_git_info()
    meta = {
        "horizontal_scale": cfg.horizontal_scale,
        "vertical_scale": cfg.vertical_scale,
        "terrain_length_m": cfg.terrain_length,
        "terrain_width_m": cfg.terrain_width,
        "x_cells": X, "y_cells": Y,
        "num_goals": G, "num_rows": L, "num_cols": T,
        "spawn_x": cfg.spawn_x, "spawn_y": cfg.spawn_y,
        "upstream_commit": git["commit"],
        "upstream_module_sha256": git["benchmark_module_sha256"],
        "content_sha256": content_sha256(arrays),
    }
    for k, v in meta.items():
        arrays[k] = np.array(v)
    arrays["_git_info"] = np.array(json.dumps(git))
    return arrays


def save_archive(arrays: Dict[str, np.ndarray], npz_path: Path = paths.FROZEN_NPZ) -> str:
    npz_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(npz_path, **arrays)
    digest = sha256_file(npz_path)
    paths.FROZEN_SHA256.write_text(f"{digest}  {npz_path.name}\n")
    git = json.loads(str(arrays["_git_info"]))
    lines = [git["commit"]] + [f"{k}: {v}" for k, v in git.items() if k != "commit"]
    paths.UPSTREAM_COMMIT_TXT.write_text("\n".join(lines) + "\n")
    meta = {k: arrays[k].item() for k in META_KEYS}
    meta["npz_sha256"] = digest
    meta["task_names"] = arrays["task_names"].tolist()
    meta["shapes"] = {k: list(v.shape) for k, v in arrays.items() if v.ndim > 0}
    meta["fix_desc_counts"] = {
        t: int(np.sum(arrays["fix_descs"][i] != "")) for i, t in enumerate(meta["task_names"])
    }
    paths.FROZEN_META_JSON.write_text(json.dumps(meta, indent=2) + "\n")
    return digest


def load_archive(npz_path: Path = paths.FROZEN_NPZ, verify: bool = True) -> Dict[str, np.ndarray]:
    if verify and paths.FROZEN_SHA256.is_file():
        expected = paths.FROZEN_SHA256.read_text().split()[0]
        actual = sha256_file(npz_path)
        if actual != expected:
            raise RuntimeError(f"{npz_path.name} sha256 mismatch: {actual} != {expected} (re-run freeze?)")
    with np.load(npz_path, allow_pickle=False) as z:
        return {k: z[k] for k in z.files}


def freeze(cfg: BenchmarkTerrainCfg = BenchmarkTerrainCfg(), verify_determinism: bool = False,
           verbose: bool = True) -> str:
    arrays = build_archive(cfg, verbose=verbose)
    if verify_determinism:
        again = build_archive(cfg, verbose=False)
        assert again["content_sha256"] == arrays["content_sha256"], "regeneration is not deterministic!"
        if verbose:
            print("[freeze] determinism check passed (second generation identical)")
    digest = save_archive(arrays)
    if verbose:
        print(f"[freeze] wrote {paths.FROZEN_NPZ} ({paths.FROZEN_NPZ.stat().st_size / 1e6:.1f} MB)")
        print(f"[freeze] sha256 {digest}")
        print(f"[freeze] upstream {arrays['upstream_commit']}")
    return digest
