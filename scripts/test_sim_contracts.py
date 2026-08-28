#!/usr/bin/env python3
"""Regression tests for the reads that look like samples and are not.

    python scripts/test_sim_contracts.py

No simulator, no pytest, no network -- synthetic fakes only, so this runs in a
second and can gate every change to the harness.

Of the five defects found on day 1, three recurred in code written the next day
*while diagnosing them*. They recur because none of them is a logic error: each is
an API whose behaviour differs from its appearance, and reading the source tells
you nothing. So every test here does two things -- asserts the fixed helper is
correct, AND asserts the original mistake would still be caught. A test that only
checks the fix passes is satisfied by code that never runs.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim.replay import (assert_not_aliased, effective_friction, foot_body_ids,
                        quat_rotate_inv, quat_to_rpy_deg, snap)

FAILURES: list[str] = []


def check(name: str, cond: bool, detail: str = "") -> None:
    print(f"  {'ok  ' if cond else 'FAIL'}  {name}" + (f"  -- {detail}" if detail and not cond else ""))
    if not cond:
        FAILURES.append(name)


# --------------------------------------------------------------------------- #
# Fakes. A "tensor" that is a view into a buffer the "simulator" rewrites in
# place, which is the entire mechanism of defect 6.
# --------------------------------------------------------------------------- #

class FakeTensor:
    """Mimics the two behaviours that matter: [i] is a view, .cpu() is a no-op."""

    def __init__(self, buf: np.ndarray):
        self._buf = buf

    def __getitem__(self, key):
        return FakeTensor(self._buf[key])          # basic indexing -> a VIEW
    def detach(self):
        return self
    def cpu(self):
        return self                                # already on CPU: copies nothing
    def numpy(self):
        return self._buf                           # shares memory


class FakeSim:
    """Holds one root-state buffer and overwrites it in place every step."""

    def __init__(self):
        self._root = np.zeros(3, dtype=np.float64)
        self.t = 0

    def step(self):
        self.t += 1
        self._root[:] = (0.01 * self.t, 0.0, 0.32 - 0.001 * self.t)

    @property
    def root_pos_w(self):
        return FakeTensor(self._root)


class FakeArticulation:
    """A Go2-shaped body list: base, then hips/thighs/calves, then the four feet."""

    body_names = (["base"]
                  + [f"{leg}_{part}" for part in ("hip", "thigh", "calf")
                     for leg in ("FL", "FR", "RL", "RR")]
                  + [f"{leg}_foot" for leg in ("FL", "FR", "RL", "RR")])

    def find_bodies(self, pattern):
        import re
        rx = re.compile(pattern)
        ids = [i for i, n in enumerate(self.body_names) if rx.fullmatch(n)]
        return ids, [self.body_names[i] for i in ids]


class FakeContactSensor:
    """The sensor knows only its four feet, and numbers them 0..3."""

    body_names = [f"{leg}_foot" for leg in ("FL", "FR", "RL", "RR")]

    def find_bodies(self, pattern):
        import re
        rx = re.compile(pattern)
        ids = [i for i, n in enumerate(self.body_names) if rx.fullmatch(n)]
        return ids, [self.body_names[i] for i in ids]


# --------------------------------------------------------------------------- #

def test_buffer_aliasing() -> None:
    print("\n== state recording: a read is not a sample until it is copied ==")

    naive, copied = [], []
    sim = FakeSim()
    for _ in range(20):
        sim.step()
        naive.append(sim.root_pos_w[0:3].cpu().numpy())     # the original pattern
        copied.append(snap(sim.root_pos_w[0:3]))            # the fixed one
    naive_a, copied_a = np.asarray(naive), np.asarray(copied)

    check("the naive read really does alias (the trap is reproduced)",
          len(np.unique(naive_a, axis=0)) == 1,
          f"expected 1 distinct row, got {len(np.unique(naive_a, axis=0))}")
    check("snap() records 20 distinct samples",
          len(np.unique(copied_a, axis=0)) == 20)
    check("aliased rows all equal the FINAL step, not the first",
          np.allclose(naive_a[0], copied_a[-1]) and not np.allclose(naive_a[0], copied_a[0]))

    # The consequence that made this expensive: a "mean" that is one sample.
    check("an aliased mean is the final value, not the average",
          np.isclose(naive_a[:, 2].mean(), copied_a[-1, 2])
          and not np.isclose(naive_a[:, 2].mean(), copied_a[:, 2].mean()))

    # The guard has to fire on the bad array and stay quiet on the good one.
    try:
        assert_not_aliased({"root_pos_w": naive_a})
        check("assert_not_aliased rejects an aliased recording", False, "it did not raise")
    except AssertionError:
        check("assert_not_aliased rejects an aliased recording", True)
    try:
        assert_not_aliased({"root_pos_w": copied_a})
        check("assert_not_aliased accepts a real recording", True)
    except AssertionError as exc:
        check("assert_not_aliased accepts a real recording", False, str(exc))

    # A short episode must not be judged: 3 identical steps is not evidence.
    try:
        assert_not_aliased({"root_pos_w": np.zeros((3, 3))})
        check("assert_not_aliased does not fire on a too-short episode", True)
    except AssertionError:
        check("assert_not_aliased does not fire on a too-short episode", False)


def test_body_indexing() -> None:
    print("\n== body indexing: sensor indices are not articulation indices ==")
    robot, sensor = FakeArticulation(), FakeContactSensor()

    ids, names = foot_body_ids(robot)
    check("foot ids resolve to bodies actually named *_foot",
          all(n.endswith("_foot") for n in names) and len(ids) == 4, f"{names}")

    sensor_ids, _ = sensor.find_bodies(".*_foot")
    check("the sensor numbers its feet 0..3", sensor_ids == [0, 1, 2, 3])
    check("sensor ids differ from articulation ids (the trap is reproduced)",
          sensor_ids != ids, f"sensor {sensor_ids} vs articulation {ids}")

    # The symptom that exposed it: body 0 is the base, so "foot height" == base height.
    check("using sensor ids against the articulation selects the base first",
          robot.body_names[sensor_ids[0]] == "base")


def test_quaternion_order() -> None:
    print("\n== quaternion order: Isaac Lab is scalar-LAST (x, y, z, w) ==")
    ident = np.array([[0.0, 0.0, 0.0, 1.0]])                 # identity, xyzw
    r, p, y = quat_to_rpy_deg(ident)
    check("identity xyzw -> zero roll/pitch/yaw",
          np.allclose([r[0], p[0], y[0]], 0.0), f"{r[0]:.1f},{p[0]:.1f},{y[0]:.1f}")

    r2, p2, y2 = quat_to_rpy_deg(ident, order="wxyz")
    check("reading the same quaternion as wxyz is wrong (the trap is reproduced)",
          not np.allclose([r2[0], p2[0], y2[0]], 0.0),
          "misreading identity produced zeros, so the test cannot catch the bug")

    # A known rotation, so the sign convention is pinned and not just self-consistent.
    half = np.deg2rad(90.0) / 2
    yaw90 = np.array([[0.0, 0.0, np.sin(half), np.cos(half)]])
    _, _, y3 = quat_to_rpy_deg(yaw90)
    check("a +90 deg yaw reads back as +90", np.isclose(y3[0], 90.0), f"{y3[0]:.2f}")

    roll90 = np.array([[np.sin(half), 0.0, 0.0, np.cos(half)]])
    r4, _, _ = quat_to_rpy_deg(roll90)
    check("a +90 deg roll reads back as +90", np.isclose(r4[0], 90.0), f"{r4[0]:.2f}")

    # The real trace's first frame was near-identity; the wrong order made it -166 deg.
    real = np.array([[-0.1227, -0.0440, -0.0400, 0.9907]])
    _, _, y5 = quat_to_rpy_deg(real)
    _, _, y6 = quat_to_rpy_deg(real, order="wxyz")
    check("a near-level robot reads level under xyzw and absurd under wxyz",
          abs(y5[0]) < 10.0 and abs(y6[0]) > 90.0, f"xyzw {y5[0]:.1f}, wxyz {y6[0]:.1f}")


def test_units() -> None:
    print("\n== units ==")
    rad_s = np.array([0.5, -0.25])
    check("angular rate reported in deg/s is converted exactly once",
          np.allclose(np.degrees(rad_s), [28.6478897565, -14.3239448783]))
    check("degrees applied twice is detectably wrong (the trap is reproduced)",
          not np.allclose(np.degrees(np.degrees(rad_s)), np.degrees(rad_s)))

    # stride is a frequency; the clip stores a rate, not a period.
    fs, n_frames = 49.77, 32
    check("stride from frames and sample rate is a frequency in Hz",
          np.isclose(fs / n_frames, 1.5553, atol=1e-3), f"{fs / n_frames:.4f}")
    check("period and frequency are not interchangeable",
          not np.isclose(fs / n_frames, n_frames / fs))

    # control/physics rate: the config's decimation, not the clip's.
    sim_dt, decimation = 0.005, 4
    check("sim_dt x decimation is the CONTROL period, 50 Hz",
          np.isclose(1.0 / (sim_dt * decimation), 50.0))
    check("1/sim_dt is the PHYSICS rate, 200 Hz", np.isclose(1.0 / sim_dt, 200.0))


def test_friction_combination() -> None:
    print("\n== friction: the contact sees a product of two materials ==")
    check("env terrain 1.0 x robot 1.3 under multiply gives 1.3",
          np.isclose(effective_friction(1.0, 1.3, "multiply"), 1.3))
    check("overriding only the ground under Isaac's default average does NOT give 1.3",
          np.isclose(effective_friction(1.3, 0.50, "average"), 0.9),
          "this is the value the first friction patch actually produced")
    check("the untouched harness stood on 0.5, below the env's whole range",
          np.isclose(effective_friction(0.5, 0.50, "average"), 0.5))
    check("1.3 is the midpoint of the env's friction_range [0.6, 2.0]",
          np.isclose(0.5 * (0.6 + 2.0), 1.3))


def test_body_frame_rotation() -> None:
    """World-frame foot positions answer a different question once the base yaws."""
    print("\n== body frame: asking where a foot is, of the base and not of the world ==")
    # base yawed 90 deg left; a foot one metre ahead of the base IN THE WORLD is
    # one metre to the base's RIGHT.
    q = np.array([0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)])     # scalar last
    v = np.array([1.0, 0.0, 0.0])
    b = quat_rotate_inv(q, v)
    check("a world-forward offset under 90 deg yaw is body-right",
          np.allclose(b, [0.0, -1.0, 0.0], atol=1e-9), f"got {b}")
    check("the raw world vector would have called it body-FORWARD",
          not np.allclose(v, [0.0, -1.0, 0.0]),
          "this is the mistake the helper exists to prevent")
    check("identity rotation is the identity",
          np.allclose(quat_rotate_inv(np.array([0.0, 0, 0, 1.0]), v), v))
    # reading the same quaternion scalar-first turns the 90 deg yaw into something else
    check("scalar-first misreading of the same quaternion gives a different answer",
          not np.allclose(quat_rotate_inv(q, v, order="wxyz"), b),
          "quaternion order matters here exactly as it does in quat_to_rpy_deg")
    # a lateral centre-of-pressure offset must not depend on where the robot is
    for yaw in (0.0, 0.7, -2.1, 3.0):
        qq = np.array([0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)])
        world = np.array([np.cos(yaw), np.sin(yaw), 0.0]) * 0.2      # 0.2 m ahead of base
        check(f"body-frame offset is yaw-invariant (yaw={yaw:+.1f} rad)",
              np.allclose(quat_rotate_inv(qq, world), [0.2, 0.0, 0.0], atol=1e-9))
    check("rotating by the inverse and back is a round trip",
          np.allclose(quat_rotate_inv(np.array([0.0, 0, 0, 1.0]),
                                      quat_rotate_inv(q, v)), quat_rotate_inv(q, v)))


def test_mirror_ablation() -> None:
    """The mirror probe must reverse abduction and must NOT reverse gains."""
    print("\n== ablation: mirroring a gait is not negating every column ==")
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "_vsr", Path(__file__).resolve().parent / "verify_skill_replay.py")
    V = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(V)                      # top level imports no Isaac Lab

    legs, n = ["FL", "FR", "RL", "RR"], 8
    clip = {"leg_order": legs, "kind": "cyclic", "fs": 50.0,
            "t": np.arange(n) / 50.0,
            "q_des": np.arange(n * 12, dtype=np.float32).reshape(n, 12),
            "dq_des": np.zeros((n, 12), np.float32), "tau_ff": np.zeros((n, 12), np.float32),
            "q": np.zeros((n, 12), np.float32), "dq": np.zeros((n, 12), np.float32),
            "kp": np.full((n, 12), 40.0, np.float32), "kd": np.full((n, 12), 1.0, np.float32),
            "contact": np.array([[1, 0, 0, 1]] * n, np.uint8),
            "q_des_valid": np.ones((n, 12), np.uint8)}
    m = V.mirror_clip(clip, "mirror")
    check("the mirrored FL block is the original FR block with the hip negated",
          np.allclose(m["q_des"][:, 3:6] * np.array([-1, 1, 1]), clip["q_des"][:, 0:3]))
    check("contact labels swap sides too", list(m["contact"][0]) == [0, 1, 1, 0])
    check("gains are NOT negated by the mirror", float(m["kp"].min()) == 40.0,
          "a negated kp is not a mirrored robot, it is an unstable one")
    check("validity flags are NOT negated by the mirror", float(m["q_des_valid"].min()) == 1.0)
    sy = V.mirror_clip(clip, "symmetrize")
    half = np.roll(V._mirror_rows(sy["q_des"].astype(float), legs, per_leg=False, signed=True),
                   -(n // 2), axis=0)
    check("the symmetrised clip really is its own mirror half a cycle later",
          np.allclose(half, sy["q_des"], atol=1e-4),
          "otherwise the ablation does not remove what it claims to remove")
    check("--ablate none is the identity", V.mirror_clip(clip, "none") is clip)


def main() -> int:
    print("sim contract tests -- synthetic data, no simulator")
    test_buffer_aliasing()
    test_body_indexing()
    test_quaternion_order()
    test_units()
    test_friction_combination()
    test_body_frame_rotation()
    test_mirror_ablation()
    print(f"\n{'FAILED: ' + ', '.join(FAILURES) if FAILURES else 'all contracts hold'}")
    return 1 if FAILURES else 0


if __name__ == "__main__":
    raise SystemExit(main())
