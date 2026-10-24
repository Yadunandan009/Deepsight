#!/usr/bin/env python3
"""Tests for stonefish_bluerov2/consistency_gate.py. Run: python3 eval/test_consistency_gate.py"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "stonefish_bluerov2"))
from stonefish_bluerov2.consistency_gate import ConsistencyGate  # noqa: E402


def test_width_never_exceeds_cap_and_grows_with_path():
    g = ConsistencyGate(floor=1.0, k_drift=0.01, cap=5.0)
    g.on_probation = False
    assert g.width() == 1.0
    for i in range(100):
        g.update_reference(float(i), 0.0)       # 99 m
    assert abs(g.width() - (1.0 + 0.99)) < 1e-9
    for i in range(100, 1000):
        g.update_reference(float(i), 0.0)
    assert g.width() == 5.0                      # capped, never looser than the old gate


def test_jump_in_reference_is_not_travel():
    g = ConsistencyGate(floor=1.0, k_drift=0.01)
    g.update_reference(0.0, 0.0)
    g.update_reference(50.0, 0.0)                # a 50 m step: a reset of the reference, not path
    assert g.path == 0.0


def test_probation_holds_width_at_floor_and_requires_poses_and_spread():
    g = ConsistencyGate(floor=1.0, k_drift=0.01, probation_poses=3, probation_spread=8.0)
    for i in range(300):
        g.update_reference(float(i), 0.0)
    assert g.on_probation and g.width() == 1.0   # floor, though 299 m travelled
    g.alignment_changed(spread=5.0)              # poorly conditioned window
    for _ in range(10):
        ok, _ = g.check(0.5)
        assert ok
    assert not g.may_publish()                   # enough poses, not enough spread
    g.alignment_changed(spread=9.0)
    assert g.may_publish() and not g.on_probation
    assert g.width() > 1.0                       # trusted now: floor + k * s


def test_failed_pose_on_probation_restarts_the_count():
    g = ConsistencyGate(floor=1.0, probation_poses=3, probation_spread=0.0)
    g.alignment_changed(spread=10.0)
    for d in (0.2, 0.2, 3.0, 0.2, 0.2):
        g.check(d)
    assert not g.may_publish()                   # only two since the failure
    g.check(0.2)
    assert g.may_publish()


def test_reset_puts_the_gate_back_on_probation():
    g = ConsistencyGate(floor=1.0, probation_poses=1, probation_spread=0.0)
    g.alignment_changed(spread=10.0)
    g.check(0.1)
    assert g.may_publish()
    g.alignment_reset()
    assert g.on_probation and not g.may_publish()


def test_bad_arguments():
    for kw in (dict(floor=0.0), dict(floor=6.0, cap=5.0), dict(k_drift=-1.0)):
        try:
            ConsistencyGate(**kw)
        except ValueError:
            continue
        raise AssertionError(f"accepted {kw}")


if __name__ == "__main__":
    n = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            fn()
            n += 1
            print("ok", name)
    print(f"{n} tests passed")
