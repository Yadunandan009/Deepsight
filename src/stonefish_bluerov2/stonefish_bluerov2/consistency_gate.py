"""Consistency gate for the SLAM bridge: how far may an aligned SLAM pose sit from
the dead-reckoned reference before it is refused?

WHY THIS EXISTS. The bridge's gate was a constant 5 m. Measured on eleven flights
(docs/paper/MANUSCRIPT_DRAFT_20261022.md, Section 4.3.3; eval/gate_replay.py):

  * the SLAM-free reference (dead reckoning) was never more than 0.65 m from the
    truth on a lap, yet in 7 of 11 flights the published SLAM pose reached
    4.69 to 5.00 m from it, so the fused error was set by the gate's width;
  * published poses within 1 m of the truth were never more than 1.30 m from the
    reference, and 98 % were within 1.0 m;
  * poses more than 1 m from the truth: a 1.5 m gate would have refused 76 %,
    a 1.0 m gate 97 %, the 5 m gate 0.02 %.

WHAT IT DOES. The allowed disagreement is

    width(s) = min(cap, floor + k_drift * s)

with s the path length (m) the reference has travelled. The floor is the margin
for the reference's and SLAM's own noise, k_drift is the share of path length the
reference may drift, and the cap is the old fixed gate, so the new gate is never
looser than the one it replaces.

WHAT IT DOES NOT DO, honestly. It does not calibrate the reference filter's
covariance. That covariance is far too large (position variance 1.8 to 62 m^2
over a mission against a real error under 0.65 m, one stage-1 bag) and is not
used. The width is an error budget in metres, set from measured dead-reckoning
error on one lap of this scenario, not derived for any other unit or route. A
unit with worse DVL error or a long straight transect needs a larger k_drift.

A SECOND PROTECTION, probation. A freshly locked alignment is fitted on a short
window and extrapolates badly (first poses after a re-lock were 1.2 to 2.9 m
wrong). While on probation the bridge keeps the gate at its floor and withholds
publication until `probation_poses` consecutive poses have passed it AND the fit
window had at least `probation_spread` m of horizontal spread.

Pure Python, no ROS, so eval/gate_replay.py and the tests can import it.
"""
import math


class ConsistencyGate:
    def __init__(self, floor=1.0, k_drift=0.0, cap=5.0,
                 probation_poses=25, probation_spread=8.0):
        if not (0.0 < floor <= cap):
            raise ValueError(f'need 0 < floor <= cap, got {floor}, {cap}')
        if k_drift < 0.0:
            raise ValueError(f'k_drift must be >= 0, got {k_drift}')
        self.floor = float(floor)
        self.k_drift = float(k_drift)
        self.cap = float(cap)
        self.probation_poses = int(probation_poses)
        self.probation_spread = float(probation_spread)
        self.path = 0.0               # m travelled by the reference
        self._last = None
        self.on_probation = True
        self._passed = 0
        self._spread = 0.0

    # --- reference path length -------------------------------------------
    def update_reference(self, x, y):
        """Feed every reference position. A step above 20 m is a jump in the
        reference, not travel, and is not counted."""
        if self._last is not None:
            d = math.hypot(x - self._last[0], y - self._last[1])
            if d < 20.0:
                self.path += d
        self._last = (x, y)

    # --- width -----------------------------------------------------------
    def width(self):
        w = self.floor if self.on_probation else self.floor + self.k_drift * self.path
        return min(self.cap, w)

    # --- decisions -------------------------------------------------------
    def check(self, dist):
        """Return (accepted, width) for a pose `dist` m from the reference.
        On probation a pose that passes counts toward release; a pose that
        fails restarts the count."""
        w = self.width()
        ok = dist <= w
        if self.on_probation:
            if ok:
                self._passed += 1
            else:
                self._passed = 0
        return ok, w

    def may_publish(self):
        """False while on probation: poses are checked but not published."""
        if self.on_probation and self._passed >= self.probation_poses \
                and self._spread >= self.probation_spread:
            self.on_probation = False
        return not self.on_probation

    # --- alignment events ------------------------------------------------
    def alignment_changed(self, spread):
        """Call after every successful fit with the window's horizontal spread
        (m). A first lock or a re-lock puts the gate on probation; a periodic
        refit of an alignment already in force only updates the spread."""
        self._spread = float(spread)

    def alignment_reset(self):
        """Call on every reset (jump, map collapse, stale, drift cap)."""
        self.on_probation = True
        self._passed = 0
        self._spread = 0.0
