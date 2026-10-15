"""ENU -> NED for the ORB-SLAM3 pose.

`/bluerov2/robot_pose_slam` is published by the ORB-SLAM3 ROS2 wrapper in
REP-103: x forward, y left, z up. Everything else in this workspace --
the Stonefish truth odometry, the EKFs, the DVL, the depth bridge, the
controller -- is NED: x forward, y right, z down. `D_BASE = 22.0` is 22 m
of depth, not altitude.

Nothing converted between them, and the symptom was not obvious: the
bridge fits a rotation in the HORIZONTAL PLANE, where the difference
shows up as y -> -y, which is a reflection no rotation can represent. So
the bridge hunted a transform that did not exist for the whole mission,
re-aligning 13-14 times a run, and the "SLAM costs 8-10x" result followed
from that.

In 3D it is not a reflection at all:

    (x, y, z) -> (x, -y, -z)      det = +1, a 180 deg rotation about x

Measured on three runs (eval/enu_ned_check.py). Fitting the one yaw and
one scale the bridge already fits:

    as published   residual 18.1-18.7 m   scale 0.06-0.32
    after this     residual  1.6- 2.7 m   scale 0.95-0.98

See docs/SLAM_FRAME_ENU_NED.md for how this was separated from the other
candidate cause (a swapped stereo pair) on evidence.

Orientation transforms by conjugation, q_r * q * q_r^-1 with q_r the
180 deg x rotation, which for a unit x-axis quaternion reduces to
negating the y and z components -- the same rule as position. A pure yaw
psi becomes -psi, matching the measured net SLAM yaw of +453 deg against
a true -452 deg.
"""

__all__ = ["enu_to_ned_xyz", "enu_to_ned_quat", "enu_to_ned_pose",
           "enu_to_ned_pose_inplace", "FRAME_NOTE"]

FRAME_NOTE = ("SLAM pose treated as ENU (z up) and converted to this "
              "workspace's NED (z down): y,z negated, yaw inverted")


def enu_to_ned_xyz(x, y, z):
    """Position, ENU -> NED."""
    return x, -y, -z


def enu_to_ned_quat(w, x, y, z):
    """Orientation, ENU -> NED. Returns (w, x, y, z)."""
    return w, x, -y, -z


def enu_to_ned_pose(pose):
    """Return a NEW geometry_msgs Pose in NED. Leaves the input alone."""
    from geometry_msgs.msg import Pose
    out = Pose()
    out.position.x, out.position.y, out.position.z = enu_to_ned_xyz(
        pose.position.x, pose.position.y, pose.position.z)
    q = pose.orientation
    out.orientation.w, out.orientation.x, out.orientation.y, \
        out.orientation.z = enu_to_ned_quat(q.w, q.x, q.y, q.z)
    return out


def enu_to_ned_pose_inplace(pose):
    """Convert a geometry_msgs Pose in place and return it.

    For callbacks that own the message and would otherwise copy it just to
    throw the copy away.
    """
    pose.position.y = -pose.position.y
    pose.position.z = -pose.position.z
    pose.orientation.y = -pose.orientation.y
    pose.orientation.z = -pose.orientation.z
    return pose


def _self_test():
    """Check the two rules agree with the rotation they claim to be."""
    import math
    import numpy as np
    R = np.diag([1.0, -1.0, -1.0])
    assert abs(np.linalg.det(R) - 1.0) < 1e-12, "not a proper rotation"

    for p in ((1, 2, 3), (-4, 0.5, -6), (0, 0, 1)):
        assert np.allclose(enu_to_ned_xyz(*p), R @ np.array(p, float))

    def quat_to_R(w, x, y, z):
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
            [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])

    for ax, ang in (((0, 0, 1), 0.7), ((0, 1, 0), -1.1), ((1, 0, 0), 2.0),
                    ((0.3, -0.5, 0.81), 1.3)):
        n = math.sqrt(sum(a * a for a in ax))
        ax = [a / n for a in ax]
        s, c = math.sin(ang / 2), math.cos(ang / 2)
        q = (c, ax[0] * s, ax[1] * s, ax[2] * s)
        got = quat_to_R(*enu_to_ned_quat(*q))
        want = R @ quat_to_R(*q) @ R.T          # change of basis
        assert np.allclose(got, want, atol=1e-12), (q, got, want)

    # a pure yaw must come back negated -- the measured signature
    psi = 0.9
    q = (math.cos(psi/2), 0.0, 0.0, math.sin(psi/2))
    w2, x2, y2, z2 = enu_to_ned_quat(*q)
    assert abs(math.atan2(2*(w2*z2 + x2*y2),
                          1 - 2*(y2*y2 + z2*z2)) + psi) < 1e-12
    print("frames.py self-test OK: position and orientation both agree "
          "with a 180 deg rotation about x, and yaw inverts")


if __name__ == "__main__":
    _self_test()
