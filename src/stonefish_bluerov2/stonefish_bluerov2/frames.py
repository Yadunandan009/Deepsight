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
           "enu_to_ned_pose_inplace", "FRAME_NOTE",
           "vi_to_ned_xyz", "vi_to_ned_pose_inplace", "VI_FRAME_NOTE"]

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


VI_FRAME_NOTE = ("SLAM pose treated as stereo-inertial ORB-SLAM3: published "
                 "z down, taken as NED as-is (measured 2026-11-15)")


# Stereo-inertial ORB-SLAM3 publishes in a DIFFERENT frame from stereo-only,
# and enu_to_ned is wrong for it.
#
# MEASURED, not derived. On a50_vishake_seed0_20261115_021022 the published
# z follows Stonefish depth with correlation 1.000 over the whole mission
# (descent: published dz +4.5 m per truth +5.1 m), x and y carry the
# horizontal motion, and a proper rotation about z fits the trajectory far
# better than a mirror image (Sim(3) rms 3.3 m vs 10.0 m over 900 s). So the
# published pose is already NED-like: z down, right-handed, with an
# arbitrary heading, which the bridge fits anyway.
#
# The first version of this function was derived from the wrapper source
# instead (ros = (z_orb, -x_orb, -y_orb) on a gravity-aligned map with z up,
# which puts UP in published x) and mapped (x, y, z) -> (-y, z, -x). That
# derivation misread which frame TrackStereo's pose is in once the IMU is
# initialised; the shakedown showed the result tilted 85 deg from level.
# The self-test below now checks the measured fact: published +z is NED down.
def vi_to_ned_xyz(x, y, z):
    """Position, published stereo-inertial pose -> NED. Returns (x, y, z).

    Identity: the published pose is already z-down and right-handed (see
    above). Kept as a function so the bridge, the replay tools and the
    orientation rule below all go through one place.
    """
    return x, y, z


def _matrix_of(f):
    """The 3x3 matrix of a linear map given as f(x, y, z) -> (x, y, z)."""
    cols = [f(*e) for e in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))]
    return [[cols[j][i] for j in range(3)] for i in range(3)]


def _quat_of(R):
    """(w, x, y, z) of a proper rotation matrix (Shepperd's method)."""
    import math
    t = R[0][0] + R[1][1] + R[2][2]
    if t > 0:
        s = 2.0 * math.sqrt(1.0 + t)
        return (0.25 * s, (R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s,
                (R[1][0] - R[0][1]) / s)
    i = max(range(3), key=lambda k: R[k][k])
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * math.sqrt(1.0 + R[i][i] - R[j][j] - R[k][k])
    q = [0.0, 0.0, 0.0, 0.0]
    q[0] = (R[k][j] - R[j][k]) / s
    q[1 + i] = 0.25 * s
    q[1 + j] = (R[j][i] + R[i][j]) / s
    q[1 + k] = (R[k][i] + R[i][k]) / s
    return tuple(q)


def _qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw*bw - ax*bx - ay*by - az*bz, aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx, aw*bz + ax*by - ay*bx + az*bw)


_Q_VI = None
_Q_BODY = (1.0, 0.0, 0.0, 0.0)   # body axes as published: no flip measured


def vi_to_ned_pose_inplace(pose):
    """Convert a published stereo-inertial pose to NED in place, return it.

    Orientation is derived from vi_to_ned_xyz, not written separately, so the
    two cannot disagree: the world side turns by the same rotation as the
    position. The body side is left as published; nothing downstream reads
    the SLAM orientation except as a heading offset, which the bridge fits.
    """
    global _Q_VI
    if _Q_VI is None:
        _Q_VI = _quat_of(_matrix_of(vi_to_ned_xyz))
    p = pose.position
    p.x, p.y, p.z = vi_to_ned_xyz(p.x, p.y, p.z)
    o = pose.orientation
    o.w, o.x, o.y, o.z = _qmul(_qmul(_Q_VI, (o.w, o.x, o.y, o.z)), _Q_BODY)
    return pose


def _self_test():
    """Check the two rules agree with the rotation they claim to be."""
    import math
    try:
        _vi_self_test()
    except NotImplementedError:
        print("frames.py: vi_to_ned_xyz not written yet, skipping its test")
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


def _vi_self_test():
    import numpy as np
    from types import SimpleNamespace as NS
    R = np.array(_matrix_of(vi_to_ned_xyz), float)
    assert np.allclose(R @ R.T, np.eye(3)), "vi_to_ned_xyz is not a rotation"
    assert abs(np.linalg.det(R) - 1.0) < 1e-12, \
        "vi_to_ned_xyz is a reflection (det -1), not a rotation"
    # published +z follows depth (measured), so it must stay NED down
    assert np.allclose(R @ [0, 0, 1], [0, 0, 1]), \
        f"published +z (down) maps to {R @ [0, 0, 1]}, not NED down (0, 0, 1)"
    # orientation follows the same rotation on the world side
    pose = NS(position=NS(x=1.0, y=2.0, z=3.0),
              orientation=NS(w=1.0, x=0.0, y=0.0, z=0.0))
    vi_to_ned_pose_inplace(pose)
    w, x, y, z = (pose.orientation.w, pose.orientation.x,
                  pose.orientation.y, pose.orientation.z)
    Rq = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                   [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                   [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    assert np.allclose(Rq, R)
    print("frames.py vi self-test OK: proper rotation, published z -> NED down")


if __name__ == "__main__":
    _self_test()
