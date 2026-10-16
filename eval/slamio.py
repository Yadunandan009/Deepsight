"""One place where the eval tools convert the SLAM pose into NED.

`/bluerov2/robot_pose_slam` is published by the ORB-SLAM3 wrapper in
REP-103 (z up). This workspace is NED (z down). The live nodes convert on
ingest via stonefish_bluerov2.frames; the eval tools read the bag topic
directly, so without this they measure the UNCONVERTED track -- which is
how "no fixed map-to-world transform exists" survived as long as it did.
See docs/SLAM_FRAME_ENU_NED.md.

    (x, y, z) -> (x, -y, -z)      det = +1, 180 deg about x

Tools whose whole job is to observe the raw handedness --
reflect_check.py, yaw_handedness.py, axis_discriminator.py,
enu_ned_check.py -- deliberately do NOT use this. They read the topic as
published, because the flip is the thing they measure.
"""

import os
import sys

__all__ = ["slam_to_ned_xy", "slam_to_ned_xyz", "SLAM_TOPIC", "NOTE"]

SLAM_TOPIC = "/bluerov2/robot_pose_slam"
NOTE = ("SLAM pose converted ENU->NED on read (y,z negated); "
        "see docs/SLAM_FRAME_ENU_NED.md")

# Import the single definition rather than re-deriving it here, so the
# eval path and the live path can never disagree.
_PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    "..", "src", "stonefish_bluerov2")
try:
    from stonefish_bluerov2.frames import enu_to_ned_xyz
except ImportError:
    sys.path.insert(0, _PKG)
    from stonefish_bluerov2.frames import enu_to_ned_xyz


def slam_to_ned_xyz(x, y, z):
    """Position from /bluerov2/robot_pose_slam, in NED."""
    return enu_to_ned_xyz(x, y, z)


def slam_to_ned_xy(x, y):
    """Horizontal only, for the 2-D fits."""
    x, y, _ = enu_to_ned_xyz(x, y, 0.0)
    return x, y


if __name__ == "__main__":
    assert slam_to_ned_xyz(1.0, 2.0, 3.0) == (1.0, -2.0, -3.0)
    assert slam_to_ned_xy(1.0, 2.0) == (1.0, -2.0)
    print("slamio self-test OK -", NOTE)
