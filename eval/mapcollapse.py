#!/usr/bin/env python3
"""Count map-point collapses straight from the bag.

The bridge only logs a crash when it happens to be holding an alignment,
so its console count under-reports. /bluerov2/map_points carries the
Atlas size at every tick; a collapse is a drop to a small fraction of
the running maximum, which is what re-initialisation looks like.
"""
import glob, os, sys
from mcap_ros2.reader import read_ros2_messages

for bag in sys.argv[1:]:
    pts = []
    for f in sorted(glob.glob(os.path.join(bag, "*.mcap"))):
        for m in read_ros2_messages(f, topics=["/bluerov2/map_points"]):
            t = (m.log_time.timestamp() if hasattr(m.log_time, "timestamp")
                 else m.log_time / 1e9)
            # PointCloud2: the point count is width*height, not len(data)
            msg = m.ros_msg
            n = int(msg.width) * int(msg.height)
            pts.append((t, n))
    if not pts:
        print(f"{os.path.basename(bag)}: no map_points")
        continue
    t0 = pts[0][0]
    peak = 0
    events = []
    for (ta, a), (tb, b) in zip(pts, pts[1:]):
        peak = max(peak, a)
        if a > 1000 and b < 0.5 * a:
            events.append((tb - t0, a, b))
    print(f"{os.path.basename(bag)}")
    print(f"  {len(pts)} samples, peak {peak}, final {pts[-1][1]}, "
          f"duration {pts[-1][0]-t0:.0f}s")
    print(f"  {len(events)} collapse(s) (>50% drop from above 1000):")
    for s, a, b in events:
        print(f"    t={s:6.0f}s   {a:7d} -> {b:6d}")
