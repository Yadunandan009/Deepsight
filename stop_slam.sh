#!/bin/bash
echo "=== Stopping ORB-SLAM3 cleanly ==="

# Marker for detecting whether the Atlas file below is actually rewritten
# by THIS shutdown, not just present from some earlier run. The old check
# (`ls ~/ros2_ws/slam_atlas/deepsight_map*`) only tested existence, so it
# reported "Atlas file detected!" off a stale file even on a run where
# ORB-SLAM3 segfaulted during its own shutdown destructor before the save
# ever ran (confirmed via the console log: "Interface destructor" followed
# immediately by "Segmentation fault", no save in between, and the file's
# mtime unchanged from the previous run).
ATLAS_STAMP=$(mktemp)
touch -d "@$(( $(date +%s) - 1 ))" "$ATLAS_STAMP" 2>/dev/null || touch "$ATLAS_STAMP"

# Try multiple patterns to find the right PID
SLAM_PID=$(docker exec orb_slam3_running pgrep -f "stereo" 2>/dev/null | grep -v "^1$" | tail -1)

if [ -z "$SLAM_PID" ]; then
    echo "WARNING: pgrep found nothing, trying ros2 wrapper pattern..."
    SLAM_PID=$(docker exec orb_slam3_running pgrep -f "orb_slam3" 2>/dev/null | tail -1)
fi

if [ -z "$SLAM_PID" ]; then
    echo "WARNING: falling back to all processes in container..."
    docker exec orb_slam3_running ps aux | grep -v grep | grep -E "stereo|orb_slam3"
    echo "Sending SIGINT to container PID 1 as last resort..."
    docker kill --signal SIGINT orb_slam3_running
else
    echo "Found ORB-SLAM3 PID: $SLAM_PID"
    docker exec orb_slam3_running kill -2 $SLAM_PID
fi

echo "Waiting 45s for Atlas save (large map needs time)..."
ATLAS_SAVED=0
for i in $(seq 1 45); do
    sleep 1
    # -newer only matches files actually written since ATLAS_STAMP was
    # touched above, i.e. by this shutdown -- not a leftover from before.
    if find ~/ros2_ws/slam_atlas -maxdepth 1 -name 'deepsight_map*' -newer "$ATLAS_STAMP" 2>/dev/null | grep -q .; then
        echo "  [${i}s] Atlas file freshly written this run!"
        ATLAS_SAVED=1
        break
    fi
    [ $((i % 5)) -eq 0 ] && echo "  [${i}s] waiting..."
done
rm -f "$ATLAS_STAMP"

if [ "$ATLAS_SAVED" -eq 0 ]; then
    echo ""
    echo "WARNING: Atlas was NOT saved this run. Any deepsight_map.osa"
    echo "present below is stale, from an earlier run -- do not assume"
    echo "it reflects this mission. Known cause: ORB-SLAM3 has segfaulted"
    echo "during its own shutdown destructor before reaching the save"
    echo "step (check the latest slam_atlas/orb_slam3_console_*.log for"
    echo "'Interface destructor' immediately followed by 'Segmentation"
    echo "fault' with no save message in between)."
fi

echo ""
echo "Atlas directory contents:"
ls -lh ~/ros2_ws/slam_atlas/ 2>/dev/null || echo "(empty or missing)"

docker rm -f orb_slam3_running 2>/dev/null
echo "Done."
