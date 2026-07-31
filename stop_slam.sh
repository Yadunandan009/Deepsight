#!/bin/bash
echo "=== Stopping ORB-SLAM3 cleanly ==="

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
for i in $(seq 1 45); do
    sleep 1
    # Check if file appeared
    if ls ~/ros2_ws/slam_atlas/deepsight_map* 2>/dev/null | grep -q .; then
        echo "  [${i}s] Atlas file detected!"
        break
    fi
    [ $((i % 5)) -eq 0 ] && echo "  [${i}s] waiting..."
done

echo ""
echo "Atlas directory contents:"
ls -lh ~/ros2_ws/slam_atlas/ 2>/dev/null || echo "(empty or missing)"

docker rm -f orb_slam3_running 2>/dev/null
echo "Done."
