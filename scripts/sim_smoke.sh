#!/bin/bash
# Gazebo smoke test - verifies simulator and ROS2 bridge work
# Runs headless, waits for /clock topic, then exits
#
# Usage:
#   bash scripts/sim_smoke.sh

set -e

echo "🔬 Running Gazebo smoke test..."
echo ""

# Ensure ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ Error: ROS2 environment not loaded"
    echo "   Run: source scripts/dev_env.sh"
    exit 1
fi

# Cleanup function
cleanup() {
    echo ""
    echo "🧹 Cleaning up..."
    if [ -n "$GZ_PID" ] && kill -0 $GZ_PID 2>/dev/null; then
        echo "   Stopping Gazebo (PID $GZ_PID)..."
        kill $GZ_PID 2>/dev/null || true
        wait $GZ_PID 2>/dev/null || true
    fi
    if [ -n "$BRIDGE_PID" ] && kill -0 $BRIDGE_PID 2>/dev/null; then
        echo "   Stopping ROS-Gazebo bridge (PID $BRIDGE_PID)..."
        kill $BRIDGE_PID 2>/dev/null || true
        wait $BRIDGE_PID 2>/dev/null || true
    fi
}

trap cleanup EXIT INT TERM

# Start Gazebo in headless mode (server only, no GUI)
echo "🚀 Starting Gazebo headless..."
gz sim -s -r shapes.sdf > /tmp/gz_sim.log 2>&1 &
GZ_PID=$!

echo "   Gazebo PID: $GZ_PID"
echo "   Waiting for Gazebo to initialize..."
sleep 3

# Check if Gazebo is still running
if ! kill -0 $GZ_PID 2>/dev/null; then
    echo "❌ Gazebo failed to start!"
    echo "   Check /tmp/gz_sim.log for details"
    exit 1
fi

# Start ROS-Gazebo bridge
echo "🌉 Starting ROS-Gazebo bridge..."
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock > /tmp/bridge.log 2>&1 &
BRIDGE_PID=$!

echo "   Bridge PID: $BRIDGE_PID"
echo "   Waiting for bridge to initialize..."
sleep 2

# Check if bridge is still running
if ! kill -0 $BRIDGE_PID 2>/dev/null; then
    echo "❌ Bridge failed to start!"
    echo "   Check /tmp/bridge.log for details"
    exit 1
fi

# Wait for /clock topic to publish
echo "⏰ Waiting for /clock topic (timeout: 10s)..."
if timeout 10s bash -c 'until ros2 topic list | grep -q "/clock"; do sleep 0.5; done'; then
    echo "   /clock topic detected!"
else
    echo "❌ Timeout waiting for /clock topic"
    echo "   Available topics:"
    ros2 topic list || true
    exit 1
fi

# Echo one clock message to verify
echo "📡 Verifying /clock publishes messages..."
if CLOCK_MSG=$(timeout 5s ros2 topic echo --once /clock 2>&1); then
    echo "✅ Received clock message:"
    echo "$CLOCK_MSG" | head -5
else
    echo "❌ Failed to receive clock message"
    exit 1
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✅ Gazebo smoke test PASSED!"
echo "   Gazebo and ROS2 bridge are working correctly."
echo ""
echo "To run Gazebo with GUI:"
echo "  ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=\"shapes.sdf\""
echo ""

# Cleanup called automatically via trap
exit 0
