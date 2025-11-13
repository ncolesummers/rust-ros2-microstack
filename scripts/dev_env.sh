#!/bin/bash
# Development environment setup for Rust ROS2 Microstack
# Source this script to load ROS2 Jazzy environment
#
# Usage:
#   source scripts/dev_env.sh

set -e

# Detect shell
if [ -n "$ZSH_VERSION" ]; then
    SHELL_TYPE="zsh"
    SETUP_FILE="/opt/ros/jazzy/setup.zsh"
elif [ -n "$BASH_VERSION" ]; then
    SHELL_TYPE="bash"
    SETUP_FILE="/opt/ros/jazzy/setup.bash"
else
    echo "⚠️  Warning: Unknown shell. Defaulting to bash setup."
    SHELL_TYPE="bash"
    SETUP_FILE="/opt/ros/jazzy/setup.bash"
fi

# Check if ROS2 Jazzy is installed
if [ ! -f "$SETUP_FILE" ]; then
    echo "❌ Error: ROS2 Jazzy not found at $SETUP_FILE"
    echo "   Please install ROS2 Jazzy: https://docs.ros.org/en/jazzy/Installation.html"
    return 1
fi

# Source ROS2 environment
echo "🔧 Loading ROS2 Jazzy environment ($SHELL_TYPE)..."
source "$SETUP_FILE"

# Verify ROS2 is available
if ! command -v ros2 &> /dev/null; then
    echo "❌ Error: ros2 command not found after sourcing setup"
    return 1
fi

# Set default ROS2 configuration
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
# Note: RMW_IMPLEMENTATION not set - uses ROS2 default (typically rmw_fastrtps_cpp)
# To use a specific RMW: export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp (requires installation)

# Set Rust logging (optional, can be overridden)
export RUST_LOG=${RUST_LOG:-info}

echo "✅ ROS2 Jazzy environment loaded successfully!"
echo "   ROS_DISTRO: $ROS_DISTRO"
echo "   ROS_VERSION: $ROS_VERSION"
echo "   RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-default}"
echo "   RUST_LOG: $RUST_LOG"
echo ""
echo "🚀 Ready to build and run nodes:"
echo "   cargo build --workspace"
echo "   cargo run -p apps/teleop_mux -- --help"
