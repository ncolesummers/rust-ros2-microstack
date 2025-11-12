#!/bin/bash
# One-time setup verification script
# Checks that all required dependencies are installed
#
# Usage:
#   bash scripts/setup.sh

set -e

echo "🔍 Verifying Rust ROS2 Microstack dependencies..."
echo ""

# Track success/failure
ALL_OK=true

# Check ROS2 Jazzy
echo -n "Checking ROS2 Jazzy... "
if [ -f /opt/ros/jazzy/setup.bash ]; then
    echo "✅ Found"
else
    echo "❌ NOT FOUND"
    echo "   Install: https://docs.ros.org/en/jazzy/Installation.html"
    ALL_OK=false
fi

# Check Rust
echo -n "Checking Rust toolchain... "
if command -v rustc &> /dev/null; then
    RUST_VERSION=$(rustc --version | awk '{print $2}')
    echo "✅ Found ($RUST_VERSION)"

    # Check edition 2024 support (requires Rust 1.85+)
    MAJOR=$(echo $RUST_VERSION | cut -d. -f1)
    MINOR=$(echo $RUST_VERSION | cut -d. -f2)
    if [ "$MAJOR" -eq 1 ] && [ "$MINOR" -lt 85 ]; then
        echo "   ⚠️  Warning: Rust $RUST_VERSION may not support edition 2024"
        echo "   Update: rustup update stable"
    fi
else
    echo "❌ NOT FOUND"
    echo "   Install: https://rustup.rs"
    ALL_OK=false
fi

# Check cargo
echo -n "Checking cargo... "
if command -v cargo &> /dev/null; then
    CARGO_VERSION=$(cargo --version | awk '{print $2}')
    echo "✅ Found ($CARGO_VERSION)"
else
    echo "❌ NOT FOUND"
    ALL_OK=false
fi

# Check Gazebo
echo -n "Checking Gazebo... "
if command -v gz &> /dev/null; then
    GZ_VERSION=$(gz sim --version 2>&1 | head -1 | awk '{print $NF}')
    echo "✅ Found ($GZ_VERSION)"
else
    echo "❌ NOT FOUND"
    echo "   Install: sudo apt install ros-jazzy-ros-gz"
    ALL_OK=false
fi

# Check build tools
echo -n "Checking clang... "
if command -v clang &> /dev/null; then
    CLANG_VERSION=$(clang --version | head -1 | awk '{print $3}')
    echo "✅ Found ($CLANG_VERSION)"
else
    echo "❌ NOT FOUND"
    echo "   Install: sudo apt install clang"
    ALL_OK=false
fi

echo -n "Checking cmake... "
if command -v cmake &> /dev/null; then
    CMAKE_VERSION=$(cmake --version | head -1 | awk '{print $3}')
    echo "✅ Found ($CMAKE_VERSION)"
else
    echo "❌ NOT FOUND"
    echo "   Install: sudo apt install cmake"
    ALL_OK=false
fi

echo -n "Checking pkg-config... "
if command -v pkg-config &> /dev/null; then
    PKGCONFIG_VERSION=$(pkg-config --version)
    echo "✅ Found ($PKGCONFIG_VERSION)"
else
    echo "❌ NOT FOUND"
    echo "   Install: sudo apt install pkg-config"
    ALL_OK=false
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if [ "$ALL_OK" = true ]; then
    echo "✅ All dependencies satisfied!"
    echo ""
    echo "Next steps:"
    echo "  1. source scripts/dev_env.sh"
    echo "  2. cargo build --workspace"
    echo "  3. cargo test"
    echo "  4. cargo run -p apps/teleop_mux -- --help"
    exit 0
else
    echo "❌ Some dependencies are missing."
    echo "   Please install missing packages and run this script again."
    exit 1
fi
