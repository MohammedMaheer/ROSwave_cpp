#!/bin/bash
# ROS2 Dashboard Launcher with proper environment setup

set -e

# Ensure ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Sourcing ROS2 Humble..."
    source /opt/ros/humble/setup.bash
fi

# Set display for GUI
export QT_QPA_PLATFORM=xcb

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"
EXECUTABLE="$BUILD_DIR/ros2_dashboard"

echo "========================================"
echo "ROS2 Live Status Dashboard"
echo "========================================"
echo ""
echo "Environment:"
echo "  ROS_DISTRO: $ROS_DISTRO"
echo "  ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}"
echo "  Display: $DISPLAY"
echo "  QT_QPA_PLATFORM: $QT_QPA_PLATFORM"
echo ""
echo "Executable: $EXECUTABLE"
echo ""

# Check if executable exists
if [ ! -f "$EXECUTABLE" ]; then
    echo "❌ Executable not found: $EXECUTABLE"
    echo ""
    echo "Building the dashboard..."
    cd "$BUILD_DIR"
    make -j4
    echo ""
fi

# Run the dashboard
echo "✓ Starting dashboard..."
echo ""

"$EXECUTABLE"
