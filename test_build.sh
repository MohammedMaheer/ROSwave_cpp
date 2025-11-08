#!/bin/bash
# ROS2 Live Status Dashboard - Test Suite
# This script tests all components of the dashboard

set -e

DASHBOARD_DIR="/home/maahir/Desktop/cpp_ros2_live_status_dashboard"
BUILD_DIR="$DASHBOARD_DIR/build"

echo "=========================================="
echo "ROS2 Live Status Dashboard - Build Verification"
echo "=========================================="
echo ""

# Check if build directory exists
if [ ! -d "$BUILD_DIR" ]; then
    echo "❌ Build directory not found: $BUILD_DIR"
    exit 1
fi

# Check for executables
echo "✓ Checking executables..."
EXECUTABLES=("ros2_dashboard" "ros2_upload_server" "ros2_verify_performance")

for exe in "${EXECUTABLES[@]}"; do
    if [ -f "$BUILD_DIR/$exe" ]; then
        SIZE=$(ls -lh "$BUILD_DIR/$exe" | awk '{print $5}')
        echo "  ✓ $exe ($SIZE)"
    else
        echo "  ❌ $exe not found"
        exit 1
    fi
done

echo ""
echo "✓ Checking file types..."
cd "$BUILD_DIR"
for exe in "${EXECUTABLES[@]}"; do
    TYPE=$(file "$exe" | grep -o "ELF.*")
    echo "  ✓ $exe: $TYPE"
done

echo ""
echo "✓ Checking dependencies..."
for exe in "${EXECUTABLES[@]}"; do
    echo "  ✓ $exe dependencies:"
    ldd "$exe" 2>/dev/null | grep "=>" | head -5 | sed 's/^/    /'
done

echo ""
echo "=========================================="
echo "Build Verification Complete!"
echo "=========================================="
echo ""
echo "To run the dashboard:"
echo "  cd $BUILD_DIR"
echo "  ./ros2_dashboard"
echo ""
echo "To run the upload server:"
echo "  ./ros2_upload_server"
echo ""
echo "To run performance verification:"
echo "  ./ros2_verify_performance"
echo ""
