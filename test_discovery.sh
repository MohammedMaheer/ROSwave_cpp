#!/bin/bash
# Test script to verify dashboard discovers topics

echo "=== ROS2 Dashboard Discovery Test ==="
echo ""

# Check ROS2 environment
echo "1. Checking ROS2 environment..."
if [ -z "$ROS_DISTRO" ]; then
    echo "   ❌ ROS_DISTRO not set"
    source /opt/ros/humble/setup.bash
    echo "   ✅ Sourced ROS2 Humble"
else
    echo "   ✅ ROS_DISTRO=$ROS_DISTRO"
fi

echo ""
echo "2. Checking ROS2 domain..."
echo "   Domain ID: ${ROS_DOMAIN_ID:-default (0)}"

echo ""
echo "3. Listing available topics..."
ros2 topic list -t 2>&1 | head -10

echo ""
echo "4. Listing available nodes..."
ros2 node list 2>&1

echo ""
echo "5. Listing available services..."
ros2 service list 2>&1 | head -5

echo ""
echo "6. Checking dashboard process..."
if pgrep -f "ros2_dashboard" > /dev/null; then
    PID=$(pgrep -f "ros2_dashboard")
    echo "   ✅ Dashboard running (PID: $PID)"
else
    echo "   ❌ Dashboard not running"
fi

echo ""
echo "=== Test Complete ==="
