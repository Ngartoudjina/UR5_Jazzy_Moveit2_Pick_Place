#!/bin/bash
# ════════════════════════════════════════════════════════════════════════════
# launch_pick_place_complete.sh
# ════════════════════════════════════════════════════════════════════════════
# Complete pick-place workflow:
# 1. Verify MoveIt2 is running
# 2. Run diagnostics
# 3. Execute pick-place sequence
# ════════════════════════════════════════════════════════════════════════════

set -e  # Exit on error

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "\n${BLUE}════════════════════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}PICK & PLACE COMPLETE WORKFLOW${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════════════════${NC}\n"

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}❌ ROS2 not sourced! Run:${NC}"
    echo "   source /opt/ros/humble/setup.bash"
    exit 1
fi

# Check if workspace is sourced
if [ -z "$COLCON_PREFIX_PATH" ]; then
    echo -e "${YELLOW}⚠️  Workspace not sourced, attempting to source...${NC}"
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
    fi
fi

echo -e "${GREEN}✓ ROS2 environment: ${ROS_DISTRO}${NC}"
echo -e "${GREEN}✓ Workspace: ${COLCON_PREFIX_PATH}${NC}\n"

# ════════════════════════════════════════════════════════════════════════════
# STEP 1: Check if MoveIt2 is running
# ════════════════════════════════════════════════════════════════════════════

echo -e "${BLUE}STEP 1: Checking MoveIt2 status${NC}"
echo "─────────────────────────────────────────────────────────────────────────"

# Check if move_group is running
if pgrep -f "move_group" > /dev/null; then
    echo -e "${GREEN}✓ MoveIt2 move_group is running${NC}"
else
    echo -e "${YELLOW}⚠️  MoveIt2 not running. Starting...${NC}"
    echo "   LaunchingUR5 MoveIt2 in background..."
    ros2 launch ur5_moveit moveit.launch.py &> /tmp/moveit_launch.log &
    MOVEIT_PID=$!
    
    # Wait for MoveIt2 to start
    echo "   Waiting for MoveIt2 to initialize (this takes ~10 seconds)..."
    sleep 10
    
    if pgrep -f "move_group" > /dev/null; then
        echo -e "${GREEN}✓ MoveIt2 started successfully${NC}"
    else
        echo -e "${RED}❌ Failed to start MoveIt2${NC}"
        echo "   Check logs: tail /tmp/moveit_launch.log"
        exit 1
    fi
fi

echo ""

# ════════════════════════════════════════════════════════════════════════════
# STEP 2: Run diagnostics
# ════════════════════════════════════════════════════════════════════════════

echo -e "${BLUE}STEP 2: Running MoveIt2 System Diagnostics${NC}"
echo "─────────────────────────────────────────────────────────────────────────"

if ! ros2 run ur5_pick_place moveit_diagnostics; then
    echo -e "\n${RED}❌ Diagnostics failed - System not ready for pick-place${NC}"
    echo "See diagnostics output above for details"
    exit 1
fi

echo ""

# ════════════════════════════════════════════════════════════════════════════
# STEP 3: Run pick-place sequence
# ════════════════════════════════════════════════════════════════════════════

echo -e "${BLUE}STEP 3: Executing Pick & Place Sequence${NC}"
echo "─────────────────────────────────────────────────────────────────────────\n"

if ros2 run ur5_pick_place pick_place_v3; then
    echo -e "\n${GREEN}════════════════════════════════════════════════════════════════════════════${NC}"
    echo -e "${GREEN}✅ PICK & PLACE COMPLETED SUCCESSFULLY!${NC}"
    echo -e "${GREEN}════════════════════════════════════════════════════════════════════════════${NC}\n"
    exit 0
else
    echo -e "\n${RED}════════════════════════════════════════════════════════════════════════════${NC}"
    echo -e "${RED}❌ PICK & PLACE FAILED${NC}"
    echo -e "${RED}════════════════════════════════════════════════════════════════════════════${NC}\n"
    exit 1
fi
