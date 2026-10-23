#!/usr/bin/env bash
# Step 2 (headless): quintic joint move with joint_trajectory_controller (P1 step 3 case).
#   bash /mnt/d/Projects/space-robotics/project5_ros2_gazebo/scripts/run_step2.sh
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
OUT=/mnt/d/Projects/space-robotics/project5_ros2_gazebo/results
TAG=${1:-}          # optional: run_step2.sh _dt05 /path/world.sdf  (experiments)
WORLD=${2:-}
mkdir -p "$OUT"

ros2 launch zero_g_manipulator sim.launch.py headless:=true controller:=joint_trajectory_controller \
    ${WORLD:+world:=$WORLD} > "$OUT/step2${TAG}_launch.log" 2>&1 &
LAUNCH_PID=$!

timeout 150 ros2 run zero_g_manipulator step2_quintic_move --ros-args \
    -p use_sim_time:=true -p output:="$OUT/step2_gazebo${TAG}.npz"

kill -INT $LAUNCH_PID 2>/dev/null
sleep 5
pkill -9 -f 'gz sim|parameter_bridge|robot_state_publisher|ros2 launch zero_g' 2>/dev/null
echo "step 2 done"
