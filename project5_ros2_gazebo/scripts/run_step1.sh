#!/usr/bin/env bash
# Step 1 (headless): zero-g rest check + open-loop torque pulse, logged to results/.
#   bash /mnt/d/Projects/space-robotics/project5_ros2_gazebo/scripts/run_step1.sh
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
OUT=/mnt/d/Projects/space-robotics/project5_ros2_gazebo/results
TAG=${1:-}          # optional: run_step1.sh _dt05 /path/world.sdf  (experiments)
WORLD=${2:-}
mkdir -p "$OUT"

# the exact URDF that Gazebo gets (checked against Project 1's params in step1_compare.py)
xacro "$(ros2 pkg prefix --share zero_g_manipulator)/urdf/robot.urdf.xacro" > "$OUT/robot.urdf"

ros2 launch zero_g_manipulator sim.launch.py headless:=true ${WORLD:+world:=$WORLD} > "$OUT/step1${TAG}_launch.log" 2>&1 &
LAUNCH_PID=$!

timeout 120 ros2 run zero_g_manipulator step1_torque_pulse --ros-args \
    -p use_sim_time:=true -p output:="$OUT/step1_gazebo${TAG}.npz"

# stop the simulation
kill -INT $LAUNCH_PID 2>/dev/null
sleep 5
pkill -9 -f 'gz sim|parameter_bridge|robot_state_publisher|ros2 launch zero_g' 2>/dev/null
echo "step 1 done"
