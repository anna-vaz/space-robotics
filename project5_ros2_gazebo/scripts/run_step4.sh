#!/usr/bin/env bash
# Step 4 (headless): reach + capture Project 3's tumbling target with the 3-link arm.
# Runs the reactionless controller ('rns') and the plain generalized-Jacobian one ('minnorm').
#   bash /mnt/d/Projects/space-robotics/project5_ros2_gazebo/scripts/run_step4.sh [modes...]
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
OUT=/mnt/d/Projects/space-robotics/project5_ros2_gazebo/results
mkdir -p "$OUT"
WORLD=$(ros2 pkg prefix --share zero_g_manipulator)/worlds/zero_g_target.sdf
# start pose chosen by step4_plan.py: q = (0, 150, -150) deg
Q0="q1:=0.0 q2:=2.6179938779914944 q3:=-2.6179938779914944"

for mode in ${*:-rns minnorm}; do
    ros2 launch zero_g_manipulator sim.launch.py headless:=true n_links:=3 cmd:=velocity grasp:=true \
        controller:=velocity_controller world:=$WORLD $Q0 > "$OUT/step4_${mode}_launch.log" 2>&1 &
    pid=$!
    timeout 150 ros2 run zero_g_manipulator capture_controller --ros-args -p use_sim_time:=true \
        -p mode:=$mode -p output:="$OUT/step4_${mode}.npz"
    kill -INT $pid 2>/dev/null; sleep 5
    pkill -9 -f 'gz sim|parameter_bridge|robot_state_publisher|ros2 launch zero_g' 2>/dev/null; sleep 1
done
echo "step 4 done"
