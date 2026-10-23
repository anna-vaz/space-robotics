#!/usr/bin/env bash
# Step 5 helper: run the RNS capture WITH the Gazebo window (WSLg) and take screenshots
# through the GUI's screenshot service. Pictures -> results/gz_shots/
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
OUT=/mnt/d/Projects/space-robotics/project5_ros2_gazebo/results
SHOTS=$OUT/gz_shots; rm -rf "$SHOTS"; mkdir -p "$SHOTS"
WORLD=$(ros2 pkg prefix --share zero_g_manipulator)/worlds/zero_g_target.sdf
Q0="q1:=0.0 q2:=2.6179938779914944 q3:=-2.6179938779914944"

ros2 launch zero_g_manipulator sim.launch.py headless:=false n_links:=3 cmd:=velocity grasp:=true \
    controller:=velocity_controller world:=$WORLD $Q0 > "$OUT/step5_gui_launch.log" 2>&1 &
LPID=$!
ros2 run zero_g_manipulator capture_controller --ros-args -p use_sim_time:=true -p mode:=rns \
    -p output:="$OUT/step5_gui_rns.npz" > "$OUT/step5_gui_node.log" 2>&1 &
NPID=$!

# wait for the GUI's screenshot service (camera: top-down, set in the world's <gui> block)
for i in $(seq 1 60); do gz service -l 2>/dev/null | grep -q /gui/screenshot && break; sleep 1; done
# screenshots every 4 s of wall time
for i in $(seq -w 0 12); do
    gz service -s /gui/screenshot --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 3000 \
        --req "data: \"$SHOTS\"" > /dev/null    # data = directory; file name = timestamp
    sleep 4
done
wait $NPID
grep -E "CAPTURE|saved" "$OUT/step5_gui_node.log"
kill -INT $LPID 2>/dev/null; sleep 5
pkill -9 -f 'gz sim|parameter_bridge|robot_state_publisher|ros2 launch zero_g' 2>/dev/null
ls -la "$SHOTS"
