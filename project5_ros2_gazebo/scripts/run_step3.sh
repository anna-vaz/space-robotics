#!/usr/bin/env bash
# Step 3 (headless): Project 1b 3-link arm, joint-velocity servos, RNS controller.
# Runs: commanded vs RNS at 1 kHz, then RNS at lower controller rates.
#   bash /mnt/d/Projects/space-robotics/project5_ros2_gazebo/scripts/run_step3.sh
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
OUT=/mnt/d/Projects/space-robotics/project5_ros2_gazebo/results
mkdir -p "$OUT"
# P1b step 2 start configuration: q = (30, 60, -45) deg
Q0="q1:=0.5235987755982988 q2:=1.0471975511965976 q3:=-0.7853981633974483"

run () {   # run <mode> <rate_hz> [t_ramp]
    local ramp=${3:-0.0}
    local tag="step3_${1}_${2}hz"; [ "$ramp" != "0.0" ] && tag="${tag}_ramp"
    ros2 launch zero_g_manipulator sim.launch.py headless:=true n_links:=3 cmd:=velocity \
        controller:=velocity_controller $Q0 > "$OUT/${tag}_launch.log" 2>&1 &
    local pid=$!
    timeout 120 ros2 run zero_g_manipulator rns_controller --ros-args -p use_sim_time:=true \
        -p mode:=$1 -p rate_hz:=$2.0 -p t_ramp:=${ramp} -p output:="$OUT/${tag}.npz"
    kill -INT $pid 2>/dev/null; sleep 5
    pkill -9 -f 'gz sim|parameter_bridge|robot_state_publisher|ros2 launch zero_g' 2>/dev/null; sleep 1
}

# cases mode:rate[:ramp]; ramp in units of 0.1 s -> 2 = 0.2 s (keeps the bash simple)
CASES=${*:-"commanded:1000 rns:1000 rns:100 rns:20 rns:1000:2 rns:100:2 rns:20:2"}
for c in $CASES; do
    IFS=: read -r mode rate r <<< "$c"
    if [ -n "$r" ]; then run $mode $rate "0.$r"; else run $mode $rate; fi
done
echo "step 3 done"
