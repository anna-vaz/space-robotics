#!/usr/bin/env bash
# One-time setup (run inside Ubuntu as anna):
#   bash /mnt/d/Projects/space-robotics/project5_ros2_gazebo/scripts/setup_workspace.sh
# The package source stays in the git repo on D:, ~/ros2_ws/src only links to it;
# the build itself happens on the fast Linux disk (~/ros2_ws, physically on D: too).
set -e
source /opt/ros/jazzy/setup.bash
REPO=/mnt/d/Projects/space-robotics/project5_ros2_gazebo
mkdir -p ~/ros2_ws/src
ln -sfn "$REPO/zero_g_manipulator" ~/ros2_ws/src/zero_g_manipulator
cd ~/ros2_ws
colcon build --symlink-install
grep -q 'ros2_ws/install/setup.bash' ~/.bashrc || echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
echo "workspace ready: ~/ros2_ws"
