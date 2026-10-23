"""Start Gazebo (zero-g world), spawn the space robot and its ros2_control controllers.

    ros2 launch zero_g_manipulator sim.launch.py                 # with the Gazebo window
    ros2 launch zero_g_manipulator sim.launch.py headless:=true  # server only (remote runs)
    ros2 launch zero_g_manipulator sim.launch.py n_links:=3 cmd:=velocity         controller:=velocity_controller q1:=0.5236 q2:=1.0472 q3:=-0.7854   # Project 1b arm
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("zero_g_manipulator")
    xacro_file = os.path.join(pkg, "urdf", "robot.urdf.xacro")
    gz_launch = os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")

    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    controller = LaunchConfiguration("controller")

    robot_description = ParameterValue(Command([
        "xacro ", xacro_file,
        " n_links:=", LaunchConfiguration("n_links"), " cmd:=", LaunchConfiguration("cmd"),
        " q1:=", LaunchConfiguration("q1"), " q2:=", LaunchConfiguration("q2"),
        " q3:=", LaunchConfiguration("q3"), " grasp:=", LaunchConfiguration("grasp")]), value_type=str)

    # 1) Gazebo: '-r' = start running, '-s' = server only (no GUI)
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={"gz_args": ["-r -v 2 ", world], "on_exit_shutdown": "true"}.items(),
        condition=UnlessCondition(headless))
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={"gz_args": ["-s -r -v 2 ", world], "on_exit_shutdown": "true"}.items(),
        condition=IfCondition(headless))

    # 2) robot_state_publisher: URDF -> /robot_description + TF of the links
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               parameters=[{"robot_description": robot_description, "use_sim_time": True}],
               output="screen")

    # 3) spawn the URDF into Gazebo (at the origin, at rest)
    spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-topic", "robot_description", "-name", "space_robot",
                            "-x", "0", "-y", "0", "-z", "0"])

    # 4) bridge Gazebo -> ROS: simulation clock and the base odometry
    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
                  arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                             "/model/space_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry"])

    # 4b) grasp scenario (step 4): target odometry, capture status, attach/detach, spin-up wrench
    grasp_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        condition=IfCondition(LaunchConfiguration("grasp")),
        arguments=["/model/target/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                   "/space_robot/attach_state@std_msgs/msg/String[gz.msgs.StringMsg",
                   "/space_robot/attach@std_msgs/msg/Empty]gz.msgs.Empty",
                   "/space_robot/detach@std_msgs/msg/Empty]gz.msgs.Empty",
                   "/world/zero_g/wrench@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench"])

    # 5) start the controllers (the spawner waits for the controller manager)
    jsb = Node(package="controller_manager", executable="spawner",
               arguments=["joint_state_broadcaster", "--controller-manager-timeout", "120"])
    ctrl = Node(package="controller_manager", executable="spawner",
                arguments=[controller, "--controller-manager-timeout", "120"])

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("world", default_value=os.path.join(pkg, "worlds", "zero_g.sdf")),
        DeclareLaunchArgument("controller", default_value="effort_controller"),
        DeclareLaunchArgument("n_links", default_value="2"),
        DeclareLaunchArgument("cmd", default_value="effort"),
        DeclareLaunchArgument("q1", default_value="0.0"),
        DeclareLaunchArgument("q2", default_value="0.0"),
        DeclareLaunchArgument("q3", default_value="0.0"),
        DeclareLaunchArgument("grasp", default_value="false"),
        gazebo_gui, gazebo_server, rsp, spawn, bridge, grasp_bridge, jsb, ctrl,
    ])
