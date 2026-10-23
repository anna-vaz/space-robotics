"""Step 2 test: quintic joint move with the standard ros2_control trajectory controller.

Same move as Project 1 step 3: joints (0, 0) -> (90, -90) deg in 10 s, then hold
until 15 s. The base is not controlled and reacts freely.

The goal is sent as TWO waypoints on /joint_trajectory_controller/joint_trajectory:
    point 0: q = (0, 0) at rest, time_from_start = 0      (stamped 1 s in the future)
    point 1: q = goal  at rest, time_from_start = 10 s
so the controller's quintic spline between them is exactly P1's quintic. We log
the controller's own state (reference, measured, torque output) plus the base
odometry, and save everything to .npz for step2_compare.py.
"""
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTrajectoryControllerState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from zero_g_manipulator.step1_torque_pulse import rpy, stamp

JOINTS = ["joint1", "joint2"]
CTRL = "/joint_trajectory_controller"


def point(q, t):
    return JointTrajectoryPoint(positions=list(q), velocities=[0.0, 0.0], accelerations=[0.0, 0.0],
                                time_from_start=Duration(sec=int(t), nanosec=int(round((t % 1) * 1e9))))


class QuinticMove(Node):
    def __init__(self):
        super().__init__("step2_quintic_move")
        self.output = self.declare_parameter("output", "step2_gazebo.npz").value
        self.q_goal = np.radians(self.declare_parameter("q_goal_deg", [90.0, -90.0]).value)
        self.t_move = self.declare_parameter("t_move", 10.0).value
        self.t_end = self.declare_parameter("t_end", 15.0).value

        self.pub = self.create_publisher(JointTrajectory, f"{CTRL}/joint_trajectory", 10)
        self.create_subscription(JointTrajectoryControllerState, f"{CTRL}/controller_state", self.on_state, 100)
        self.create_subscription(Odometry, "/model/space_robot/odometry", self.on_odom, 100)
        self.t0 = None
        self.done = False
        self.ctrl_log, self.odom_log = [], []
        self.get_logger().info("waiting for the trajectory controller ...")

    def on_state(self, msg):
        t = stamp(msg)
        if self.t0 is None:
            # controller is active -> start the move 1 s from now (sim time)
            self.t0 = t + 1.0
            traj = JointTrajectory(joint_names=JOINTS)
            traj.header.stamp = Time(seconds=self.t0).to_msg()
            traj.points = [point([0.0, 0.0], 0.0), point(self.q_goal, self.t_move)]
            self.pub.publish(traj)
            self.get_logger().info(f"trajectory sent at t = {t:.3f} s, starts at {self.t0:.3f} s")
            return
        tau = t - self.t0
        if self.done or tau < -0.5:
            return
        idx = [msg.joint_names.index(j) for j in JOINTS]
        r, f, o = msg.reference, msg.feedback, msg.output
        self.ctrl_log.append([tau, *[r.positions[i] for i in idx], *[r.velocities[i] for i in idx],
                              *[f.positions[i] for i in idx], *[f.velocities[i] for i in idx],
                              *[o.effort[i] for i in idx]])
        if tau > self.t_end:
            self.done = True

    def on_odom(self, msg):
        if self.t0 is None or self.done:
            return
        tau = stamp(msg) - self.t0
        if tau < -0.5:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        w = msg.twist.twist.angular
        self.odom_log.append([tau, p.x, p.y, p.z, *rpy(q), w.z])

    def save(self):
        c, o = np.array(self.ctrl_log), np.array(self.odom_log)
        np.savez(self.output, t=c[:, 0], q_ref=c[:, 1:3], qd_ref=c[:, 3:5], q=c[:, 5:7],
                 qd=c[:, 7:9], tau=c[:, 9:11], odom_t=o[:, 0], base_xyz=o[:, 1:4],
                 base_rpy=o[:, 4:7], base_wz=o[:, 7], q_goal=self.q_goal, t_move=self.t_move)
        self.get_logger().info(f"saved {len(c)} controller samples, {len(o)} odometry samples -> {self.output}")


def main():
    rclpy.init()
    node = QuinticMove()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.save()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
