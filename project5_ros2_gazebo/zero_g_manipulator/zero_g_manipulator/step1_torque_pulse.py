"""Step 1 test: open-loop torque pulse on the free-floating robot in Gazebo.

Timeline (test time tau, starts 1 s after the controllers are up):
    0 .. t_rest            : no torque  -> robot must stay exactly at rest (zero-g check)
    t_rest .. +t_pulse     : tau_j = A_j sin(2 pi s / t_pulse)  (one full sine period)
    then .. t_coast        : no torque  -> free drift, momentum must be conserved

Everything is logged (joint states, base odometry, the commanded torques) and
saved to an .npz file, which step1_compare.py replays in the Project 1 Python model.
"""
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINTS = ("joint1", "joint2")


def stamp(msg):
    return msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec


def rpy(q):
    """Quaternion -> roll, pitch, yaw."""
    roll = np.arctan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x**2 + q.y**2))
    pitch = np.arcsin(np.clip(2 * (q.w * q.y - q.z * q.x), -1, 1))
    yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
    return roll, pitch, yaw


class TorquePulseTest(Node):
    def __init__(self):
        super().__init__("step1_torque_pulse")
        self.output = self.declare_parameter("output", "step1_gazebo.npz").value
        self.t_rest = self.declare_parameter("t_rest", 2.0).value
        self.t_pulse = self.declare_parameter("t_pulse", 4.0).value
        self.t_coast = self.declare_parameter("t_coast", 6.0).value
        self.amp = np.array(self.declare_parameter("amplitude", [3.0, -1.5]).value)
        self.t_end = self.t_rest + self.t_pulse + self.t_coast

        self.pub = self.create_publisher(Float64MultiArray, "/effort_controller/commands", 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 100)
        self.create_subscription(Odometry, "/model/space_robot/odometry", self.on_odom, 100)

        self.t0 = None          # sim time at which the test starts
        self.done = False
        self.joint_log, self.odom_log = [], []
        self.get_logger().info("waiting for /joint_states ...")

    def torque(self, tau):
        s = tau - self.t_rest
        if 0.0 <= s < self.t_pulse:
            return self.amp * np.sin(2 * np.pi * s / self.t_pulse)
        return np.zeros(2)

    def on_joints(self, msg):
        t = stamp(msg)
        if self.t0 is None:
            self.t0 = t + 1.0   # let the effort controller activate first
            self.get_logger().info(f"first joint state at t = {t:.3f} s, test starts at {self.t0:.3f} s")
        tau = t - self.t0
        idx = [msg.name.index(j) for j in JOINTS]
        cmd = self.torque(tau) if tau >= 0 else np.zeros(2)
        self.pub.publish(Float64MultiArray(data=cmd.tolist()))
        if tau >= 0 and not self.done:
            pos = [msg.position[i] for i in idx]
            vel = [msg.velocity[i] for i in idx]
            eff = [msg.effort[i] for i in idx] if msg.effort else [np.nan, np.nan]
            self.joint_log.append([tau, *pos, *vel, *eff, *cmd])
            if tau > self.t_end:
                self.done = True

    def on_odom(self, msg):
        if self.t0 is None:
            return
        tau = stamp(msg) - self.t0
        if tau < 0 or self.done:
            return
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        self.odom_log.append([tau, p.x, p.y, p.z, *rpy(o), v.x, v.y, v.z, w.x, w.y, w.z])

    def save(self):
        j = np.array(self.joint_log)
        o = np.array(self.odom_log)
        np.savez(self.output,
                 joint_t=j[:, 0], q=j[:, 1:3], qd=j[:, 3:5], effort=j[:, 5:7], cmd=j[:, 7:9],
                 odom_t=o[:, 0], base_xyz=o[:, 1:4], base_rpy=o[:, 4:7],
                 base_v=o[:, 7:10], base_w=o[:, 10:13],
                 t_rest=self.t_rest, t_pulse=self.t_pulse, amplitude=self.amp)
        self.get_logger().info(f"saved {len(j)} joint samples, {len(o)} odometry samples -> {self.output}")


def main():
    rclpy.init()
    node = TorquePulseTest()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.pub.publish(Float64MultiArray(data=[0.0, 0.0]))
    node.save()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
