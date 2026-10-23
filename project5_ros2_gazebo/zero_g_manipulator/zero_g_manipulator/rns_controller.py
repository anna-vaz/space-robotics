"""Step 3: Reaction Null Space (RNS) joint-velocity controller.

Each control cycle (rate_hz) the node reads the joint angles and sends joint
velocity commands to the velocity_controller (ideal joint velocity servos):

    mode "commanded":  qd_cmd = qd_des                     (base rotates)
    mode "rns"      :  qd_cmd = P(q) qd_des,  P = I - b^T b / (b b^T)
                       = the part of qd_des that does NOT rotate the base

b(q) (base angular rate per unit joint rate at zero momentum) comes from the
model in free_floating_model.py, which only needs the mass matrix M(q). b does
not depend on the base pose, so only the joint angles are needed (no base sensor).

Timeline (test time tau): 0 .. t_move -> command, then zero, log until t_end.
t_ramp > 0 scales qd_des by a smooth 0 -> 1 -> 0 ramp (raised cosine) at the start
and end, so the joint servos never see a velocity step (a step needs more torque
than the 200 N m joint limit, and while a servo lags its command the joint rates
are NOT in the null space -> the base gets a kick).
Logs joint states, base odometry and the commands to an .npz file.
"""
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from zero_g_manipulator.free_floating_model import P1B_ARM, PlanarRobot
from zero_g_manipulator.step1_torque_pulse import rpy, stamp

JOINTS = ("joint1", "joint2", "joint3")


def ramp(tau, t_move, t_ramp):
    """Smooth 0 -> 1 -> 0 velocity scaling (raised cosine of length t_ramp at both ends)."""
    if t_ramp <= 0.0:
        return 1.0
    x = min(tau, t_move - tau) / t_ramp
    return 1.0 if x >= 1.0 else 0.5 - 0.5 * np.cos(np.pi * max(x, 0.0))


class RnsController(Node):
    def __init__(self):
        super().__init__("rns_controller")
        self.output = self.declare_parameter("output", "step3_gazebo.npz").value
        self.mode = self.declare_parameter("mode", "rns").value
        self.rate = self.declare_parameter("rate_hz", 1000.0).value
        self.qd_des = np.array(self.declare_parameter("qd_des", [0.20, -0.30, 0.40]).value)
        self.t_move = self.declare_parameter("t_move", 4.0).value
        self.t_end = self.declare_parameter("t_end", 5.0).value
        self.t_ramp = self.declare_parameter("t_ramp", 0.0).value
        self.robot = PlanarRobot(**P1B_ARM)

        self.pub = self.create_publisher(Float64MultiArray, "/velocity_controller/commands", 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 100)
        self.create_subscription(Odometry, "/model/space_robot/odometry", self.on_odom, 100)
        self.t0, self.t_last_cmd, self.cmd = None, -np.inf, np.zeros(3)
        self.done = False
        self.joint_log, self.odom_log = [], []
        self.get_logger().info(f"mode={self.mode}, rate={self.rate} Hz, qd_des={self.qd_des}")

    def control(self, tau, q_joints):
        qd_des = ramp(tau, self.t_move, self.t_ramp) * self.qd_des
        if self.mode == "rns":
            q = np.concatenate([[0.0, 0.0, 0.0], q_joints])   # b(q) is independent of the base pose
            return self.robot.rns_projector(q) @ qd_des
        return qd_des

    def on_joints(self, msg):
        t = stamp(msg)
        if self.t0 is None:
            self.t0 = t + 1.0
            self.get_logger().info(f"test starts at t = {self.t0:.3f} s")
        tau = t - self.t0
        idx = [msg.name.index(j) for j in JOINTS]
        q = np.array([msg.position[i] for i in idx])
        if 0.0 <= tau < self.t_move:
            if tau - self.t_last_cmd >= 1.0 / self.rate - 1e-6:   # controller rate (zero-order hold)
                self.cmd = self.control(tau, q)
                self.t_last_cmd = tau
        else:
            self.cmd = np.zeros(3)
        self.pub.publish(Float64MultiArray(data=self.cmd.tolist()))
        if tau >= -0.2 and not self.done:
            qd = [msg.velocity[i] for i in idx]
            self.joint_log.append([tau, *q, *qd, *self.cmd])
            if tau > self.t_end:
                self.done = True

    def on_odom(self, msg):
        if self.t0 is None or self.done:
            return
        tau = stamp(msg) - self.t0
        if tau < -0.2:
            return
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        self.odom_log.append([tau, p.x, p.y, p.z, *rpy(o)])

    def save(self):
        j, o = np.array(self.joint_log), np.array(self.odom_log)
        np.savez(self.output, joint_t=j[:, 0], q=j[:, 1:4], qd=j[:, 4:7], cmd=j[:, 7:10],
                 odom_t=o[:, 0], base_xyz=o[:, 1:4], base_rpy=o[:, 4:7],
                 mode=self.mode, rate_hz=self.rate, qd_des=self.qd_des, t_move=self.t_move,
                 t_ramp=self.t_ramp)
        self.get_logger().info(f"saved {len(j)} joint samples, {len(o)} odometry samples -> {self.output}")


def main():
    rclpy.init()
    node = RnsController()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.pub.publish(Float64MultiArray(data=[0.0, 0.0, 0.0]))
    node.save()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
