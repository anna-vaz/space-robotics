"""Step 4: reach and capture a tumbling target with the free-floating 3-link arm.

Sequence (sim time, tau = test time):
  setup   : DetachableJoint starts welded -> send 'detach' while everything is at rest
  tau = 0 : one instantaneous torque spins the target up to w_T (Project 3: 3 deg/s)
  0 .. T1 : approach a stand-off point in front of the moving handle (reach.py, phase A)
            mode 'minnorm' -> plain J*^+ (base rotates) | 'rns' -> reactionless (P1b)
  T1 .. T2: final approach + spin synchronisation (phase B, both modes): the gripper
            points into the handle and turns with the target, so the weld below happens
            at ~zero relative motion
  capture : first time the gripper is within capture_tol of the handle after T2 ->
            'attach' (Gazebo welds gripper and handle = rigid-impact capture)
            and the joint servos lock (zero joint velocity)
  log until T2 + t_after.

Sensors used: joint angles, base pose (odometry; needed for the gripper position in the
inertial frame), target pose + spin (odometry). Commands: joint velocities.
"""
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.msg import EntityWrench
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64MultiArray, String

from zero_g_manipulator import reach as R
from zero_g_manipulator.free_floating_model import P1B_ARM, PlanarRobot
from zero_g_manipulator.step1_torque_pulse import rpy, stamp

JOINTS = ("joint1", "joint2", "joint3")
I_TARGET = 50.0 * 2.0 / 12.0       # target Izz [kg m^2]
DT_PHYSICS = 0.001


class CaptureController(Node):
    def __init__(self):
        super().__init__("capture_controller")
        self.output = self.declare_parameter("output", "step4_gazebo.npz").value
        self.mode = self.declare_parameter("mode", "rns").value
        self.rate = self.declare_parameter("rate_hz", 250.0).value
        self.spin = np.radians(self.declare_parameter("spin_deg_s", 3.0).value)
        self.tol = self.declare_parameter("capture_tol", 0.005).value
        self.t_after = self.declare_parameter("t_after", 12.0).value
        self.robot = PlanarRobot(**P1B_ARM)

        self.cmd_pub = self.create_publisher(Float64MultiArray, "/velocity_controller/commands", 10)
        self.attach_pub = self.create_publisher(Empty, "/space_robot/attach", 10)
        self.detach_pub = self.create_publisher(Empty, "/space_robot/detach", 10)
        self.wrench_pub = self.create_publisher(EntityWrench, "/world/zero_g/wrench", 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 100)
        self.create_subscription(Odometry, "/model/space_robot/odometry", self.on_base, 100)
        self.create_subscription(Odometry, "/model/target/odometry", self.on_target, 100)
        self.create_subscription(String, "/space_robot/attach_state", self.on_attach_state, 10)

        self.t0 = None
        self.base = None            # (x, y, yaw)
        self.target = None          # (x, y, yaw, vx, vy, wz)  world frame
        self.pe0 = None
        self.phi_start = None
        self.spun = self.captured = self.done = False
        self.t_capture = np.nan
        self.t_last_cmd, self.cmd = -np.inf, np.zeros(3)
        self.attach_state = ""
        self.joint_log, self.base_log, self.target_log = [], [], []
        self.get_logger().info(f"mode={self.mode}, rate={self.rate} Hz")

    # ------------------------------------------------------------------ sensors
    def on_base(self, msg):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = rpy(o)[2]
        self.base = (p.x, p.y, yaw)
        if self.t0 is not None and not self.done:
            self.base_log.append([stamp(msg) - self.t0, p.x, p.y, yaw])

    def on_target(self, msg):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        yaw = rpy(o)[2]
        c, s = np.cos(yaw), np.sin(yaw)          # odometry twist is in the body frame
        self.target = (p.x, p.y, yaw, c * v.x - s * v.y, s * v.x + c * v.y, w.z)
        if self.t0 is not None and not self.done:
            self.target_log.append([stamp(msg) - self.t0, *self.target])

    def on_attach_state(self, msg):
        if msg.data != self.attach_state:
            self.get_logger().info(f"DetachableJoint: {msg.data}")
        self.attach_state = msg.data

    # ------------------------------------------------------------------ control loop
    def on_joints(self, msg):
        t = stamp(msg)
        if self.t0 is None:
            self.t0 = t + 3.0
            self.detach_pub.publish(Empty())       # release the initial weld (all at rest)
            self.get_logger().info(f"detach sent; test starts at t = {self.t0:.3f} s")
        tau = t - self.t0
        idx = [msg.name.index(j) for j in JOINTS]
        qj = np.array([msg.position[i] for i in idx])
        if tau < -2.0 and self.attach_state != "detached":
            self.detach_pub.publish(Empty())       # repeat until Gazebo confirms
        if self.base is None or self.target is None:
            return
        q = np.r_[self.base, qj]

        if tau >= 0.0 and not self.spun:          # one-step torque: w = tau_z dt / I
            w = EntityWrench()
            w.entity.name = "target"
            w.entity.type = 2                      # MODEL (its canonical link)
            w.wrench.torque.z = I_TARGET * self.spin / DT_PHYSICS
            self.wrench_pub.publish(w)
            self.spun = True
            self.pe0 = self.robot.end_effector(q)

        cx, cy, th, vx, vy, wz = self.target
        p_h, v_h, n, dn = R.handle_state(np.array([cx, cy]), th, np.array([vx, vy]), wz)
        p_e = self.robot.end_effector(q)
        err = np.linalg.norm(p_e - p_h)
        detA = np.nan
        phi_err = R.wrap(R.gripper_angle(q) - (th + np.pi))
        if self.spun and not self.captured:
            if tau >= R.T2 and err < self.tol:
                self.attach_pub.publish(Empty())   # weld gripper to handle
                self.captured, self.t_capture = True, tau
                self.cmd = np.zeros(3)             # lock the joints
                self.get_logger().info(f"CAPTURE at tau = {tau:.3f} s, gripper-handle {err*1e3:.2f} mm,"
                                       f" gripper angle error {np.degrees(phi_err):.3f} deg")
            elif tau - self.t_last_cmd >= 1.0 / self.rate - 1e-6:
                p_ref, v_ref = R.reference(tau, self.pe0, p_h, v_h, n, dn)
                v = v_ref + R.KP * (p_ref - p_e)
                if tau < R.T1:
                    self.cmd, detA = R.joint_rates(self.robot, q, v, self.mode)
                else:
                    if self.phi_start is None:
                        self.phi_start = R.gripper_angle(q)
                    phr, dphr = R.angle_reference(tau, self.phi_start, th, wz)
                    phid = dphr + R.K_PHI * R.wrap(phr - R.gripper_angle(q))
                    self.cmd, detA = R.joint_rates_sync(self.robot, q, v, phid)
                self.t_last_cmd = tau
        if self.captured:
            self.cmd = np.zeros(3)
        self.cmd_pub.publish(Float64MultiArray(data=self.cmd.tolist()))

        if tau >= -0.5 and not self.done:
            qd = [msg.velocity[i] for i in idx]
            self.joint_log.append([tau, *qj, *qd, *self.cmd, err, detA, float(self.captured), phi_err])
            if tau > R.T2 + self.t_after or tau > R.T2 + 10.0 and not self.captured:
                self.done = True

    def save(self):
        j, b, g = (np.array(x) for x in (self.joint_log, self.base_log, self.target_log))
        np.savez(self.output, joint_t=j[:, 0], q=j[:, 1:4], qd=j[:, 4:7], cmd=j[:, 7:10],
                 grip_err=j[:, 10], detA=j[:, 11], captured=j[:, 12], phi_err=j[:, 13],
                 base_t=b[:, 0], base=b[:, 1:4], target_t=g[:, 0], target=g[:, 1:7],
                 t_capture=self.t_capture, mode=self.mode, rate_hz=self.rate, spin=self.spin)
        self.get_logger().info(f"saved -> {self.output} (capture at {self.t_capture:.3f} s)")


def main():
    rclpy.init()
    node = CaptureController()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.cmd_pub.publish(Float64MultiArray(data=[0.0, 0.0, 0.0]))
    node.save()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
