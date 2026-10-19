"""Reaching controller: bring the end effector onto the moving handle.

Guidance (where the hand should be), in three phases:
  approach  0 - 20 s : smooth blend from the start to a PRE-GRASP point 10 cm
                        outside the handle (moving with the target)
  sync     20 - 25 s : ride along with the pre-grasp point
  close-in 25 - 30 s : shrink the 10 cm offset to 0 -> hand on the handle
  (then keep tracking the handle; step 3 closes the gripper)

Control (two layers):
  1. resolved-rate, in inertial space, with the generalized Jacobian:
        qd_des = J*^+ ( v_ref + Kp (p_ref - p_e) )
     J*^+ is a damped least-squares inverse, so it stays finite near singularities.
  2. joint level, exact linearization of the free-floating dynamics:
        joint accelerations are  qdd_m = Hs^-1 tau + qdd_m0
        (Hs = generalized inertia matrix, qdd_m0 = acceleration with zero torque)
        choose  tau = Hs ( Kv (qd_des - qd_m) - qdd_m0 )
     so each joint velocity follows qd_des like a first-order system.
"""
import numpy as np

import gen_jacobian as gj              # (also puts project 1 on the import path)
import dynamics as d                   # project 1
import target as tg

T_APPROACH, T_SYNC, T_CLOSE, T_END = 20.0, 25.0, 30.0, 35.0
PRE_OFFSET = 0.10                       # pre-grasp stand-off [m]
KP = 1.0                                # task-space position gain [1/s]
KV = 10.0                               # joint velocity gain [1/s]
LAMBDA_MAX = 0.05                       # max damping of the least-squares inverse
W_DAMP = 0.08                           # start damping below this manipulability
TAU_MAX = 10.0                          # joint motor limit [N m]


def quintic(t, t0, t1):
    x = np.clip((t - t0) / (t1 - t0), 0.0, 1.0)
    s = 10 * x**3 - 15 * x**4 + 6 * x**5
    ds = (30 * x**2 - 60 * x**3 + 30 * x**4) / (t1 - t0) if t0 < t < t1 else 0.0
    return s, ds


class Guidance:
    def __init__(self, p_start):
        self.p_start = np.asarray(p_start, float)

    def reference(self, t):
        """Desired hand position and velocity (inertial)."""
        h, vh, _, n, dn = tg.handle(t)
        s_off, ds_off = quintic(t, T_SYNC, T_CLOSE)          # offset 10 cm -> 0
        off = PRE_OFFSET * (1 - s_off)
        doff = -PRE_OFFSET * ds_off
        p_goal = h + off * n
        v_goal = vh + off * dn + doff * n
        s, ds = quintic(t, 0.0, T_APPROACH)                   # start -> moving goal
        p = (1 - s) * self.p_start + s * p_goal
        v = ds * (p_goal - self.p_start) + s * v_goal
        return p, v


def dls_inverse(J):
    """Damped least-squares inverse J^T (J J^T + lambda^2 I)^-1."""
    w = gj.manipulability(J)
    lam = LAMBDA_MAX * (1 - w / W_DAMP) if w < W_DAMP else 0.0
    return J.T @ np.linalg.inv(J @ J.T + lam**2 * np.eye(2)), w


def joint_torques(t, q, qd, guidance):
    p_ref, v_ref = guidance.reference(t)
    p_e = gj.end_effector(q)
    Jinv, w = dls_inverse(gj.generalized_jacobian(q))
    qd_des = Jinv @ (v_ref + KP * (p_ref - p_e))
    # free-floating joint dynamics:  qdd_m = Hs^-1 tau + qdd_m0
    Minv = np.linalg.inv(d.mass_matrix(q))
    Hs = np.linalg.inv(Minv[3:, 3:])                          # generalized inertia matrix
    qdd_m0 = d.accelerations(q, qd, [0.0, 0.0])[3:]
    tau = Hs @ (KV * (qd_des - qd[3:]) - qdd_m0)
    return np.clip(tau, -TAU_MAX, TAU_MAX), w
