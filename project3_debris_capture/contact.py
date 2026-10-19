"""Grasp and contact models for capturing the target.

COMPLIANT GRASP (the main model)
  When the gripper closes, the hand and the handle are tied together by a
  stiff spring-damper in translation AND rotation (the gripper locks the
  relative angle it had at the moment of closing):
      F     = k  (p_handle - p_hand) + c  (v_handle - v_hand)     force on the hand
      Gamma = kr (th_t - phi2 - d0)  + cr (w_t - phi2_dot)       torque on the hand
  Equal and opposite force/torque act on the target, at the same point as on
  the hand (otherwise the tiny hand-handle gap would create a spurious couple
  and angular momentum would not be conserved exactly).
  Generalized forces on the chaser: J_hand^T F + J_phi2^T Gamma.

RIGID IMPACT (limit k -> infinity)
  The velocities jump instantly so that the relative velocity at the grasp
  (2 translations + 1 rotation) becomes zero, with momentum conserved:
      M u+ = M u- + A^T lambda,   A u+ = 0
  ->  lambda = -(A M^-1 A^T)^-1 A u-       (impulse [N s, N s, N m s])
"""
import numpy as np
import sympy as sp

import gen_jacobian as gj        # also puts project 1 on the import path
import dynamics as d
import target as tg

JPHI2 = np.array([0.0, 0.0, 1.0, 1.0, 1.0])        # phi2 = th0 + q1 + q2
_Jfull = sp.lambdify([d.q], sp.Matrix(gj.ee_sym).jacobian(d.q), "numpy")


def hand_jacobian(q):
    return np.array(_Jfull(np.asarray(q, float)), dtype=float)       # 2 x 5


def target_handle(xt):
    """Handle position/velocity and lever arm from the target state [x, y, th, vx, vy, w]."""
    c, th, v, w = xt[:2], xt[2], xt[3:5], xt[5]
    r = tg.HANDLE_R * np.array([np.cos(th), np.sin(th)])
    v_h = v + w * np.array([-r[1], r[0]])
    return c + r, v_h, r


class Gripper:
    def __init__(self, k, zeta=0.7, m_eff=20.0, i_eff=5.0, lever=0.1):
        self.k = k
        self.c = 2 * zeta * np.sqrt(k * m_eff)
        self.kr = k * lever**2                       # rotational stiffness [N m/rad]
        self.cr = 2 * zeta * np.sqrt(self.kr * i_eff)
        self.d0 = None                               # locked relative angle

    def wrench(self, q, qd, xt):
        p_h, v_h, _ = target_handle(xt)
        p_e = gj.end_effector(q)
        v_e = hand_jacobian(q) @ qd
        phi2, dphi2 = JPHI2 @ q, JPHI2 @ qd
        F = self.k * (p_h - p_e) + self.c * (v_h - v_e)
        G = self.kr * (xt[2] - phi2 - self.d0) + self.cr * (xt[5] - dphi2)
        return F, G


def coupled_rhs(t, x, grip, joint_torque):
    """x = [q(5), qd(5), target(6)]. Chaser + target joined by the gripper."""
    q, qd, xt = x[:5], x[5:10], x[10:]
    F, G = grip.wrench(q, qd, xt)
    tau_j = joint_torque(t, q, qd)
    Q = hand_jacobian(q).T @ F + JPHI2 * G                     # generalized contact forces
    Q[3:] += tau_j
    qdd = np.linalg.solve(d.mass_matrix(q), Q - d.coriolis(q, qd))
    r = gj.end_effector(q) - xt[:2]                         # lever: target centre -> contact point
    a_t = -F / tg.M_T
    alpha_t = (r[0] * (-F[1]) - r[1] * (-F[0]) - G) / tg.I_T   # torque of -F at the handle, and -G
    return np.concatenate([qd, qdd, xt[3:], a_t, [alpha_t]])


def rigid_impact(q, qd, xt):
    """Velocities after an instantaneous rigid grasp and the impulse (see header)."""
    Mc = d.mass_matrix(q)
    Mt = np.diag([tg.M_T, tg.M_T, tg.I_T])
    M = np.block([[Mc, np.zeros((5, 3))], [np.zeros((3, 5)), Mt]])
    r = gj.end_effector(q) - xt[:2]                     # contact point (on the hand)
    J_h = hand_jacobian(q)
    A = np.zeros((3, 8))
    A[:2, :5] = -J_h                                    # target point velocity - hand velocity
    A[:2, 5:] = np.array([[1, 0, -r[1]], [0, 1, r[0]]])
    A[2, :5] = -JPHI2                                   # target rate - phi2 rate
    A[2, 7] = 1.0
    u = np.concatenate([qd, xt[3:]])
    Minv = np.linalg.inv(M)
    lam = -np.linalg.solve(A @ Minv @ A.T, A @ u)
    # lambda acts on the target with +, on the hand with -  ->  impulse ON THE HAND = -lambda
    return u + Minv @ A.T @ lam, -lam, A @ u


def system_inertia(q, xt):
    """Centre of mass and inertia (about it) of chaser + target as one rigid body."""
    from kinematics import positions                    # project 1
    from params import m0, m1, m2, I0, I1, I2
    p = positions(q)
    bodies = [(m0, I0, p["r0"]), (m1, I1, p["c1"]), (m2, I2, p["c2"]), (tg.M_T, tg.I_T, xt[:2])]
    m_tot = sum(b[0] for b in bodies)
    cm = sum(b[0] * b[2] for b in bodies) / m_tot
    I = sum(b[1] + b[0] * np.sum((b[2] - cm) ** 2) for b in bodies)
    return cm, I, m_tot
