"""Full capture sequence with a reaction wheel on the chaser base.

  reach  (0 - 32 s)  : arm brings the hand onto the handle (step 2 controller),
                       the wheel is idle
  grasp  (32 s)      : gripper closes (step 3 compliant model)
  detumble (32 s - ) : the WHEEL absorbs the angular momentum of the whole stack
                       and brings the base back to its original attitude; the arm
                       joints hold their angles (PD) so the stack becomes rigid.

Reaction wheel model: a rotor with inertia I_W spinning at absolute rate W about
the base axis. The motor torque tau_w speeds the rotor up and pushes the base
the other way:   I_W W' = tau_w,   base gets  -tau_w.
Total angular momentum (chaser + target + rotor I_W W) is conserved.
Limits: |tau_w| <= 0.1 N m, |I_W W| <= 1.0 N m s (wheel speed limit).
"""
import os
import sys

import numpy as np
from scipy.integrate import solve_ivp

import contact as ct
import dynamics as d
import gen_jacobian as gj
import reach_control as rc
import target as tg

sys.path.insert(0, gj.P1)
from step4_conservation import _P, _L     # noqa: E402  (project 1, exact chaser momentum)

I_W = 1.6e-3                      # rotor inertia [kg m^2]  (1 N m s at 6000 rpm)
H_MAX = 1.0                       # wheel momentum capacity [N m s]
TAU_W_MAX = 0.1                   # wheel motor torque limit [N m]
T_GRASP = 32.0
K_GRIP = 1e3                      # gripper stiffness [N/m]
KP_J, KD_J = 5.0, 5.0             # joint hold gains after the grasp
I_STACK = 214.0                   # approximate inertia of the captured stack [kg m^2]
WN, ZETA = 0.03, 1.0              # base attitude loop: natural freq [rad/s], damping
KP_B, KD_B = I_STACK * WN**2, 2 * ZETA * WN * I_STACK
TH_REF = 0.0                      # bring the base back to its original attitude


def reach(q0):
    guid = rc.Guidance(gj.end_effector(q0))

    def rhs(t, x):
        q, qd = x[:5], x[5:]
        tau, _ = rc.joint_torques(t, q, qd, guid)
        return np.concatenate([qd, d.accelerations(q, qd, tau)])

    sol = solve_ivp(rhs, (0, T_GRASP), np.concatenate([q0, np.zeros(5)]),
                    method="DOP853", rtol=1e-9, atol=1e-11, max_step=0.05)
    return sol.y[:5, -1], sol.y[5:, -1]


def wheel_torque(q, qd, W):
    """Torque on the ROTOR. The base receives the opposite torque."""
    u_base = -KP_B * (q[2] - TH_REF) - KD_B * qd[2]           # what the base should feel
    tau_w = np.clip(-u_base, -TAU_W_MAX, TAU_W_MAX)
    h = I_W * W
    if (h >= H_MAX and tau_w > 0) or (h <= -H_MAX and tau_w < 0):
        tau_w = 0.0                                            # wheel at its speed limit
    return tau_w


def detumble_rhs(t, x, grip, q_hold):
    q, qd, xt, W = x[:5], x[5:10], x[10:16], x[16]
    F, G = grip.wrench(q, qd, xt)
    tau_j = -KP_J * (q[3:] - q_hold) - KD_J * qd[3:]
    tau_w = wheel_torque(q, qd, W)
    Q = ct.hand_jacobian(q).T @ F + ct.JPHI2 * G
    Q[3:] += tau_j
    Q[2] += -tau_w                                             # wheel reaction on the base
    qdd = np.linalg.solve(d.mass_matrix(q), Q - d.coriolis(q, qd))
    r = gj.end_effector(q) - xt[:2]
    a_t = -F / tg.M_T
    alpha_t = (r[0] * (-F[1]) - r[1] * (-F[0]) - G) / tg.I_T
    return np.concatenate([qd, qdd, xt[3:], a_t, [alpha_t], [tau_w / I_W]])


def total_L(x):
    q, qd, xt, W = x[:5], x[5:10], x[10:16], x[16]
    return (float(_L(q, qd)) + tg.M_T * (xt[0] * xt[4] - xt[1] * xt[3])
            + tg.I_T * xt[5] + I_W * W)


def run(spin_deg, t_detumble=400.0, n_out=4001):
    tg.set_spin(spin_deg)
    q0 = np.radians([0, 0, 0, 30, 100]); q0[:2] = 0
    qg, qdg = reach(q0)
    c, th, v, w = tg.state(T_GRASP)
    xt = np.array([*c, th, *v, w])
    grip = ct.Gripper(K_GRIP)
    grip.d0 = xt[2] - ct.JPHI2 @ qg
    x0 = np.concatenate([qg, qdg, xt, [0.0]])
    t = np.linspace(0, t_detumble, n_out)
    sol = solve_ivp(detumble_rhs, (0, t_detumble), x0, t_eval=t, args=(grip, qg[3:]),
                    method="DOP853", rtol=1e-9, atol=1e-11, max_step=0.02)
    return dict(t=t, X=sol.y.T, spin=spin_deg, q_grasp=qg)
