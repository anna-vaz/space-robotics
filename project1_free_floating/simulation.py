"""Simulation of the free-floating robot with a joint-space PD controller.

The joints follow a smooth (quintic) path from a start to a goal configuration.
The base is NOT controlled: we just watch how it reacts.
"""
import numpy as np
from scipy.integrate import solve_ivp

import dynamics as d

# ---- Desired joint motion -------------------------------------------------
Q_START = np.radians([0.0, 0.0])
Q_GOAL = np.radians([90.0, -90.0])
T_MOVE = 10.0          # duration of the move [s]
T_END = 15.0           # total simulated time [s] (hold the goal for 5 s)

# ---- PD gains (per joint) -------------------------------------------------
KP = np.array([40.0, 20.0])   # [N m / rad]
KD = np.array([30.0, 10.0])   # [N m s / rad]


def quintic(t, T=T_MOVE):
    """Smooth 0 -> 1 profile with zero velocity and acceleration at both ends.

    s(t) = 10 x^3 - 15 x^4 + 6 x^5, with x = t / T.
    Returns s, ds/dt.
    """
    x = np.clip(t / T, 0.0, 1.0)
    s = 10 * x**3 - 15 * x**4 + 6 * x**5
    ds = (30 * x**2 - 60 * x**3 + 30 * x**4) / T if t < T else 0.0
    return s, ds


def desired(t):
    s, ds = quintic(t)
    return Q_START + s * (Q_GOAL - Q_START), ds * (Q_GOAL - Q_START)


def controller(t, q, qd):
    """PD on the joint angles only -> joint torques [tau1, tau2]."""
    q_des, qd_des = desired(t)
    return KP * (q_des - q[3:]) + KD * (qd_des - qd[3:])


def rhs(t, x):
    """State-space form x = [q, qd]  ->  xdot = [qd, qdd]."""
    q, qd = x[:5], x[5:]
    tau = controller(t, q, qd)
    return np.concatenate([qd, d.accelerations(q, qd, tau)])


def simulate(q0=None, n_points=1501):
    q0 = np.zeros(5) if q0 is None else np.asarray(q0, float)
    x0 = np.concatenate([q0, np.zeros(5)])           # start at rest
    t_eval = np.linspace(0, T_END, n_points)
    sol = solve_ivp(rhs, (0, T_END), x0, t_eval=t_eval, method="DOP853",
                    rtol=1e-10, atol=1e-12)
    if not sol.success:
        raise RuntimeError(sol.message)
    q, qd = sol.y[:5].T, sol.y[5:].T
    tau = np.array([controller(t, qi, qdi) for t, qi, qdi in zip(sol.t, q, qd)])
    return sol.t, q, qd, tau
