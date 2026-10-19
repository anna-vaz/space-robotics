"""Translation guidance and control for the chaser (LQR + feedforward).

Guidance : a smooth reference path along the V-bar (the target's direction of
           motion), from the start point to a hold point in front of the docking port.
Control  : u = u_ff(t) - K (s - s_ref(t)),   then clipped to the thruster limit.
           K comes from an LQR design on the CW model (s' = A s + B u).
           u_ff is the acceleration needed to follow the reference exactly in the
           CW model (includes the Coriolis term -2 n y'_ref on the radial axis).
"""
import numpy as np
from scipy.linalg import solve_continuous_are

import orbit as o

# ---- Chaser spacecraft ------------------------------------------------------
MASS = 500.0                    # [kg]
F_MAX = 4.0                     # thrust available per axis [N]
A_MAX = F_MAX / MASS            # 0.008 m/s^2 per axis

# ---- Mission profile ---------------------------------------------------------
S_START = np.array([30.0, -1000.0, 15.0, 0.0, 0.0, 0.0])   # 1 km behind, a bit off-axis
# The real chaser does not start exactly on the plan: navigation and previous
# manoeuvres leave errors. This offset tests that the feedback corrects them.
DISPERSION = np.array([5.0, -8.0, 3.0, 0.02, -0.03, 0.01])   # [m, m, m, m/s, m/s, m/s]
S_TRUE0 = S_START + DISPERSION
Y_HOLD = -10.0                  # hold point: 10 m behind the docking port, on V-bar
T_APPROACH = 3000.0             # duration of the approach [s] (~0.54 orbit)
T_END = 4000.0                  # then hold for 1000 s

# ---- LQR weights (Bryson's rule: 1 / (max acceptable value)^2) --------------
POS_ERR, VEL_ERR = 1.0, 0.05    # acceptable tracking errors [m], [m/s]
Q = np.diag([1 / POS_ERR**2] * 3 + [1 / VEL_ERR**2] * 3)
R = np.diag([1 / A_MAX**2] * 3)


def lqr_gain(Q=Q, R=R):
    A, B = o.cw_matrices()
    P = solve_continuous_are(A, B, Q, R)
    return np.linalg.solve(R, B.T @ P)          # K = R^-1 B^T P


K = lqr_gain()


def quintic(t, T):
    x = np.clip(t / T, 0.0, 1.0)
    s = 10 * x**3 - 15 * x**4 + 6 * x**5
    ds = (30 * x**2 - 60 * x**3 + 30 * x**4) / T
    dds = (60 * x - 180 * x**2 + 120 * x**3) / T**2
    if t >= T:
        ds = dds = 0.0
    return s, ds, dds


def reference(t, start=S_START):
    """Reference state and feedforward acceleration at time t.

    x and z go smoothly to 0 (onto the V-bar), y goes smoothly to Y_HOLD.
    """
    s, ds, dds = quintic(t, T_APPROACH)
    p0 = start[:3]
    p1 = np.array([0.0, Y_HOLD, 0.0])
    pos = p0 + s * (p1 - p0)
    vel = ds * (p1 - p0)
    acc = dds * (p1 - p0)
    s_ref = np.concatenate([pos, vel])
    # u_ff = what the CW model needs:  acc_ref = A s_ref + u_ff  (velocity rows)
    A, _ = o.cw_matrices()
    u_ff = acc - (A @ s_ref)[3:]
    return s_ref, u_ff


def control(t, s, saturate=True):
    """Thrust acceleration command in LVLH [m/s^2]."""
    s_ref, u_ff = reference(t)
    u = u_ff - K @ (s - s_ref)
    return np.clip(u, -A_MAX, A_MAX) if saturate else u
