"""Full docking mission: guidance (reference path + pointing) for all phases.

Phases (time in s)
  1. Approach  0    - 3000 : from ~1 km behind to the hold point 10 m behind
  2. Hold      3000 - 3600 : wait at the hold point (checks, go/no-go)
  3. Final     3600 - 3850 : straight-line V-bar approach to contact at ~3 cm/s

Geometry (along the V-bar, LVLH y axis)
  target docking port at y = -1.00 m (on the target's rear face)
  chaser docking port is 0.75 m ahead of the chaser's centre of mass, on body axis b1
  -> contact when the chaser CoM reaches y = -1.75 m
"""
import numpy as np

import attitude as att
import orbit as o
import translation_control as tc

T_HOLD_END = 3600.0
T_FINAL = 250.0
T_CONTACT_PLAN = T_HOLD_END + T_FINAL
TARGET_PORT_Y = -1.00
CHASER_PORT = 0.75
Y_CONTACT = TARGET_PORT_Y - CHASER_PORT          # -1.75 m
V_CONTACT = 0.03                                 # planned closing speed at contact [m/s]
CONE_HALF_ANGLE = np.radians(10.0)               # approach corridor half-angle

# Assumed contact requirements (same order of magnitude as real docking systems)
REQ = dict(v_axial=(0.01, 0.10),     # m/s   closing speed window
           lateral=0.10,             # m     lateral misalignment of the ports
           v_lateral=0.02,           # m/s   lateral velocity
           angle=5.0,                # deg   angular misalignment
           rate=0.2)                 # deg/s angular rate relative to the target


def _hermite(tau, T, p0, p1, v0, v1):
    """Cubic Hermite: position p0 -> p1 and velocity v0 -> v1 in time T."""
    x = tau / T
    h00, h10, h01, h11 = 2*x**3 - 3*x**2 + 1, x**3 - 2*x**2 + x, -2*x**3 + 3*x**2, x**3 - x**2
    d00, d10, d01, d11 = 6*x**2 - 6*x, 3*x**2 - 4*x + 1, -6*x**2 + 6*x, 3*x**2 - 2*x
    a00, a10, a01, a11 = 12*x - 6, 6*x - 4, -12*x + 6, 6*x - 2
    pos = h00 * p0 + h10 * T * v0 + h01 * p1 + h11 * T * v1
    vel = (d00 * p0 + d10 * T * v0 + d01 * p1 + d11 * T * v1) / T
    acc = (a00 * p0 + a10 * T * v0 + a01 * p1 + a11 * T * v1) / T**2
    return pos, vel, acc


def translation_reference(t):
    """Reference LVLH state s_ref and CW feedforward acceleration u_ff."""
    if t < T_HOLD_END:
        return tc.reference(t)                       # phases 1 and 2 (from step 2)
    tau = t - T_HOLD_END
    if tau <= T_FINAL:
        y, vy, ay = _hermite(tau, T_FINAL, tc.Y_HOLD, Y_CONTACT, 0.0, V_CONTACT)
    else:                                            # keep pushing gently until contact
        y, vy, ay = Y_CONTACT + V_CONTACT * (tau - T_FINAL), V_CONTACT, 0.0
    s_ref = np.array([0.0, y, 0.0, 0.0, vy, 0.0])
    A, _ = o.cw_matrices()
    u_ff = np.array([0.0, ay, 0.0]) - (A @ s_ref)[3:]
    return s_ref, u_ff


def RN_reference(t):
    """Desired attitude: b1 along the line of sight to the target, b3 near the orbit normal."""
    rho = translation_reference(t)[0][:3]
    r1_L = -rho / np.linalg.norm(rho)
    z = np.array([0.0, 0.0, 1.0])
    r3_L = z - (z @ r1_L) * r1_L
    r3_L /= np.linalg.norm(r3_L)
    RL = np.vstack([r1_L, np.cross(r3_L, r1_L), r3_L])
    LN = o.eci_to_lvlh_dcm(*o.target_state(t))
    return RL @ LN


def _omega_r_N(t, h=0.5):
    """Reference angular velocity (N components) from [RN]' = -[w~][RN]."""
    RN, RNp, RNm = RN_reference(t), RN_reference(t + h), RN_reference(t - h)
    W = -((RNp - RNm) / (2 * h)) @ RN.T              # = [w_r~] in R components
    w_R = np.array([W[2, 1], W[0, 2], W[1, 0]])
    return RN.T @ w_R


def attitude_reference(t, h=0.5):
    """[RN], omega_r and its inertial derivative (both in N components)."""
    w = _omega_r_N(t)
    w_dot = (_omega_r_N(t + h) - _omega_r_N(t - h)) / (2 * h)
    return RN_reference(t), w, w_dot
