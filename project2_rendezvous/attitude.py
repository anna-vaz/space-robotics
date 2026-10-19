"""Attitude of the chaser: Euler parameters (quaternions), rigid-body dynamics, control.

Conventions (as in Schaub & Junkins, "Analytical Mechanics of Space Systems"):
  beta = [b0, b1, b2, b3]   Euler parameters of B relative to N, scalar first
  [BN]                      DCM: v_B = [BN] v_N
  omega                     angular velocity of B relative to N, in B components
Kinematics : beta' = 1/2 [B(beta)] omega
Dynamics   : J omega' = -omega x (J omega) + u          (u = control torque)
"""
import numpy as np

# ---- Chaser inertia (500 kg, roughly 1.5 m box, slightly asymmetric) ---------
J = np.array([[220.0,   5.0,  -3.0],
              [  5.0, 260.0,   4.0],
              [ -3.0,   4.0, 180.0]])            # [kg m^2]
J_INV = np.linalg.inv(J)
U_MAX = 0.2                                       # torque limit per axis [N m] (reaction wheels)


def tilde(v):
    """Skew-symmetric matrix so that tilde(a) @ b = a x b."""
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def B_matrix(b):
    b0, b1, b2, b3 = b
    return np.array([[-b1, -b2, -b3],
                     [ b0, -b3,  b2],
                     [ b3,  b0, -b1],
                     [-b2,  b1,  b0]])


def ep2dcm(b):
    b0, b1, b2, b3 = b
    return np.array([
        [b0**2 + b1**2 - b2**2 - b3**2, 2 * (b1 * b2 + b0 * b3),       2 * (b1 * b3 - b0 * b2)],
        [2 * (b1 * b2 - b0 * b3),       b0**2 - b1**2 + b2**2 - b3**2, 2 * (b2 * b3 + b0 * b1)],
        [2 * (b1 * b3 + b0 * b2),       2 * (b2 * b3 - b0 * b1),       b0**2 - b1**2 - b2**2 + b3**2]])


def dcm2ep(C):
    """Sheppard's method (robust near 180 deg); returns the short-rotation set (b0 >= 0)."""
    tr = np.trace(C)
    b2 = np.array([(1 + tr) / 4, (1 + 2 * C[0, 0] - tr) / 4,
                   (1 + 2 * C[1, 1] - tr) / 4, (1 + 2 * C[2, 2] - tr) / 4])
    i = int(np.argmax(b2))
    b = np.zeros(4)
    b[i] = np.sqrt(b2[i])
    if i == 0:
        b[1] = (C[1, 2] - C[2, 1]) / (4 * b[0]); b[2] = (C[2, 0] - C[0, 2]) / (4 * b[0]); b[3] = (C[0, 1] - C[1, 0]) / (4 * b[0])
    elif i == 1:
        b[0] = (C[1, 2] - C[2, 1]) / (4 * b[1]); b[2] = (C[0, 1] + C[1, 0]) / (4 * b[1]); b[3] = (C[2, 0] + C[0, 2]) / (4 * b[1])
    elif i == 2:
        b[0] = (C[2, 0] - C[0, 2]) / (4 * b[2]); b[1] = (C[0, 1] + C[1, 0]) / (4 * b[2]); b[3] = (C[1, 2] + C[2, 1]) / (4 * b[2])
    else:
        b[0] = (C[0, 1] - C[1, 0]) / (4 * b[3]); b[1] = (C[2, 0] + C[0, 2]) / (4 * b[3]); b[2] = (C[1, 2] + C[2, 1]) / (4 * b[3])
    return b if b[0] >= 0 else -b


def prv2ep(axis, angle):
    """Principal rotation (unit axis, angle) -> Euler parameters."""
    e = np.asarray(axis, float) / np.linalg.norm(axis)
    return np.concatenate([[np.cos(angle / 2)], e * np.sin(angle / 2)])


def rigid_body_rhs(beta, omega, u):
    """Time derivatives of beta and omega."""
    beta_dot = 0.5 * B_matrix(beta) @ omega
    omega_dot = J_INV @ (-np.cross(omega, J @ omega) + u)
    return beta_dot, omega_dot


# ---- Tracking controller -------------------------------------------------------
# Lyapunov-based PD + feedforward (Schaub & Junkins, ch. 8):
#   u = -K eps - P d_omega + J (omega_r_dot - omega x omega_r) + omega x (J omega)
#   eps     = vector part of the attitude error EPs of B relative to R (short way)
#   d_omega = omega - omega_r   (all in B components)
# With V = 1/2 d_omega^T J d_omega + 2K (1 - eps0):  V' = -d_omega^T P d_omega <= 0
WN, ZETA = 0.05, 0.9                              # closed-loop natural freq [rad/s], damping
K_GAIN = 2 * np.trace(J) / 3 * WN**2              # linearised: J th'' + (K/2) th + P th' = 0
P_GAIN = 2 * ZETA * WN * np.trace(J) / 3


def attitude_error(beta, RN):
    """Error EPs of B relative to the reference frame R, and [BR]."""
    BN = ep2dcm(beta)
    BR = BN @ RN.T
    return dcm2ep(BR), BR


def control_torque(beta, omega, RN, omega_r_N, omega_r_dot_N, saturate=True):
    """Control torque [N m] in B. Reference rates are given in N components
    (omega_r_dot_N = inertial derivative)."""
    d_beta, _ = attitude_error(beta, RN)
    BN = ep2dcm(beta)
    w_r = BN @ omega_r_N
    w_r_dot = BN @ omega_r_dot_N - np.cross(omega, w_r)   # derivative as seen from B
    d_omega = omega - w_r
    u = (-K_GAIN * d_beta[1:] - P_GAIN * d_omega          # PD on the errors
         + J @ w_r_dot + np.cross(omega, J @ omega))      # feedforward (cancels dynamics)
    return np.clip(u, -U_MAX, U_MAX) if saturate else u


def lyapunov(beta, omega, RN, omega_r_N):
    d_beta, _ = attitude_error(beta, RN)
    d_omega = omega - ep2dcm(beta) @ omega_r_N
    return 0.5 * d_omega @ J @ d_omega + 2 * K_GAIN * (1 - d_beta[0])


def error_angle(beta, RN):
    """Total attitude error angle [rad] = principal rotation angle of [BR]."""
    d_beta, _ = attitude_error(beta, RN)
    return 2 * np.arctan2(np.linalg.norm(d_beta[1:]), d_beta[0])   # accurate also near 0
