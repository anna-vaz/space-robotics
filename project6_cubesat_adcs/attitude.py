"""Attitude kinematics and rigid-body dynamics (Schaub & Junkins conventions).

  beta = [b0, b1, b2, b3]  Euler parameters of body B relative to ECI N, scalar first
  [BN]                     DCM, v_B = [BN] v_N
  omega                    angular velocity of B relative to N, body components
Same conventions and functions as project2_rendezvous/attitude.py.
"""
import numpy as np


def tilde(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def B_matrix(b):
    b0, b1, b2, b3 = b
    return np.array([[-b1, -b2, -b3],
                     [b0, -b3, b2],
                     [b3, b0, -b1],
                     [-b2, b1, b0]])


def ep2dcm(b):
    b0, b1, b2, b3 = b
    return np.array([
        [b0**2 + b1**2 - b2**2 - b3**2, 2 * (b1 * b2 + b0 * b3), 2 * (b1 * b3 - b0 * b2)],
        [2 * (b1 * b2 - b0 * b3), b0**2 - b1**2 + b2**2 - b3**2, 2 * (b2 * b3 + b0 * b1)],
        [2 * (b1 * b3 + b0 * b2), 2 * (b2 * b3 - b0 * b1), b0**2 - b1**2 - b2**2 + b3**2]])


def dcm2ep(C):
    """Sheppard's method; returns the short-rotation set (b0 >= 0)."""
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
    e = np.asarray(axis, float) / np.linalg.norm(axis)
    return np.concatenate([[np.cos(angle / 2)], e * np.sin(angle / 2)])


def rigid_body_rhs(beta, omega, torque, J, J_inv):
    """beta' = 1/2 B(beta) omega ;  J omega' = -omega x J omega + torque."""
    return 0.5 * B_matrix(beta) @ omega, J_inv @ (-np.cross(omega, J @ omega) + torque)
