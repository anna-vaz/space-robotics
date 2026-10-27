"""Attitude determination: TRIAD and a multiplicative extended Kalman filter (MEKF).

Conventions as in attitude.py: beta = Euler parameters of B relative to N (scalar first),
[BN] maps N components to B components.

MEKF (Lefferts, Markley & Shuster 1982; Crassidis & Junkins ch. 7):
  estimate      beta_hat, b_hat (gyro bias)
  error state   dx = [d_alpha, d_b],  true [BN] = (I - [d_alpha x]) [BN]_hat
  propagate     w_hat = w_m - b_hat ; beta_hat exactly for constant w_hat over dt
                d_alpha' = -[w_hat x] d_alpha - d_b - eta_v ;  d_b' = eta_u
  measure       unit vector v_B = [BN] r_N  ->  residual v_B - [BN]_hat r_N = [(BN_hat r_N) x] d_alpha
  reset         [BN]_hat <- (I - [d_alpha x]) [BN]_hat ,  b_hat <- b_hat + d_b
"""
import numpy as np
from scipy.linalg import expm

import attitude as at


def triad(v1_B, v2_B, v1_N, v2_N):
    """[BN] from two vector pairs; v1 is trusted most (exactly matched)."""
    def frame(a, b):
        t1 = a / np.linalg.norm(a)
        t2 = np.cross(a, b); t2 /= np.linalg.norm(t2)
        return np.column_stack([t1, t2, np.cross(t1, t2)])
    return frame(v1_B, v2_B) @ frame(v1_N, v2_N).T


def omega_matrix(w):
    """Omega(w) with beta' = 1/2 Omega(w) beta  (= 1/2 B(beta) w)."""
    w1, w2, w3 = w
    return np.array([[0, -w1, -w2, -w3],
                     [w1, 0, w3, -w2],
                     [w2, -w3, 0, w1],
                     [w3, w2, -w1, 0]])


def propagate_beta(beta, w, dt):
    """Exact solution of beta' = 1/2 Omega(w) beta for constant w."""
    wn = np.linalg.norm(w)
    if wn < 1e-14:
        return beta
    a = 0.5 * wn * dt
    beta = (np.cos(a) * np.eye(4) + np.sin(a) / wn * omega_matrix(w)) @ beta
    return beta / np.linalg.norm(beta)


def attitude_error(BN_true, BN_est):
    """Small-angle error vector d_alpha with BN_true = (I - [d_alpha x]) BN_est, and the
    principal angle. Uses the Euler parameters of dC = BN_true BN_est^T."""
    dC = BN_true @ BN_est.T
    db = at.dcm2ep(dC)
    ang = 2 * np.arctan2(np.linalg.norm(db[1:]), db[0])
    # ep2dcm([1, e]) ~ I - 2[e x]  ->  d_alpha = 2 e (to first order; exact direction)
    return 2 * db[1:] * (ang / (2 * np.sin(ang / 2)) if ang > 1e-12 else 1.0), ang


class MEKF:
    def __init__(self, beta0, b0, P0, sigma_v, sigma_u):
        self.beta = np.array(beta0, float)
        self.b = np.array(b0, float)
        self.P = np.array(P0, float)
        self.sv, self.su = sigma_v, sigma_u

    def propagate(self, w_m, dt):
        w = w_m - self.b
        F = np.zeros((6, 6))
        F[:3, :3] = -at.tilde(w)
        F[:3, 3:] = -np.eye(3)
        Phi = expm(F * dt)
        sv2, su2 = self.sv**2, self.su**2
        Q = np.block([[(sv2 * dt + su2 * dt**3 / 3) * np.eye(3), -(su2 * dt**2 / 2) * np.eye(3)],
                      [-(su2 * dt**2 / 2) * np.eye(3), su2 * dt * np.eye(3)]])
        self.beta = propagate_beta(self.beta, w, dt)
        self.P = Phi @ self.P @ Phi.T + Q

    def update(self, v_B, r_N, sigma):
        """One unit-vector measurement with isotropic noise sigma [rad]."""
        BN = at.ep2dcm(self.beta)
        v_hat = BN @ r_N
        H = np.hstack([at.tilde(v_hat), np.zeros((3, 3))])
        S = H @ self.P @ H.T + sigma**2 * np.eye(3)
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ (v_B - v_hat)
        IKH = np.eye(6) - K @ H
        self.P = IKH @ self.P @ IKH.T + sigma**2 * K @ K.T       # Joseph form
        self.P = 0.5 * (self.P + self.P.T)
        # reset: BN <- (I - [da x]) BN  ==  ep2dcm([1, da/2]) BN to first order
        dq = np.concatenate([[1.0], 0.5 * dx[:3]]); dq /= np.linalg.norm(dq)
        self.beta = at.dcm2ep(at.ep2dcm(dq) @ BN)
        self.b = self.b + dx[3:]
