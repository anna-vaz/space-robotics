"""Pointing control with reaction wheels + magnetic momentum unloading.

Reference frames R (rows of [RN]):
  nadir   r3 = -r_hat (body +z, the camera end, looks down), r2 = -h_hat, r1 = r2 x r3 (~velocity)
  target  r3 = unit(p_target - r), r2 = unit(r3 x v), r1 = r2 x r3  (same as nadir when p = 0)
omega_RN and its derivative by central differences of [RN](t) (the target moves with
the Earth's rotation, so finite differences are simpler and exact enough).

Attitude law (body frame, wheels: h_w' = -u, J w' = -w x (J w + h_w) + u + tau_mtq + tau_d):
  eps    = vector part of the Euler parameters of [BR] (short way)
  w_des  = w_r + sat(-k_th * 2 eps, w_max)           rate-limited slew (wheel momentum!)
  u      = J (w_r' - w x w_r) + w x (J w + h_w) - P (w - w_des) - tau_mtq_expected
  P = 2 zeta wn J, k_th = wn / (2 zeta)  ->  linearised: th'' + 2 zeta wn th' + wn^2 th = 0
Unloading (cross-product law):
  m = (k_u / |B|^2) (h_w x B)   ->  tau_mtq = m x B = -k_u (I - b b^T) h_w
  only the part of h_w perpendicular to B can be dumped at any instant.
"""
import numpy as np
from scipy.linalg import sqrtm

import attitude as at
import environment as en
import params as pr

WN = 0.1                 # [rad/s] attitude bandwidth
ZETA = 0.9
W_MAX = np.radians(1.0)  # slew-rate limit: J_max w_max = 0.7 mN m s < 1.5 wheel capacity
K_UNLOAD = 5e-4          # [1/s] unloading gain (time constant ~ 2/(3 k) ~ 20 min)
DUTY = 0.8               # magnetorquers on for 80 % of each 1 s cycle


def nadir_or_target_frame(r, v, p=None):
    r3 = (-r if p is None else p - r)
    r3 = r3 / np.linalg.norm(r3)
    r2 = np.cross(r3, v); r2 /= np.linalg.norm(r2)
    return np.vstack([np.cross(r2, r3), r2, r3])


def target_eci(p_ecef, jd):
    return en.R3(en.gmst(jd)).T @ p_ecef


def reference(env, t, p_ecef=None, d=1.0):
    """[RN], omega_RN (R frame), omega_RN' (R frame) at time t."""
    def RN_at(tt):
        e = env.at(tt)
        p = None if p_ecef is None else target_eci(p_ecef, env.jd0 + tt / 86400)
        return nadir_or_target_frame(e["r"], e["v"], p)

    Rs = [RN_at(t + k * d) for k in (-2, -1, 0, 1, 2)]

    def omega(Rm, R0, Rp):
        W = -((Rp - Rm) / (2 * d)) @ R0.T            # [w x] in R components
        return 0.5 * np.array([W[2, 1] - W[1, 2], W[0, 2] - W[2, 0], W[1, 0] - W[0, 1]])

    w0 = omega(Rs[1], Rs[2], Rs[3])
    wdot = (omega(Rs[2], Rs[3], Rs[4]) - omega(Rs[0], Rs[1], Rs[2])) / (2 * d)
    return Rs[2], w0, wdot


class PointingController:
    def __init__(self, J=pr.J, wn=WN, zeta=ZETA, w_max=W_MAX, k_unload=K_UNLOAD):
        self.J = J
        self.P = 2 * zeta * wn * J
        self.k_th = wn / (2 * zeta)
        self.w_max = w_max
        self.k_u = k_unload

    def unloading_dipole(self, h_w, B_meas):
        if self.k_u == 0:
            return np.zeros(3)
        m = self.k_u / (B_meas @ B_meas) * np.cross(h_w, B_meas)
        s = np.max(np.abs(m)) / pr.MTQ_MAX
        return m / s if s > 1 else m

    def wheel_torque(self, BN, w, h_w, RN, w_r_R, wdot_r_R, tau_mtq_exp, dt=1.0):
        BR = BN @ RN.T
        eps = at.dcm2ep(BR)[1:]
        w_r = BR @ w_r_R
        wdot_r = BR @ wdot_r_R
        slew = -self.k_th * 2 * eps
        n = np.linalg.norm(slew)
        if n > self.w_max:
            slew *= self.w_max / n
        w_des = w_r + slew
        J = self.J
        u = (J @ (wdot_r - np.cross(w, w_r)) + np.cross(w, J @ w + h_w)
             - self.P @ (w - w_des) - tau_mtq_exp)
        s = np.max(np.abs(u)) / pr.RW_TMAX
        if s > 1:
            u /= s
        # do not push a wheel past its momentum capacity (h_w' = -u)
        for i in range(3):
            if abs(h_w[i] - u[i] * dt) > pr.RW_HMAX and np.sign(-u[i]) == np.sign(h_w[i]):
                u[i] = 0.0
        return u
