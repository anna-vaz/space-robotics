"""Closed-loop ADCS simulation: truth dynamics + sensors + MEKF + pointing + unloading.

Timeline of one 1 s flight-software cycle starting at t_k:
  * gyro samples every 0.1 s propagate the MEKF
  * at t_k the coils have been off for 0.2 s: magnetometer + sun sensors -> MEKF update
  * reference, wheel torque u (held 1 s), unloading dipole m (on for 0.8 s)
Truth: fixed-step RK4 (0.1 s) on x = [beta, omega, h_w, H_ext], where H_ext integrates the
external torque in ECI, so H_N(t) - H_N(0) = H_ext(t) is a built-in momentum check.
`truth` overrides the plant (Monte Carlo): J, m_res, face centres. The flight software
always uses the nominal parameters.
"""
import numpy as np
from scipy.interpolate import CubicSpline

import attitude as at
import controller as ct
import environment as en
import estimation as es
import params as pr
import sensors as sn

R_TUNED = dict(model_infl=10.0, sig_sun=0.03)      # from step 3
H = 0.1                                            # truth step = gyro period [s]


def onboard_field(env):
    """Onboard field model (IGRF degree 8) as a spline over the environment grid."""
    return CubicSpline(env.t, en.igrf_eci(env.r, env.jd, env.field_date, max_degree=8))


def simulate(env, B8, t0, T, beta0, w0, h0=np.zeros(3), mode_fn=None, unload=True,
             estimator=True, seed=0, truth=None, P0_att=np.radians(3.0), P0_bias=np.radians(0.3),
             b0=None, k_unload=ct.K_UNLOAD):
    rng = np.random.default_rng(seed)
    truth = truth or {}
    J_t = truth.get("J", pr.J)
    J_t_inv = np.linalg.inv(J_t)
    m_res = truth.get("m_res", pr.M_RESIDUAL)
    centres = truth.get("centres", None)
    ctrl = ct.PointingController(k_unload=k_unload if unload else 0.0)
    sig_model = np.sqrt(np.mean((env.B - B8(env.t))**2))
    if mode_fn is None:
        def mode_fn(t):
            return None                                   # nadir

    x = np.concatenate([beta0, w0, h0, np.zeros(3)])
    gyro_b = sn.GYRO_B0.copy() if b0 is None else np.array(b0, float)
    u = np.zeros(3); m = np.zeros(3)

    def f(t, x, u, m):
        beta, w, hw = x[:4], x[4:7], x[7:10]
        e = env.at(t)
        BN = at.ep2dcm(beta)
        tq_m = np.cross(m, BN @ e["B"])
        tq_d = sum(en.disturbance_torques(BN, e, m_res=m_res, J=J_t, centres=centres).values())
        wd = J_t_inv @ (-np.cross(w, J_t @ w + hw) + u + tq_m + tq_d)
        return np.concatenate([0.5 * at.B_matrix(beta) @ w, wd, -u, BN.T @ (tq_m + tq_d)])

    n = int(round(T))
    keys = ["t", "beta", "w", "hw", "u", "m", "know", "point", "zerr", "Hchk", "bias_err", "mode", "sun"]
    out = {k: [] for k in keys}
    filt = None
    w_m = x[4:7] + gyro_b
    for k in range(n):
        t = t0 + k
        x[:4] /= np.linalg.norm(x[:4])
        e = env.at(t)
        BN_true = at.ep2dcm(x[:4])
        # --- sensors (coils off) ---
        Bm = sn.magnetometer(BN_true @ e["B"], rng)
        s_N = e["sun"] / np.linalg.norm(e["sun"])
        sm = sn.coarse_sun_sensors(BN_true @ s_N, e["shadow"], rng)
        Bref = B8(t)
        mb, mn = Bm / np.linalg.norm(Bm), Bref / np.linalg.norm(Bref)
        # --- attitude determination ---
        if estimator:
            if filt is None:
                if sm is not None:
                    beta_i = at.dcm2ep(es.triad(mb, sm, mn, s_N))
                else:                                  # hand-over from the previous mode
                    da = P0_att * rng.standard_normal(3)
                    beta_i = at.dcm2ep(at.ep2dcm(at.prv2ep(da, np.linalg.norm(da))) @ BN_true)
                filt = es.MEKF(beta_i, np.zeros(3), np.diag([P0_att**2] * 3 + [P0_bias**2] * 3),
                               sn.SIGMA_V, sn.SIGMA_U)
            else:
                sm_dir = np.sqrt(sn.SIGMA_MAG**2 + (R_TUNED["model_infl"] * sig_model)**2) / np.linalg.norm(Bm)
                filt.update(mb, mn, sm_dir)
                if sm is not None:
                    filt.update(sm, s_N, R_TUNED["sig_sun"])
            BN_est, w_est = at.ep2dcm(filt.beta), w_m - filt.b
        else:
            BN_est, w_est = BN_true, x[4:7].copy()
        # --- guidance + control ---
        p = mode_fn(t)
        RN, wr, wrd = ct.reference(env, t, p)
        hw = x[7:10].copy()                            # wheel tachometers
        m = ctrl.unloading_dipole(hw, Bm) if unload else np.zeros(3)
        u = ctrl.wheel_torque(BN_est, w_est, hw, RN, wr, wrd, ct.DUTY * np.cross(m, Bm))
        # --- log ---
        BR_true = BN_true @ RN.T
        Hn = BN_true.T @ (J_t @ x[4:7] + hw)
        if k == 0:
            H0 = Hn.copy()
        zb = BN_true.T @ np.array([0, 0, 1.0])
        out["t"].append(t); out["beta"].append(x[:4].copy()); out["w"].append(x[4:7].copy())
        out["hw"].append(hw); out["u"].append(u.copy()); out["m"].append(m.copy())
        out["know"].append(es.attitude_error(BN_true, BN_est)[1])
        out["point"].append(2 * np.arctan2(np.linalg.norm(at.dcm2ep(BR_true)[1:]), at.dcm2ep(BR_true)[0]))
        out["zerr"].append(np.arccos(np.clip(zb @ RN[2], -1, 1)))
        out["Hchk"].append(np.linalg.norm(Hn - H0 - x[10:13]))
        out["bias_err"].append(np.linalg.norm(gyro_b - filt.b) if estimator else 0.0)
        out["mode"].append(p is not None); out["sun"].append(sm is not None)
        # --- propagate 1 s: 10 truth steps, gyro samples feed the MEKF ---
        for j in range(10):
            tj = t + j * H
            mj = m if j < int(round(ct.DUTY / H)) else np.zeros(3)
            k1 = f(tj, x, u, mj); k2 = f(tj + H / 2, x + H / 2 * k1, u, mj)
            k3 = f(tj + H / 2, x + H / 2 * k2, u, mj); k4 = f(tj + H, x + H * k3, u, mj)
            x = x + H / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            gyro_new = gyro_b + sn.SIGMA_U * np.sqrt(H) * rng.standard_normal(3)
            w_m_new = (x[4:7] + 0.5 * (gyro_b + gyro_new)
                       + np.sqrt(sn.SIGMA_V**2 / H + sn.SIGMA_U**2 * H / 12) * rng.standard_normal(3))
            if estimator:
                filt.propagate(w_m, H)
            w_m, gyro_b = w_m_new, gyro_new
    return {k: np.array(v) for k, v in out.items()}
