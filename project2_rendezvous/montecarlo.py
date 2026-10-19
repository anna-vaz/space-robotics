"""Monte Carlo robustness test of the docking mission (step 4b).

Differences from step 4 (which was ideal):
  * The flight software runs at a fixed rate (1 Hz). It reads NOISY measurements,
    computes commands, and holds them constant until the next sample (zero-order
    hold). This is how real on-board controllers work, and it also lets us add
    random noise without upsetting the adaptive ODE solver.
  * The real spacecraft differs from the model the controller uses
    (mass, inertia, thruster alignment and strength), plus a small drag disturbance.
  * Start conditions are randomized (position, velocity, attitude, rates).
  * Navigation: the sensor gives only a noisy relative POSITION. A Kalman filter
    built on the CW model estimates position and velocity for the controller.
The truth dynamics are integrated with fixed-step RK4 (0.25 s) between samples.
"""
import numpy as np
from scipy.linalg import expm

import attitude as att
import mission as m
import orbit as o
import translation_control as tc

DT_CTRL = 1.0                   # flight-software period [s]
N_SUB = 4                       # RK4 sub-steps per period (0.25 s)
T_MAX = m.T_CONTACT_PLAN + 300

# ---- Uncertainty model (1-sigma unless "uniform") ------------------------------
UNC = dict(
    pos_noise_const=0.01,       # m     relative-position noise floor
    pos_noise_frac=0.005,       # -     + 0.5 % of range (camera/lidar-like)
    kf_accel_sigma=5e-5,        # m/s^2 Kalman filter process noise (unmodelled accelerations)
    att_noise_deg=0.05,         # deg   star-tracker attitude noise, per axis
    rate_noise_deg=0.001,       # deg/s gyro noise, per axis
    thr_misalign_deg=1.0,       # deg   misalignment of each thruster axis
    thr_scale=0.03,             # -     thrust magnitude error per axis
    mass_unif=0.10,             # -     true mass = nominal * (1 +- 10 %) (uniform)
    inertia_unif=0.10,          # -     principal-axis inertia scaling +- 10 % (uniform)
    drag_max=2e-7,              # m/s^2 differential drag, uniform 0 .. max (along -y)
    start_pos=10.0,             # m     start position dispersion per axis
    start_vel=0.02,             # m/s   start velocity dispersion per axis
    start_rate_deg=1.0,         # deg/s initial tumble rate per axis
)


class CWKalmanFilter:
    """Discrete Kalman filter for the relative state, using the CW model.

    Predict : x = Phi x + Gamma a_cmd          P = Phi P Phi^T + Qd
    Update  : z = position + noise             (standard KF equations)
    Phi and Gamma are the exact discretization of the CW model over DT_CTRL
    (acceleration held constant over the period).
    """

    def __init__(self, x0, P0):
        A, B = o.cw_matrices()
        M = expm(np.block([[A, B], [np.zeros((3, 9))]]) * DT_CTRL)
        self.Phi, self.Gam = M[:6, :6], M[:6, 6:]
        self.Qd = UNC["kf_accel_sigma"]**2 * self.Gam @ self.Gam.T
        self.H = np.hstack([np.eye(3), np.zeros((3, 3))])
        self.x, self.P = np.array(x0, float), np.array(P0, float)

    def update(self, z, sigma):
        S = self.H @ self.P @ self.H.T + sigma**2 * np.eye(3)
        Kk = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + Kk @ (z - self.H @ self.x)
        self.P = (np.eye(6) - Kk @ self.H) @ self.P

    def predict(self, a_cmd_lvlh):
        self.x = self.Phi @ self.x + self.Gam @ a_cmd_lvlh
        self.P = self.Phi @ self.P @ self.Phi.T + self.Qd


def _small_rotation(rng, sigma_rad):
    """DCM of a random small rotation (axis-angle with sigma per axis)."""
    phi = rng.normal(0, sigma_rad, 3)
    ang = np.linalg.norm(phi)
    if ang < 1e-15:
        return np.eye(3)
    return att.ep2dcm(att.prv2ep(phi / ang, ang))


def _random_attitude(rng):
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    return q if q[0] >= 0 else -q


def scaled_unc(scale):
    """Uncertainty model with every real-world error multiplied by `scale`
    (the Kalman filter tuning stays the same: it is the designer's assumption)."""
    return {k: (v if k == "kf_accel_sigma" else v * scale) for k, v in UNC.items()}


def draw_case(seed, dispersed=True, scale=1.0):
    """All the random quantities of one mission."""
    rng = np.random.default_rng(seed)
    if not dispersed:   # reproduces step 4 exactly (except discrete-time control)
        RN0 = m.RN_reference(0.0)
        beta0 = att.dcm2ep(att.ep2dcm(att.prv2ep([0.3, -1, 0.5], np.radians(150))) @ RN0)
        return dict(rng=None, s0=tc.S_TRUE0.copy(), beta0=beta0,
                    omega0=np.radians([1.0, -0.5, 0.8]), mass_ratio=1.0, J=att.J.copy(),
                    thr=np.eye(3), drag=0.0)
    u = scaled_unc(scale)
    s0 = tc.S_START + np.concatenate([rng.normal(0, u["start_pos"], 3), rng.normal(0, u["start_vel"], 3)])
    mis = np.column_stack([_small_rotation(rng, np.radians(u["thr_misalign_deg"])) @ e for e in np.eye(3)])
    thr = mis @ np.diag(1 + rng.normal(0, u["thr_scale"], 3))
    S = np.diag(np.sqrt(1 + rng.uniform(-u["inertia_unif"], u["inertia_unif"], 3)))
    return dict(rng=rng, s0=s0, beta0=_random_attitude(rng),
                omega0=rng.normal(0, np.radians(u["start_rate_deg"]), 3),
                mass_ratio=1 / (1 + rng.uniform(-u["mass_unif"], u["mass_unif"])),   # m_nom / m_true
                J=S @ att.J @ S, thr=thr, drag=rng.uniform(0, u["drag_max"]))


def _truth_rhs(t, X, a_B, torque, case, J_inv):
    """True dynamics with commands held constant over the sample period."""
    beta, omega = X[6:10], X[10:13]
    BN = att.ep2dcm(beta)
    d = np.empty(13)
    d[:6] = o.two_body_rhs(t, X[:6])
    d[3:6] += BN.T @ a_B                               # body-fixed thrusters
    if case["drag"]:
        LN = o.eci_to_lvlh_dcm(*o.target_state(t))
        d[3:6] += LN.T @ np.array([0.0, -case["drag"], 0.0])
    d[6:10] = 0.5 * att.B_matrix(beta) @ omega
    d[10:13] = J_inv @ (-np.cross(omega, case["J"] @ omega) + torque)
    return d


def _rk4(f, t, X, h):
    k1 = f(t, X)
    k2 = f(t + h / 2, X + h / 2 * k1)
    k3 = f(t + h / 2, X + h / 2 * k2)
    k4 = f(t + h, X + h * k3)
    Xn = X + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    Xn[6:10] /= np.linalg.norm(Xn[6:10])               # keep |beta| = 1 (fixed-step RK4)
    return Xn


def _port(rel, beta, t):
    LN = o.eci_to_lvlh_dcm(*o.target_state(t))
    b1_L = LN @ att.ep2dcm(beta)[0]
    return rel[:3] + m.CHASER_PORT * b1_L


def run(seed, dispersed=True, keep_history=False, scale=1.0):
    case = draw_case(seed, dispersed, scale)
    rng, u = case["rng"], scaled_unc(scale)
    J_inv = np.linalg.inv(case["J"])
    X = np.concatenate([o.lvlh_to_eci(case["s0"], 0.0), case["beta0"], case["omega0"]])
    t, dv, max_cone, cone_range, hist, est_err, kf = 0.0, 0.0, 0.0, np.nan, [], [], None
    while t < T_MAX:
        rel = o.eci_to_lvlh(X[:6], t)
        beta, omega = X[6:10], X[10:13]
        # ---- measurements + navigation ----
        if rng is not None:
            sig = u["pos_noise_const"] + u["pos_noise_frac"] * np.linalg.norm(rel[:3])
            z = rel[:3] + rng.normal(0, sig, 3)
            if kf is None:                             # first fix: start the filter
                kf = CWKalmanFilter(np.concatenate([z, np.zeros(3)]),
                                    np.diag([sig**2] * 3 + [(2 * u["start_vel"])**2] * 3))
            else:
                kf.update(z, sig)
            rel_m = kf.x.copy()
            beta_m = att.dcm2ep(_small_rotation(rng, np.radians(u["att_noise_deg"])) @ att.ep2dcm(beta))
            omega_m = omega + rng.normal(0, np.radians(u["rate_noise_deg"]), 3)
            if keep_history:
                est_err.append(np.concatenate([rel_m - rel]))
        else:
            rel_m, beta_m, omega_m = rel, beta, omega
        # ---- flight software (nominal model) ----
        s_ref, u_ff = m.translation_reference(t)
        a_L = u_ff - tc.K @ (rel_m - s_ref)
        LN = o.eci_to_lvlh_dcm(*o.target_state(t))
        a_cmd = np.clip(att.ep2dcm(beta_m) @ LN.T @ a_L, -tc.A_MAX, tc.A_MAX)
        RN, w_r, w_r_dot = m.attitude_reference(t)
        torque = att.control_torque(beta_m, omega_m, RN, w_r, w_r_dot)
        if kf is not None:                             # what the filter thinks was applied
            kf.predict(LN @ att.ep2dcm(beta_m).T @ a_cmd)
        # ---- what the real hardware delivers ----
        a_act = case["mass_ratio"] * case["thr"] @ a_cmd
        dv += np.abs(a_cmd).sum() * DT_CTRL            # commanded delta-v (3 axes)
        # ---- bookkeeping ----
        port = _port(rel, beta, t)
        axial = m.TARGET_PORT_Y - port[1]
        if axial > 0.5:
            cone_now = np.arctan2(np.hypot(port[0], port[2]), axial)
            if cone_now > max_cone:
                max_cone, cone_range = cone_now, axial
        if keep_history:
            hist.append((t, *rel, *port, att.error_angle(beta, m.RN_reference(t))))
        # ---- propagate the truth for one period ----
        X_old, t_old = X, t
        f = lambda tt, XX: _truth_rhs(tt, XX, a_act, torque, case, J_inv)
        h = DT_CTRL / N_SUB
        for _ in range(N_SUB):
            X = _rk4(f, t, X, h)
            t += h
        rel_new = o.eci_to_lvlh(X[:6], t)
        if rel_new[1] >= m.Y_CONTACT:                  # contact during this period
            # interpolate in LVLH (in ECI the chaser moves 7.7 km per second!)
            frac = (m.Y_CONTACT - rel[1]) / (rel_new[1] - rel[1])
            t_c = t_old + frac * DT_CTRL
            rel_c = rel + frac * (rel_new - rel)
            beta_c = X_old[6:10] + frac * (X[6:10] - X_old[6:10])
            omega_c = X_old[10:13] + frac * (X[10:13] - X_old[10:13])
            return dict(seed=seed, contact=True, t_c=t_c, dv=dv, max_cone=max_cone, cone_range=cone_range,
                        **contact_metrics(t_c, rel_c, beta_c / np.linalg.norm(beta_c), omega_c),
                        hist=np.array(hist) if keep_history else None,
                        est_err=np.array(est_err) if keep_history else None)
    return dict(seed=seed, contact=False, t_c=np.nan, dv=dv, max_cone=max_cone, cone_range=cone_range,
                hist=np.array(hist) if keep_history else None)


def contact_metrics(t_c, r, beta, omega):
    port = _port(r, beta, t_c)
    LN = o.eci_to_lvlh_dcm(*o.target_state(t_c))
    dock_RN = np.vstack([[0, 1, 0], [-1, 0, 0], [0, 0, 1]]) @ LN
    w_rel = omega - att.ep2dcm(beta) @ (dock_RN.T @ np.array([0, 0, o.N]))
    return dict(v_axial=r[4], lateral=np.hypot(port[0], port[2]), v_lateral=np.hypot(r[3], r[5]),
                angle=np.degrees(att.error_angle(beta, dock_RN)),
                rate=np.degrees(np.linalg.norm(w_rel)))


def passes(res):
    if not res["contact"]:
        return False
    R = m.REQ
    return (R["v_axial"][0] <= res["v_axial"] <= R["v_axial"][1] and res["lateral"] <= R["lateral"]
            and res["v_lateral"] <= R["v_lateral"] and res["angle"] <= R["angle"]
            and res["rate"] <= R["rate"] and res["max_cone"] <= m.CONE_HALF_ANGLE)
