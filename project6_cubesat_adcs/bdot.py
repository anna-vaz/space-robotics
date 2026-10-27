"""B-dot magnetic detumbling.

Control law (unit-vector form, Avanzini & Giulietti, JGCD 35(4), 2012):
    m = -(k / |B|) b_dot,      b = B_B / |B_B|  (field direction seen in the body)
Since b_dot ~ -omega x b (the body rotates under a slowly varying field), the torque is
    tau = m x B = -k (I - b b^T) omega = -k omega_perp
i.e. pure damping of the rate component perpendicular to B; the component along B
cannot be touched (tau . B = 0 always). Energy rate T' = omega . tau = -k |omega_perp|^2.
Gain: k_opt = 2 n (1 + sin xi_m) J_min  (xi_m = orbit inclination w.r.t. the geomagnetic
equator).

Two implementations:
  * simulate_continuous: ideal law with the exact b_dot (b_dot from the true rates and
    the field's own orbital variation), integrated with DOP853.
  * simulate_flight: what flies -- 1 Hz magnetometer with noise, b_dot by finite
    difference of consecutive samples, magnetorquers on for `duty` of each second and off
    while the magnetometer samples (otherwise the coils corrupt the measurement).
Saturation scales the whole dipole vector (keeps its direction -> still dissipative).
"""
import numpy as np
from scipy.integrate import solve_ivp

import attitude as at
import environment as en
import orbit as ob
import params as pr


def gain_opt(env):
    """k_opt = 2 n (1 + sin xi_m) J_min with xi_m from the tilted dipole axis."""
    g10, g11, h11 = en.dipole_coefficients(env.field_date)
    m_ecef = np.array([g11, h11, g10]) / np.sqrt(g10**2 + g11**2 + h11**2)
    h = np.cross(env.r[0], env.v[0]); h /= np.linalg.norm(h)
    # dipole axis rotates with the Earth: average |sin xi_m| over one day of Earth rotation
    sins = []
    for th in np.linspace(0, 2 * np.pi, 73)[:-1]:
        m = en.R3(th).T @ m_ecef
        sins.append(abs(np.cross(h, m) @ np.cross(h, m)) ** 0.5)   # sin(angle between orbit normal and dipole)
    sin_xi = np.mean(sins)          # angle(orbit plane, magnetic equator) = angle(h, m)
    n = np.sqrt(ob.MU / ob.A**3)
    return 2 * n * (1 + sin_xi) * np.linalg.eigvalsh(pr.J)[0], np.degrees(np.arcsin(sin_xi))


def saturate(m, m_max):
    s = np.max(np.abs(m)) / m_max
    return m / s if s > 1 else m


def total_torque(t, beta, m_cmd, env, disturb):
    e = env.at(t)
    BN = at.ep2dcm(beta)
    tq = np.cross(m_cmd, BN @ e["B"])
    if disturb:
        tq = tq + sum(en.disturbance_torques(BN, e).values())
    return tq


# ---------------------------------------------------------------- continuous (ideal)
def simulate_continuous(env, k, m_max, beta0, w0, t_end, disturb=True, dt_out=10.0):
    """x = [beta, omega, E_diss] with E_diss' = omega . (m x B) (magnetorquer power)."""
    def rhs(t, x):
        beta, w = x[:4], x[4:7]
        e = env.at(t)
        BN = at.ep2dcm(beta)
        BB = BN @ e["B"]
        Bd = BN @ e["Bdot"] - np.cross(w, BB)          # d/dt of the body-frame field
        Bn = np.linalg.norm(BB)
        b = BB / Bn
        bdot = (Bd - b * (b @ Bd)) / Bn
        m = -k / Bn * bdot
        if m_max is not None:
            m = saturate(m, m_max)
        tq_c = np.cross(m, BB)
        tq = tq_c + (sum(en.disturbance_torques(BN, e).values()) if disturb else 0)
        bd, wd = at.rigid_body_rhs(beta, w, tq, pr.J, pr.J_INV)
        return np.concatenate([bd, wd, [w @ tq_c]])

    t_eval = np.arange(0, t_end + 1e-9, dt_out)
    sol = solve_ivp(rhs, (0, t_end), np.concatenate([beta0, w0, [0.0]]), method="DOP853",
                    rtol=1e-10, atol=1e-12, t_eval=t_eval)
    return dict(t=sol.t, beta=sol.y[:4].T, w=sol.y[4:7].T, E_diss=sol.y[7], nfev=sol.nfev)


# ---------------------------------------------------------------- flight (sampled)
def rk4_step(f, t, x, h):
    k1 = f(t, x); k2 = f(t + h / 2, x + h / 2 * k1)
    k3 = f(t + h / 2, x + h / 2 * k2); k4 = f(t + h, x + h * k3)
    return x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def simulate_flight(env, k, m_max, beta0, w0, t_end, dt=1.0, duty=0.8, sigma=100e-9,
                    seed=0, h=0.2, disturb=True, dt_out=10.0):
    """1 Hz flight software. Cycle k: sample magnetometer at t_k (coils off since
    t_k - (1-duty) dt), compute m_k, coils on with m_k for duty*dt, off for the rest.
    Truth integrated with fixed-step RK4 (h divides duty*dt and dt)."""
    rng = np.random.default_rng(seed)
    x = np.concatenate([beta0, w0])
    m_cmd = np.zeros(3)

    def f(t, x):
        tq = total_torque(t, x[:4], m_cmd, env, disturb)
        bd, wd = at.rigid_body_rhs(x[:4], x[4:7], tq, pr.J, pr.J_INV)
        return np.concatenate([bd, wd])

    n_cyc = int(round(t_end / dt))
    n_on, n_all = int(round(duty * dt / h)), int(round(dt / h))
    every = int(round(dt_out / dt))
    T, W, BE, M = [], [], [], []
    b_prev = None
    for c in range(n_cyc):
        t = c * dt
        x[:4] /= np.linalg.norm(x[:4])
        BB = at.ep2dcm(x[:4]) @ env.at(t)["B"] + sigma * rng.standard_normal(3)
        Bn = np.linalg.norm(BB)
        b = BB / Bn
        m = np.zeros(3) if b_prev is None else saturate(-k / Bn * (b - b_prev) / dt, m_max)
        b_prev = b
        if c % every == 0:
            T.append(t); W.append(x[4:7].copy()); BE.append(x[:4].copy()); M.append(m.copy())
        for j in range(n_all):
            m_cmd = m if j < n_on else np.zeros(3)
            x = rk4_step(f, t + j * h, x, h)
    return dict(t=np.array(T), w=np.array(W), beta=np.array(BE), m=np.array(M))


def detumble_time(t, w, threshold=np.radians(0.5)):
    """First time after which |omega| stays below the threshold."""
    above = np.where(np.linalg.norm(w, axis=1) >= threshold)[0]
    if len(above) == 0:
        return 0.0
    return t[above[-1] + 1] if above[-1] + 1 < len(t) else np.inf
