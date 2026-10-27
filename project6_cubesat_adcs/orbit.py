"""Orbit of the CubeSat: two-body + J2, sun-synchronous 500 km circular orbit.

ECI here is the mean equator/equinox of date (the Sun model and GMST below use
the same frame); precession over a few days is negligible for ADCS work.
"""
import numpy as np
from datetime import datetime, timezone
from scipy.integrate import solve_ivp

MU = 3.986004418e14       # [m^3/s^2]
RE = 6378.137e3           # Earth equatorial radius [m]
J2 = 1.08262668e-3
OMEGA_E = 7.292115e-5     # Earth rotation rate [rad/s]
YEAR = 365.2422 * 86400   # tropical year [s]

EPOCH = datetime(2026, 11, 1, 0, 0, 0, tzinfo=timezone.utc)
ALT = 500e3
A = RE + ALT
N_MEAN = np.sqrt(MU / A**3)
PERIOD = 2 * np.pi / N_MEAN
LTAN_HOURS = 22.5         # ascending node at 22:30 local time = 10:30 descending node


def sso_inclination(a, e=0.0):
    """Inclination that makes the J2 nodal drift equal to the Sun's mean motion.
    RAAN' = -3/2 n J2 (RE/p)^2 cos i  =  +360 deg / year."""
    n = np.sqrt(MU / a**3)
    p = a * (1 - e**2)
    cos_i = -(2 * np.pi / YEAR) / (1.5 * n * J2 * (RE / p)**2)
    return np.arccos(cos_i)


def raan_rate(a, i, e=0.0):
    n = np.sqrt(MU / a**3)
    p = a * (1 - e**2)
    return -1.5 * n * J2 * (RE / p)**2 * np.cos(i)


def kepler_to_rv(a, e, i, raan, argp, nu):
    p = a * (1 - e**2)
    r_pf = p / (1 + e * np.cos(nu)) * np.array([np.cos(nu), np.sin(nu), 0.0])
    v_pf = np.sqrt(MU / p) * np.array([-np.sin(nu), e + np.cos(nu), 0.0])

    def R3(x):
        return np.array([[np.cos(x), -np.sin(x), 0], [np.sin(x), np.cos(x), 0], [0, 0, 1]])

    def R1(x):
        return np.array([[1, 0, 0], [0, np.cos(x), -np.sin(x)], [0, np.sin(x), np.cos(x)]])

    Q = R3(raan) @ R1(i) @ R3(argp)
    return Q @ r_pf, Q @ v_pf


def raan_of(r, v):
    h = np.cross(r, v)
    return np.arctan2(h[..., 0], -h[..., 1])


def j2_rhs(t, s):
    r, v = s[:3], s[3:]
    rn = np.linalg.norm(r)
    z2 = (r[2] / rn)**2
    k = 1.5 * J2 * MU * RE**2 / rn**5
    a_j2 = k * np.array([r[0] * (5 * z2 - 1), r[1] * (5 * z2 - 1), r[2] * (5 * z2 - 3)])
    return np.concatenate([v, -MU * r / rn**3 + a_j2])


def propagate(s0, t_end, t_eval=None):
    """Two-body + J2 truth orbit (DOP853, tight tolerances). Returns the OdeResult
    with dense output so any time can be sampled."""
    return solve_ivp(j2_rhs, (0, t_end), s0, method="DOP853", rtol=1e-11, atol=1e-6,
                     t_eval=t_eval, dense_output=True)


def initial_state(raan):
    """Circular SSO at 500 km, starting at the ascending node."""
    r, v = kepler_to_rv(A, 0.0, sso_inclination(A), raan, 0.0, 0.0)
    return np.concatenate([r, v])
