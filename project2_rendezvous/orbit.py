"""Relative orbital motion tools for project 2 (rendezvous and docking).

Frames
------
ECI  : Earth-centred inertial frame.
LVLH : local frame attached to the TARGET spacecraft (Hill frame)
       x = radial (away from Earth), y = along-track (direction of motion),
       z = cross-track (along the orbit normal, completes the right-hand set).

Models
------
1. Clohessy-Wiltshire (CW) equations: LINEARIZED relative motion about a
   circular target orbit, valid while the chaser is close (|rho| << orbit radius):

       x'' - 3 n^2 x - 2 n y' = ax
       y''           + 2 n x' = ay
       z''           + n^2 z  = az

   n = sqrt(mu / a^3) is the target's mean motion (orbital angular rate).

2. Full nonlinear two-body motion of both spacecraft in ECI, converted to LVLH.
   Used as the "truth" to check where the CW model is accurate.
"""
import numpy as np

MU = 3.986004418e14            # Earth gravitational parameter [m^3/s^2]
R_EARTH = 6378.137e3           # Earth equatorial radius [m]
ALT = 400e3                    # target altitude (ISS-like) [m]
A_TARGET = R_EARTH + ALT       # target orbit radius (circular) [m]
N = np.sqrt(MU / A_TARGET**3)  # mean motion [rad/s]
PERIOD = 2 * np.pi / N         # orbital period [s]


# ---------------------------------------------------------------------------
# Clohessy-Wiltshire model
# ---------------------------------------------------------------------------
def cw_rhs(t, s, accel=None, n=N):
    """CW equations in state-space form. s = [x, y, z, vx, vy, vz] in LVLH."""
    x, y, z, vx, vy, vz = s
    ax, ay, az = (0.0, 0.0, 0.0) if accel is None else accel(t, s)
    return np.array([vx, vy, vz,
                     3 * n**2 * x + 2 * n * vy + ax,
                     -2 * n * vx + ay,
                     -n**2 * z + az])


def cw_matrices(n=N):
    """Continuous-time state-space matrices: s' = A s + B u (u = acceleration)."""
    A = np.zeros((6, 6))
    A[0:3, 3:6] = np.eye(3)
    A[3, 0], A[3, 4] = 3 * n**2, 2 * n
    A[4, 3] = -2 * n
    A[5, 2] = -n**2
    B = np.vstack([np.zeros((3, 3)), np.eye(3)])
    return A, B


def cw_stm(t, n=N):
    """Closed-form CW state transition matrix Phi(t): s(t) = Phi(t) s(0)."""
    s, c = np.sin(n * t), np.cos(n * t)
    return np.array([
        [4 - 3 * c,           0, 0,  s / n,             2 * (1 - c) / n,        0],
        [6 * (s - n * t),     1, 0, -2 * (1 - c) / n,   (4 * s - 3 * n * t) / n, 0],
        [0,                   0, c,  0,                 0,                      s / n],
        [3 * n * s,           0, 0,  c,                 2 * s,                  0],
        [-6 * n * (1 - c),    0, 0, -2 * s,             4 * c - 3,              0],
        [0,                   0, -n * s, 0,             0,                      c],
    ])


# ---------------------------------------------------------------------------
# Nonlinear two-body model + frame conversions
# ---------------------------------------------------------------------------
def target_state(t):
    """Target on a circular equatorial orbit (ECI position and velocity)."""
    th = N * t
    r = A_TARGET * np.array([np.cos(th), np.sin(th), 0.0])
    v = A_TARGET * N * np.array([-np.sin(th), np.cos(th), 0.0])
    return r, v


def eci_to_lvlh_dcm(r, v):
    """DCM [LN] that maps ECI vector components into LVLH components."""
    x_hat = r / np.linalg.norm(r)
    h = np.cross(r, v)
    z_hat = h / np.linalg.norm(h)
    y_hat = np.cross(z_hat, x_hat)
    return np.vstack([x_hat, y_hat, z_hat])


def lvlh_to_eci(rel, t=0.0):
    """Relative LVLH state of the chaser -> chaser's absolute ECI state."""
    rt, vt = target_state(t)
    C = eci_to_lvlh_dcm(rt, vt)
    omega = np.array([0.0, 0.0, N])                    # LVLH rotation rate (in LVLH)
    rho, rho_dot = rel[:3], rel[3:]
    rc = rt + C.T @ rho
    vc = vt + C.T @ (rho_dot + np.cross(omega, rho))   # transport theorem
    return np.concatenate([rc, vc])


def eci_to_lvlh(chaser, t):
    """Chaser absolute ECI state -> relative LVLH state w.r.t. the target."""
    rt, vt = target_state(t)
    C = eci_to_lvlh_dcm(rt, vt)
    omega = np.array([0.0, 0.0, N])
    rho = C @ (chaser[:3] - rt)
    rho_dot = C @ (chaser[3:] - vt) - np.cross(omega, rho)
    return np.concatenate([rho, rho_dot])


def two_body_rhs(t, s):
    """Point-mass gravity: r'' = -mu r / |r|^3."""
    r, v = s[:3], s[3:]
    return np.concatenate([v, -MU * r / np.linalg.norm(r)**3])
