"""Space environment along the orbit + disturbance torques.

  * Earth magnetic field: IGRF-14 (ppigrf, degree 13) and a tilted dipole for comparison
  * Sun direction (low-precision almanac, ~0.01 deg) and cylindrical Earth shadow
  * Atmospheric density: exponential model (Vallado, table 8-4)
  * Torques: gravity gradient, residual magnetic dipole, aerodynamic drag, solar pressure

The orbit and the environment do not depend on the attitude, so OrbitEnvironment
evaluates them once on a time grid and interpolates (cubic splines) inside the
attitude integrator -- one IGRF call costs ~20 ms, a spline lookup ~20 us.
"""
from datetime import timedelta
import numpy as np
import ppigrf
from ppigrf.ppigrf import read_shc
from scipy.interpolate import CubicSpline

import orbit as ob
import params as pr

AU = 1.495978707e11
P_SUN = 4.56e-6            # solar radiation pressure at 1 AU [N/m^2]
R_IGRF = 6371.2e3          # IGRF reference radius [m] (not the equatorial radius!)


# ---- Time ---------------------------------------------------------------------
def julian_date(dt):
    return 2440587.5 + dt.timestamp() / 86400.0


def gmst(jd):
    """Greenwich mean sidereal time [rad] (IAU-82, Vallado eq. 3-47)."""
    T = (jd - 2451545.0) / 36525.0
    sec = (67310.54841 + (876600 * 3600 + 8640184.812866) * T
           + 0.093104 * T**2 - 6.2e-6 * T**3)
    return np.radians((sec % 86400) / 240.0)


def R3(angle):
    """Frame rotation about z: v_new = R3 v_old (rotates the frame by +angle)."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])


# ---- Sun ----------------------------------------------------------------------
def sun_eci(jd):
    """Sun position [m] in ECI (Vallado algorithm 29). Works on arrays of jd."""
    jd = np.asarray(jd, float)
    T = (jd - 2451545.0) / 36525.0
    lam_m = np.radians(280.460 + 36000.771 * T)
    M = np.radians(357.5291092 + 35999.05034 * T)
    lam = lam_m + np.radians(1.914666471 * np.sin(M) + 0.019994643 * np.sin(2 * M))
    r = 1.000140612 - 0.016708617 * np.cos(M) - 0.000139589 * np.cos(2 * M)
    eps = np.radians(23.439291 - 0.0130042 * T)
    return AU * r[..., None] * np.stack([np.cos(lam), np.cos(eps) * np.sin(lam), np.sin(eps) * np.sin(lam)], axis=-1)


def in_shadow(r, s_hat):
    """Cylindrical shadow: behind the Earth and inside its radius. r: (...,3)."""
    along = np.sum(r * s_hat, axis=-1)
    perp = np.linalg.norm(r - along[..., None] * s_hat, axis=-1)
    return (along < 0) & (perp < ob.RE)


def eclipse_fraction(a, beta):
    """Fraction of a circular orbit spent in the cylindrical shadow (beta = Sun angle
    above the orbit plane)."""
    h = a - ob.RE
    if abs(beta) >= np.arcsin(ob.RE / a):
        return 0.0
    return np.arccos(np.sqrt(h**2 + 2 * ob.RE * h) / (a * np.cos(beta))) / np.pi


# ---- Magnetic field -------------------------------------------------------------
def igrf_eci(r_eci, jd, date, max_degree=13):
    """IGRF field [T] in ECI at positions r_eci (N,3) [m], times jd (N,)."""
    r_eci = np.atleast_2d(r_eci)
    th_g = gmst(np.atleast_1d(jd))
    c, s = np.cos(th_g), np.sin(th_g)
    x = c * r_eci[:, 0] + s * r_eci[:, 1]              # ECEF = R3(gmst) ECI
    y = -s * r_eci[:, 0] + c * r_eci[:, 1]
    z = r_eci[:, 2]
    rn = np.sqrt(x**2 + y**2 + z**2)
    theta = np.arccos(z / rn)                          # colatitude
    phi = np.arctan2(y, x)
    Br, Bt, Bp = ppigrf.igrf_gc(rn / 1e3, np.degrees(theta), np.degrees(phi), date.replace(tzinfo=None),
                                max_degree=max_degree)
    Br, Bt, Bp = (np.ravel(b) * 1e-9 for b in (Br, Bt, Bp))
    st, ct, sp, cp = np.sin(theta), np.cos(theta), np.sin(phi), np.cos(phi)
    Bx = Br * st * cp + Bt * ct * cp - Bp * sp         # spherical -> ECEF
    By = Br * st * sp + Bt * ct * sp + Bp * cp
    Bz = Br * ct - Bt * st
    return np.stack([c * Bx - s * By, s * Bx + c * By, Bz], axis=1)   # ECEF -> ECI


def dipole_coefficients(date):
    """g10, g11, h11 [nT] of IGRF-14 at `date` (linear in time, as IGRF defines)."""
    g, h = read_shc()
    t = g.index.values.astype("datetime64[s]").astype(float)
    tq = np.datetime64(date.replace(tzinfo=None), "s").astype(float)
    return tuple(np.interp(tq, t, df[(1, m)].values.astype(float)) for df, m in ((g, 0), (g, 1), (h, 1)))


def dipole_eci(r_eci, jd, date):
    """Tilted-dipole field [T] in ECI written directly from the vector formula
        B = (R_IGRF/r)^3 B0 [3 (m.r_hat) r_hat - m],  m = (g11, h11, g10)/B0 (ECEF)
    (potential V = R_IGRF^3 (g.r)/r^3, B = -grad V; g10 < 0, so m points south)
    -- independent of ppigrf's Legendre machinery."""
    g10, g11, h11 = dipole_coefficients(date)
    B0 = np.sqrt(g10**2 + g11**2 + h11**2) * 1e-9
    m_ecef = np.array([g11, h11, g10]) / np.sqrt(g10**2 + g11**2 + h11**2)
    r_eci = np.atleast_2d(r_eci)
    out = np.empty_like(r_eci)
    for k, (r, th) in enumerate(zip(r_eci, gmst(np.atleast_1d(jd)))):
        m = R3(th).T @ m_ecef                          # dipole axis in ECI
        rn = np.linalg.norm(r)
        rh = r / rn
        out[k] = (R_IGRF / rn)**3 * B0 * (3 * (m @ rh) * rh - m)
    return out


# ---- Atmosphere -----------------------------------------------------------------
_H0 = np.array([400, 450, 500, 600, 700]) * 1e3              # base altitudes [m]
_RHO0 = np.array([3.725e-12, 1.585e-12, 6.967e-13, 1.454e-13, 3.614e-14])
_HS = np.array([58.515, 60.828, 63.822, 71.835, 88.667]) * 1e3


def density(h):
    i = np.clip(np.searchsorted(_H0, h, side="right") - 1, 0, len(_H0) - 1)
    return _RHO0[i] * np.exp(-(h - _H0[i]) / _HS[i])


# ---- Pre-computed orbit environment ---------------------------------------------
class OrbitEnvironment:
    def __init__(self, t_end, dt=10.0, raan=None, epoch=ob.EPOCH):
        self.epoch = epoch
        self.jd0 = julian_date(epoch)
        if raan is None:   # place the ascending node at the chosen local time
            s = sun_eci(self.jd0)
            raan = np.arctan2(s[1], s[0]) + np.radians((ob.LTAN_HOURS - 12) * 15)
        self.s0 = ob.initial_state(raan)
        self.sol = ob.propagate(self.s0, t_end + 2 * dt)
        self.t = np.arange(0, t_end + 2 * dt, dt)
        X = self.sol.sol(self.t).T
        self.r, self.v = X[:, :3], X[:, 3:]
        self.jd = self.jd0 + self.t / 86400
        mid = epoch + timedelta(seconds=t_end / 2)     # secular variation over days: ~0.01 nT/h
        self.B = igrf_eci(self.r, self.jd, mid)
        self.sun = sun_eci(self.jd)
        self.shadow = in_shadow(self.r, self.sun / np.linalg.norm(self.sun, axis=1)[:, None])
        self._B = CubicSpline(self.t, self.B)
        self._sun = CubicSpline(self.t, self.sun)
        self.field_date = mid

    def at(self, t):
        """Environment at time t (scalar): r, v, B [T], sun vector [m], shadow flag."""
        rv = self.sol.sol(t)
        sun = self._sun(t)
        r = rv[:3]
        return dict(r=r, v=rv[3:], B=self._B(t), sun=sun,
                    shadow=bool(in_shadow(r, sun / np.linalg.norm(sun))))


# ---- Disturbance torques (all inputs/outputs in BODY axes) ------------------------
def gravity_gradient_torque(r_B, J=pr.J):
    rn = np.linalg.norm(r_B)
    return 3 * ob.MU / rn**5 * np.cross(r_B, J @ r_B)


def magnetic_torque(m_B, B_B):
    return np.cross(m_B, B_B)


def drag_torque(v_rel_B, rho):
    """Flat-plate free-molecular drag on every face that sees the flow."""
    V = np.linalg.norm(v_rel_B)
    vh = v_rel_B / V
    cosang = pr.FACE_NORMALS @ vh
    lit = cosang > 0
    F = -0.5 * rho * pr.CD * V**2 * (pr.FACE_AREAS[lit] * cosang[lit])[:, None] * vh
    return np.sum(np.cross(pr.FACE_CENTRES[lit], F), axis=0)


def srp_torque(s_B, dist, shadow):
    """Solar pressure on every sunlit face; s_B = unit vector to the Sun."""
    if shadow:
        return np.zeros(3)
    P = P_SUN * (AU / dist)**2
    cosang = pr.FACE_NORMALS @ s_B
    lit = cosang > 0
    F = -P * pr.CR * (pr.FACE_AREAS[lit] * cosang[lit])[:, None] * s_B
    return np.sum(np.cross(pr.FACE_CENTRES[lit], F), axis=0)


def disturbance_torques(BN, env, m_res=pr.M_RESIDUAL):
    """All four disturbance torques [N m] in body axes for attitude [BN]."""
    r, v = env["r"], env["v"]
    v_rel = v - np.cross([0, 0, ob.OMEGA_E], r)        # co-rotating atmosphere
    rho = density(np.linalg.norm(r) - ob.RE)
    s = env["sun"]
    d = np.linalg.norm(s)
    return dict(gg=gravity_gradient_torque(BN @ r),
                mag=magnetic_torque(m_res, BN @ env["B"]),
                drag=drag_torque(BN @ v_rel, rho),
                srp=srp_torque(BN @ (s / d), d, env["shadow"]))
