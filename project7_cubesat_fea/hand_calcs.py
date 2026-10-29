"""Hand-calculation tools for Project 7 (independent of SOLIDWORKS).

- angle_section / box_bending_I : cross-section properties
- timoshenko_beam_modes         : small Timoshenko beam FE (bending modes)
- plate_modes_point_supported   : Rayleigh-Ritz thin plate on point supports
- gevs_asd / grms / miles_grms  : random-vibration spectrum tools
"""
import numpy as np
from numpy.polynomial import legendre as L
from scipy.linalg import eigh, null_space


# --------------------------------------------------------------------------
# Cross sections
# --------------------------------------------------------------------------
def angle_section(w, t):
    """Equal-leg angle, legs w, wall t, corner at the origin, legs along +x, +y.

    Returns dict: A, centroid (cx, cy), Ix, Iy, Ixy about the centroid,
    principal I_min / I_max.
    """
    # two rectangles: (x0, y0, bx, by)
    rects = [(0.0, 0.0, w, t), (0.0, t, t, w - t)]
    A = sum(bx * by for _, _, bx, by in rects)
    cx = sum(bx * by * (x0 + bx / 2) for x0, _, bx, by in rects) / A
    cy = sum(bx * by * (y0 + by / 2) for _, y0, bx, by in rects) / A
    Ix = Iy = Ixy = 0.0
    for x0, y0, bx, by in rects:
        a = bx * by
        dx, dy = x0 + bx / 2 - cx, y0 + by / 2 - cy
        Ix += bx * by**3 / 12 + a * dy**2
        Iy += by * bx**3 / 12 + a * dx**2
        Ixy += a * dx * dy
    Iavg, R = (Ix + Iy) / 2, np.hypot((Ix - Iy) / 2, Ixy)
    return dict(A=A, cx=cx, cy=cy, Ix=Ix, Iy=Iy, Ixy=Ixy, I_min=Iavg - R, I_max=Iavg + R)


def box_bending_I(side, rail_w, rail_t, wall_t):
    """Bending I (about a central axis parallel to one face) of the 3U section:
    4 corner angle rails (outer faces flush with the envelope) + 4 flat walls of
    thickness wall_t spanning between the rails, flush with the outside.

    Returns dict with I_rails, I_walls, I_total, A_rails, A_walls, A_shear
    (shear area = the two walls parallel to the load, i.e. the webs).
    """
    s = angle_section(rail_w, rail_t)
    d = side / 2 - s["cy"]                 # rail centroid distance from the axis
    I_rails = 4 * (s["Ix"] + s["A"] * d**2)
    bw = side - 2 * rail_w                 # free wall width between rails
    yf = side / 2 - wall_t / 2             # flange mid-plane
    I_flanges = 2 * (bw * wall_t**3 / 12 + bw * wall_t * yf**2)
    I_webs = 2 * (wall_t * bw**3 / 12)
    A_walls = 4 * bw * wall_t
    return dict(I_rails=I_rails, I_walls=I_flanges + I_webs, I_total=I_rails + I_flanges + I_webs,
                A_rails=4 * s["A"], A_walls=A_walls, A_shear=2 * bw * wall_t, rail=s)


# --------------------------------------------------------------------------
# Timoshenko beam finite elements (Friedman & Kosmatka 1993 stiffness,
# consistent translational mass + rotary inertia)
# --------------------------------------------------------------------------
def timoshenko_beam_modes(length, EI, m_per_len, kGA=np.inf, rhoI=0.0, n_el=60,
                          bc="cantilever", n_modes=4):
    """Bending natural frequencies (Hz) of a uniform beam.

    kGA = inf -> Euler-Bernoulli; rhoI = rotary inertia per length (kg m).
    bc: 'cantilever' (fixed at x=0) or 'free'.
    """
    h = length / n_el
    phi = 0.0 if np.isinf(kGA) else 12 * EI / (kGA * h**2)
    k = EI / (h**3 * (1 + phi)) * np.array([
        [12, 6 * h, -12, 6 * h],
        [6 * h, (4 + phi) * h**2, -6 * h, (2 - phi) * h**2],
        [-12, -6 * h, 12, -6 * h],
        [6 * h, (2 - phi) * h**2, -6 * h, (4 + phi) * h**2]])
    mt = m_per_len * h / 420 * np.array([
        [156, 22 * h, 54, -13 * h],
        [22 * h, 4 * h**2, 13 * h, -3 * h**2],
        [54, 13 * h, 156, -22 * h],
        [-13 * h, -3 * h**2, -22 * h, 4 * h**2]])
    mr = rhoI / (30 * h) * np.array([
        [36, 3 * h, -36, 3 * h],
        [3 * h, 4 * h**2, -3 * h, -h**2],
        [-36, -3 * h, 36, -3 * h],
        [3 * h, -h**2, -3 * h, 4 * h**2]])
    n = 2 * (n_el + 1)
    K, M = np.zeros((n, n)), np.zeros((n, n))
    for e in range(n_el):
        i = slice(2 * e, 2 * e + 4)
        K[i, i] += k
        M[i, i] += mt + mr
    keep = np.arange(2, n) if bc == "cantilever" else np.arange(n)
    w2 = eigh(K[np.ix_(keep, keep)], M[np.ix_(keep, keep)], eigvals_only=True)
    w2 = w2[w2 > 1e-6 * w2.max()] if bc == "free" else w2
    return np.sqrt(np.abs(w2[:n_modes])) / (2 * np.pi)


# --------------------------------------------------------------------------
# Thin rectangular plate on point supports, Rayleigh-Ritz with Legendre
# polynomials P_i(xi) P_j(eta) (Kirchhoff plate, isotropic).
# --------------------------------------------------------------------------
def plate_modes_point_supported(a, b, h, E, nu, rho_h, supports, n_poly=14, n_modes=4,
                                return_shape=False):
    """Natural frequencies (Hz) of an a x b plate (centre at origin) with
    transverse point supports w(x_k, y_k) = 0 at `supports` (list of (x, y)).
    rho_h = mass per unit area (kg/m^2), smeared (board + components).
    supports = [] -> free plate (rigid-body modes removed).
    """
    D = E * h**3 / (12 * (1 - nu**2))
    nq = n_poly + 4
    xg, wg = L.leggauss(nq)                 # exact for the polynomial integrands

    def basis(n, s):                        # values, d/ds, d2/ds2 of P_0..P_{n-1}
        V = np.zeros((3, n, s.size))
        for i in range(n):
            c = np.zeros(i + 1); c[i] = 1
            V[0, i] = L.legval(s, c)
            V[1, i] = L.legval(s, L.legder(c, 1))
            V[2, i] = L.legval(s, L.legder(c, 2))
        return V

    Bx, By = basis(n_poly, xg), basis(n_poly, xg)
    sx, sy = 2 / a, 2 / b                   # d xi/dx, d eta/dy
    jac = (a / 2) * (b / 2)

    def gram(Ui, Uj):                       # 1D integrals int Ui Uj dxi
        return (Ui * wg) @ Uj.T

    # 1D matrices: I_pq = int P^(p) P^(q)
    Ix = {(p, q): gram(Bx[p], Bx[q]) for p in range(3) for q in range(3)}
    Iy = {(p, q): gram(By[p], By[q]) for p in range(3) for q in range(3)}
    kron = np.kron
    K = D * jac * (
        sx**4 * kron(Ix[2, 2], Iy[0, 0]) + sy**4 * kron(Ix[0, 0], Iy[2, 2])
        + nu * sx**2 * sy**2 * (kron(Ix[2, 0], Iy[0, 2]) + kron(Ix[0, 2], Iy[2, 0]))
        + 2 * (1 - nu) * sx**2 * sy**2 * kron(Ix[1, 1], Iy[1, 1]))
    M = rho_h * jac * kron(Ix[0, 0], Iy[0, 0])

    if supports:
        C = []
        for (xs, ys) in supports:
            px = basis(n_poly, np.array([2 * xs / a]))[0, :, 0]
            py = basis(n_poly, np.array([2 * ys / b]))[0, :, 0]
            C.append(np.kron(px, py))
        Z = null_space(np.array(C))
    else:
        Z = np.eye(K.shape[0])
    w2, vec = eigh(Z.T @ K @ Z, Z.T @ M @ Z)
    if not supports:
        keep = w2 > 1e-8 * w2.max()
        w2, vec = w2[keep], vec[:, keep]
    f = np.sqrt(np.abs(w2[:n_modes])) / (2 * np.pi)
    if not return_shape:
        return f
    c = Z @ vec[:, 0]
    def shape(x, y):
        px = basis(n_poly, 2 * np.ravel(x) / a)[0]
        py = basis(n_poly, 2 * np.ravel(y) / b)[0]
        return np.einsum("ij,ik,jk->k", c.reshape(n_poly, n_poly), px, py).reshape(np.shape(x))
    return f, shape


def plate_lambda(f_hz, a, D, rho_h):
    """Non-dimensional frequency  lambda = omega a^2 sqrt(rho_h / D)."""
    return 2 * np.pi * f_hz * a**2 * np.sqrt(rho_h / D)


# --------------------------------------------------------------------------
# Random vibration
# --------------------------------------------------------------------------
def gevs_asd(f, bps):
    """ASD (g^2/Hz) at f, log-log interpolation between breakpoints; 0 outside."""
    f = np.atleast_1d(np.asarray(f, float))
    fb, ab = np.array(bps).T
    out = np.exp(np.interp(np.log(f), np.log(fb), np.log(ab)))
    out[(f < fb[0]) | (f > fb[-1])] = 0.0
    return out


def grms(bps):
    """Exact overall g rms of a log-log piecewise-linear ASD profile."""
    total = 0.0
    for (f1, a1), (f2, a2) in zip(bps[:-1], bps[1:]):
        n = np.log(a2 / a1) / np.log(f2 / f1)          # slope in log-log
        if abs(n + 1) < 1e-12:
            total += a1 * f1 * np.log(f2 / f1)
        else:
            total += a1 * f1 / (n + 1) * ((f2 / f1)**(n + 1) - 1)
    return np.sqrt(total)


def miles_grms(fn, Q, asd_at_fn):
    """Miles' equation: rms response acceleration (g) of a single-DOF system
    with natural frequency fn and amplification Q on a flat input ASD."""
    return np.sqrt(np.pi / 2 * fn * Q * asd_at_fn)
