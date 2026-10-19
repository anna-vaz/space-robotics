"""Generalized Jacobian of the planar free-floating robot (Umetani & Yoshida, 1989).

For a fixed-base arm the end-effector velocity is  v_e = J_m(q) qd_m.
In space the base moves too:  v_e = J_b qd_b + J_m qd_m,  with qd_b = (x0', y0', th0').
If the system starts at rest, its momentum stays zero, so the base velocity is
fixed by the joint velocities:

    M_bb qd_b + M_bm qd_m = 0      ->      qd_b = -M_bb^-1 M_bm qd_m

(the first three rows of M qd are the momenta conjugate to x0, y0, th0).
Substituting gives the GENERALIZED JACOBIAN

    v_e = J* qd_m,      J* = J_m - J_b M_bb^-1 M_bm

which depends on the masses and inertias, not only on the geometry.
"""
import os
import sys

import numpy as np
import sympy as sp

P1 = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "project1_free_floating")
sys.path.insert(0, P1)
import dynamics as d          # noqa: E402  (project 1: symbolic model, M(q), c(q, qd))
from params import l2         # noqa: E402

# end-effector position (symbolic, same geometry as project 1) and its Jacobian
ee_sym = d.j2 + l2 * sp.Matrix([sp.cos(d.phi2), sp.sin(d.phi2)])
_J_full = sp.lambdify([d.q], ee_sym.jacobian(d.q), "numpy")      # 2 x 5
_ee = sp.lambdify([d.q], ee_sym, "numpy")


def end_effector(q):
    return np.array(_ee(np.asarray(q, float)), dtype=float).ravel()


def jacobians(q):
    """J_b (2x3, base coordinates) and J_m (2x2, joints)."""
    J = np.array(_J_full(np.asarray(q, float)), dtype=float)
    return J[:, :3], J[:, 3:]


def base_coupling(q):
    """qd_b = B(q) qd_m for zero momentum, with B = -M_bb^-1 M_bm."""
    M = d.mass_matrix(q)
    return -np.linalg.solve(M[:3, :3], M[:3, 3:])


def generalized_jacobian(q):
    Jb, Jm = jacobians(q)
    return Jm + Jb @ base_coupling(q)


def fixed_base_jacobian(q):
    return jacobians(q)[1]


def manipulability(J):
    """Yoshikawa manipulability measure sqrt(det(J J^T)) (0 at a singularity)."""
    return float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0)))
