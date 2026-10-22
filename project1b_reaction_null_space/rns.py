"""Reaction Null Space (Nenchev & Yoshida) of the planar free-floating robot.

If the robot starts at rest its momentum stays zero (no external forces):

    M_bb qd_b + M_bm qd_m = 0      ->      qd_b = B(q) qd_m,   B = -M_bb^-1 M_bm   (3 x n)

The third row of B gives the base angular rate caused by the joint rates:

    w0 = b(q) qd_m,      b = B[2, :]   (1 x n)

Any joint rate in the null space of b leaves the base attitude unchanged:
    REACTION NULL SPACE = null(b)   (dimension n - 1 = 2 for the 3-link arm).

The base CoM still drifts (the system CoM must stay put), but it does not rotate --
for a satellite, attitude is what matters (antennas, solar panels, cameras).

End-effector velocity, with the base motion included (generalized Jacobian):
    v_e = J* qd_m,       J* = J_m + J_b B

Stacking task and reaction:   [v_e; w0] = A(q) qd_m,   A = [J*; b]  (3 x 3)
If A is invertible, ANY end-effector velocity can be produced with w0 = 0 exactly:
    qd_m = A^-1 [v_e; 0].
Where det A = 0 the end effector can no longer move in every direction without
disturbing the base: an "RNS singularity" (it is a property of the masses too, not
only of the geometry, like the dynamic singularities of project 3).
"""
import numpy as np


def base_coupling(rob, q):
    """B(q): base rates (x0', y0', th0') per unit joint rate, at zero momentum."""
    M = rob.mass_matrix(q)
    return -np.linalg.solve(M[:3, :3], M[:3, 3:])


def reaction_row(rob, q):
    """b(q): base angular rate per unit joint rate."""
    return base_coupling(rob, q)[2]


def rns_projector(rob, q):
    """Orthogonal projector onto null(b):  P = I - b^T b / (b b^T)."""
    b = reaction_row(rob, q)
    return np.eye(rob.n) - np.outer(b, b) / (b @ b)


def generalized_jacobian(rob, q):
    Jb, Jm = rob.ee_jacobians(q)
    return Jm + Jb @ base_coupling(rob, q)


def extended_jacobian(rob, q):
    """A = [J*; b]: joint rates -> (end-effector velocity, base angular rate)."""
    return np.vstack([generalized_jacobian(rob, q), reaction_row(rob, q)])


def full_rates(rob, q, qd_m):
    """Complete qd = [qd_b, qd_m] for zero momentum."""
    return np.concatenate([base_coupling(rob, q) @ qd_m, qd_m])
