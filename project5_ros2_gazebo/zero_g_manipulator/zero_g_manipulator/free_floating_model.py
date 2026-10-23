"""Planar free-floating robot model, NumPy only (used inside the ROS nodes).

Same robot description as project1b (base + n slender links in the x-y plane),
but written directly with geometric Jacobians instead of SymPy, so it is fast and
has no dependencies.

    q = [x0, y0, th0, q1 .. qn]          (base CoM position, base angle, joint angles)
    M(q) = sum_bodies  m Jv^T Jv + I Jw^T Jw

Reaction Null Space quantities (zero total momentum):
    B(q) = -M_bb^-1 M_bm        base rates per unit joint rate (3 x n)
    b(q) = B[2]                 base ANGULAR rate per unit joint rate (1 x n)
    P(q) = I - b^T b / (b b^T)  projector onto null(b): joint rates that do not turn the base
"""
import numpy as np


class PlanarRobot:
    def __init__(self, lengths, masses, m0=100.0, a0=1.0, b0=(0.5, 0.0)):
        self.l = np.asarray(lengths, float)
        self.m = np.asarray(masses, float)
        self.I = self.m * self.l**2 / 12            # slender rods about their centre
        self.m0, self.I0 = m0, m0 * (2 * a0**2) / 12  # square plate
        self.b0 = np.asarray(b0, float)
        self.n = len(self.l)

    @staticmethod
    def _perp(v):
        return np.array([-v[1], v[0]])            # z x v  (planar cross product)

    def mass_matrix(self, q):
        n, N = self.n, self.n + 3
        th0, qm = q[2], np.asarray(q[3:], float)
        r0 = np.asarray(q[:2], float)
        c, s = np.cos(th0), np.sin(th0)
        joint = r0 + np.array([c * self.b0[0] - s * self.b0[1], s * self.b0[0] + c * self.b0[1]])

        M = np.zeros((N, N))
        M[0, 0] = M[1, 1] = self.m0                 # base: translation + rotation
        M[2, 2] = self.I0
        phi = th0
        joints = []                                  # positions of joints 1..i
        for i in range(n):
            phi += qm[i]
            joints.append(joint)
            u = np.array([np.cos(phi), np.sin(phi)])
            com = joint + 0.5 * self.l[i] * u
            Jv = np.zeros((2, N))
            Jv[:, 0:2] = np.eye(2)
            Jv[:, 2] = self._perp(com - r0)
            for j in range(i + 1):
                Jv[:, 3 + j] = self._perp(com - joints[j])
            Jw = np.zeros(N)
            Jw[2] = 1.0
            Jw[3:3 + i + 1] = 1.0
            M += self.m[i] * Jv.T @ Jv + self.I[i] * np.outer(Jw, Jw)
            joint = joint + self.l[i] * u
        return M

    def base_coupling(self, q):
        M = self.mass_matrix(q)
        return -np.linalg.solve(M[:3, :3], M[:3, 3:])

    def reaction_row(self, q):
        return self.base_coupling(q)[2]

    def rns_projector(self, q):
        b = self.reaction_row(q)
        return np.eye(self.n) - np.outer(b, b) / (b @ b)


# The robots of the portfolio
P1_ARM = dict(lengths=[1.0, 1.0], masses=[10.0, 10.0])
P1B_ARM = dict(lengths=[1.0, 1.0, 0.5], masses=[10.0, 10.0, 5.0])
