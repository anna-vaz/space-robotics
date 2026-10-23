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
    J*(q) = J_m + J_b B         generalized Jacobian: end-effector velocity per joint rate
"""
import numpy as np


def _perp(v):
    return np.array([-v[1], v[0]])                # z x v  (planar cross product)


class PlanarRobot:
    def __init__(self, lengths, masses, m0=100.0, a0=1.0, b0=(0.5, 0.0)):
        self.l = np.asarray(lengths, float)
        self.m = np.asarray(masses, float)
        self.I = self.m * self.l**2 / 12            # slender rods about their centre
        self.m0, self.I0 = m0, m0 * (2 * a0**2) / 12  # square plate
        self.b0 = np.asarray(b0, float)
        self.n = len(self.l)

    # ------------------------------------------------------------------ geometry
    def _chain(self, q):
        """Joint positions, link CoMs, absolute angles and the end effector."""
        th0, qm = q[2], np.asarray(q[3:], float)
        r0 = np.asarray(q[:2], float)
        c, s = np.cos(th0), np.sin(th0)
        joint = r0 + np.array([c * self.b0[0] - s * self.b0[1], s * self.b0[0] + c * self.b0[1]])
        joints, coms, phis = [], [], []
        phi = th0
        for i in range(self.n):
            phi += qm[i]
            u = np.array([np.cos(phi), np.sin(phi)])
            joints.append(joint)
            coms.append(joint + 0.5 * self.l[i] * u)
            phis.append(phi)
            joint = joint + self.l[i] * u
        return r0, joints, coms, phis, joint            # last 'joint' = end effector

    def _point_jacobian(self, q, p, joints, upto):
        """d p / d q for a point p fixed to link 'upto' (the base if upto < 0)."""
        Jv = np.zeros((2, self.n + 3))
        Jv[:, 0:2] = np.eye(2)
        Jv[:, 2] = _perp(p - np.asarray(q[:2], float))
        for j in range(upto + 1):
            Jv[:, 3 + j] = _perp(p - joints[j])
        return Jv

    def bodies(self, q):
        """[(mass, inertia, CoM position, Jv (2xN), Jw (N))] for base + links."""
        N = self.n + 3
        r0, joints, coms, _, _ = self._chain(q)
        Jw0 = np.zeros(N)
        Jw0[2] = 1.0
        out = [(self.m0, self.I0, r0, self._point_jacobian(q, r0, joints, -1), Jw0)]
        for i in range(self.n):
            Jw = np.zeros(N)
            Jw[2] = 1.0
            Jw[3:3 + i + 1] = 1.0
            out.append((self.m[i], self.I[i], coms[i], self._point_jacobian(q, coms[i], joints, i), Jw))
        return out

    def mass_matrix(self, q):
        N = self.n + 3
        M = np.zeros((N, N))
        for m, I, _, Jv, Jw in self.bodies(q):
            M += m * Jv.T @ Jv + I * np.outer(Jw, Jw)
        return M

    def end_effector(self, q):
        return self._chain(q)[4]

    def ee_jacobian(self, q):
        """Full 2 x N end-effector Jacobian (base columns first)."""
        _, joints, _, _, ee = self._chain(q)
        return self._point_jacobian(q, ee, joints, self.n - 1)

    def momentum(self, q, qd):
        """Linear momentum (Px, Py) and angular momentum about the inertial origin L_O."""
        P, L = np.zeros(2), 0.0
        for m, I, r, Jv, Jw in self.bodies(q):
            v = Jv @ qd
            P += m * v
            L += m * (r[0] * v[1] - r[1] * v[0]) + I * (Jw @ qd)
        return P, L

    def mass_points(self, q):
        """[(mass, inertia about own CoM, CoM position)] -- for system inertia sums."""
        return [(m, I, r) for m, I, r, _, _ in self.bodies(q)]

    # ------------------------------------------------------------------ RNS
    def base_coupling(self, q):
        M = self.mass_matrix(q)
        return -np.linalg.solve(M[:3, :3], M[:3, 3:])

    def reaction_row(self, q):
        return self.base_coupling(q)[2]

    def rns_projector(self, q):
        b = self.reaction_row(q)
        return np.eye(self.n) - np.outer(b, b) / (b @ b)

    def generalized_jacobian(self, q):
        J = self.ee_jacobian(q)
        return J[:, 3:] + J[:, :3] @ self.base_coupling(q)


# The robots of the portfolio
P1_ARM = dict(lengths=[1.0, 1.0], masses=[10.0, 10.0])
P1B_ARM = dict(lengths=[1.0, 1.0, 0.5], masses=[10.0, 10.0, 5.0])
