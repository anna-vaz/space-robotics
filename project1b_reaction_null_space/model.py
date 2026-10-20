"""Symbolic (SymPy) dynamics of an n-link planar free-floating robot.

Same method as project 1 (Lagrange, kinetic energy only), but written for any number
of links so we can build both the 2-link robot of project 1 (to check we reproduce
it) and the redundant 3-link robot used in this extension.

    M(q) qdd + c(q, qd) = [0, 0, 0, tau_1 ... tau_n]

Also provides the pieces needed for the Reaction Null Space:
    * momentum  h = M_b. qd = [P_x, P_y, L]  (first three rows of M qd)
    * end-effector position and its Jacobian [J_b | J_m]
"""
import numpy as np
import sympy as sp


class PlanarFreeFloatingRobot:
    def __init__(self, lengths, masses, inertias, m0, I0, b0):
        n = len(lengths)
        self.n = n
        self.lengths = np.asarray(lengths, float)
        self.masses = np.asarray(masses, float)
        self.inertias = np.asarray(inertias, float)
        self.m0, self.I0, self.b0 = m0, I0, b0

        # ---- symbols ----
        qs = sp.symbols(f"x0 y0 th0 q1:{n + 1}")
        qds = sp.symbols(f"dx0 dy0 dth0 dq1:{n + 1}")
        q, qd = sp.Matrix(qs), sp.Matrix(qds)
        self.q_sym, self.qd_sym = q, qd

        def u(a):
            return sp.Matrix([sp.cos(a), sp.sin(a)])

        R0 = sp.Matrix([[sp.cos(q[2]), -sp.sin(q[2])], [sp.sin(q[2]), sp.cos(q[2])]])

        # ---- positions of every body's CoM, and absolute angles ----
        r0 = sp.Matrix(q[:2])
        joint = r0 + R0 * sp.Matrix(b0)
        phi = q[2]
        bodies = [(m0, I0, r0, q[2])]
        for i in range(n):
            phi = phi + q[3 + i]
            L = sp.nsimplify(self.lengths[i])
            bodies.append((sp.nsimplify(self.masses[i]), sp.nsimplify(self.inertias[i]),
                           joint + L / 2 * u(phi), phi))
            joint = joint + L * u(phi)
        ee = joint

        # ---- mass matrix  M = sum m Jv^T Jv + I Jw^T Jw ----
        N = n + 3
        M = sp.zeros(N, N)
        for m, I, r, ang in bodies:
            Jv = r.jacobian(q)
            Jw = sp.Matrix([ang]).jacobian(q)
            M += m * Jv.T * Jv + I * Jw.T * Jw
        M = M.applyfunc(sp.trigsimp)
        self.M_sym = M

        # ---- Coriolis/centrifugal vector from Christoffel symbols ----
        dM = [M.diff(q[k]) for k in range(N)]
        c = sp.zeros(N, 1)
        for i in range(N):
            for j in range(N):
                for k in range(N):
                    c[i] += (dM[k][i, j] - sp.Rational(1, 2) * dM[i][j, k]) * qd[j] * qd[k]

        # ---- end effector ----
        J_ee = ee.jacobian(q)

        self._M = sp.lambdify([q], M, "numpy", cse=True)
        self._c = sp.lambdify([q, qd], c, "numpy", cse=True)
        self._ee = sp.lambdify([q], ee, "numpy", cse=True)
        self._Jee = sp.lambdify([q], J_ee, "numpy", cse=True)

    # ------------------------------------------------------------------ numeric API
    def mass_matrix(self, q):
        return np.array(self._M(np.asarray(q, float)), dtype=float)

    def coriolis(self, q, qd):
        return np.array(self._c(np.asarray(q, float), np.asarray(qd, float)), dtype=float).ravel()

    def kinetic_energy(self, q, qd):
        qd = np.asarray(qd, float)
        return 0.5 * qd @ self.mass_matrix(q) @ qd

    def momentum(self, q, qd):
        """[P_x, P_y, L_O]: linear momentum and angular momentum about the inertial origin.

        Rows 1-2 of M qd are P.  Row 3 is dT/d(th0'), which is the angular momentum
        about the MOVING base CoM r0 (because rotating th0 swings every body about r0).
        That one is not conserved when P != 0; the conserved quantity is
            L_O = p_th0 + r0 x P.
        (With zero initial momentum, P = 0 and the two coincide.)
        """
        q = np.asarray(q, float)
        px, py, pth = self.mass_matrix(q)[:3] @ np.asarray(qd, float)
        return np.array([px, py, pth + q[0] * py - q[1] * px])

    def end_effector(self, q):
        return np.array(self._ee(np.asarray(q, float)), dtype=float).ravel()

    def ee_jacobians(self, q):
        """J_b (2x3, base coordinates) and J_m (2xn, joints)."""
        J = np.array(self._Jee(np.asarray(q, float)), dtype=float)
        return J[:, :3], J[:, 3:]

    def accelerations(self, q, qd, tau_joints):
        tau = np.concatenate([np.zeros(3), np.asarray(tau_joints, float)])
        return np.linalg.solve(self.mass_matrix(q), tau - self.coriolis(q, qd))


def build(params_module):
    p = params_module
    return PlanarFreeFloatingRobot(p.LENGTHS, p.MASSES, p.INERTIAS, p.m0, p.I0, p.b0)
