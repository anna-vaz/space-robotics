"""Equations of motion of the planar free-floating robot (Lagrange method).

We derive, symbolically with SymPy:

    M(q) * qdd + c(q, qd) = tau

    q    = [x0, y0, th0, q1, q2]      generalized coordinates
    M(q) = 5x5 mass (inertia) matrix
    c    = Coriolis / centrifugal vector
    tau  = generalized forces = [0, 0, 0, tau1, tau2]
           (the base has no thrusters -> free-floating, only the joints have motors)

There is no gravity, so the Lagrangian is just the kinetic energy T.
The symbolic results are turned into fast NumPy functions with sympy.lambdify.
"""
import numpy as np
import sympy as sp

from params import b0, l1, l2, m0, m1, m2, I0, I1, I2

# ---------------------------------------------------------------------------
# 1. Symbols
# ---------------------------------------------------------------------------
x0, y0, th0, q1, q2 = sp.symbols("x0 y0 th0 q1 q2")
dx0, dy0, dth0, dq1, dq2 = sp.symbols("dx0 dy0 dth0 dq1 dq2")
q = sp.Matrix([x0, y0, th0, q1, q2])
qd = sp.Matrix([dx0, dy0, dth0, dq1, dq2])


def _u(angle):
    return sp.Matrix([sp.cos(angle), sp.sin(angle)])


def _R(theta):
    return sp.Matrix([[sp.cos(theta), -sp.sin(theta)], [sp.sin(theta), sp.cos(theta)]])


# ---------------------------------------------------------------------------
# 2. Positions of each body's centre of mass (same geometry as kinematics.py)
# ---------------------------------------------------------------------------
phi1 = th0 + q1
phi2 = phi1 + q2
r0 = sp.Matrix([x0, y0])
j1 = r0 + _R(th0) * sp.Matrix(b0)
c1 = j1 + sp.Rational(1, 2) * l1 * _u(phi1)
j2 = j1 + l1 * _u(phi1)
c2 = j2 + sp.Rational(1, 2) * l2 * _u(phi2)

# ---------------------------------------------------------------------------
# 3. Kinetic energy:  T = sum( 1/2 m v.v + 1/2 I w^2 )
#    Velocities via Jacobians: v = (d r / d q) * qd
# ---------------------------------------------------------------------------
bodies = [  # (mass, inertia, CoM position, absolute angle)
    (m0, I0, r0, th0),
    (m1, I1, c1, phi1),
    (m2, I2, c2, phi2),
]

M = sp.zeros(5, 5)
for m, I, r, ang in bodies:
    Jv = r.jacobian(q)                       # 2x5 linear-velocity Jacobian
    Jw = sp.Matrix([ang]).jacobian(q)        # 1x5 angular-velocity Jacobian
    M += m * Jv.T * Jv + I * Jw.T * Jw       # each body's contribution
M = sp.simplify(M)
T = sp.Rational(1, 2) * (qd.T * M * qd)[0]

# ---------------------------------------------------------------------------
# 4. Coriolis/centrifugal vector from Christoffel symbols:
#    c_i = sum_jk ( dM_ij/dq_k - 1/2 dM_jk/dq_i ) qd_j qd_k
# ---------------------------------------------------------------------------
c = sp.zeros(5, 1)
for i in range(5):
    for j in range(5):
        for k in range(5):
            c[i] += (sp.diff(M[i, j], q[k]) - sp.Rational(1, 2) * sp.diff(M[j, k], q[i])) * qd[j] * qd[k]
c = sp.simplify(c)

# ---------------------------------------------------------------------------
# 5. Numerical functions
# ---------------------------------------------------------------------------
_M_fun = sp.lambdify([q], M, "numpy")
_c_fun = sp.lambdify([q, qd], c, "numpy")
_T_fun = sp.lambdify([q, qd], T, "numpy")


def mass_matrix(qv):
    return np.array(_M_fun(np.asarray(qv, float)), dtype=float)


def coriolis(qv, qdv):
    return np.array(_c_fun(np.asarray(qv, float), np.asarray(qdv, float)), dtype=float).ravel()


def kinetic_energy(qv, qdv):
    return float(_T_fun(np.asarray(qv, float), np.asarray(qdv, float)))


def accelerations(qv, qdv, tau_joints):
    """Forward dynamics: solve M qdd = tau - c for qdd."""
    tau = np.array([0.0, 0.0, 0.0, tau_joints[0], tau_joints[1]])
    return np.linalg.solve(mass_matrix(qv), tau - coriolis(qv, qdv))
