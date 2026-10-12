"""Step 2: derive the dynamics and check them independently."""
import numpy as np
import sympy as sp

import dynamics as d
from kinematics import positions
from params import m0, m1, m2, I0, I1, I2, M_TOTAL

np.set_printoptions(precision=3, suppress=True)
rng = np.random.default_rng(0)

print("Which coordinates does M depend on?", sorted(str(s) for s in d.M.free_symbols))
print("M[0,0] =", d.M[0, 0], "  (should be total mass", M_TOTAL, ")")
print("M[2,2] =", sp.simplify(d.M[2, 2]))
print()

q0 = np.zeros(5)
print("M at q = 0:\n", d.mass_matrix(q0))
print()

# --- Check 1: M symmetric and positive definite at random configurations ---
worst_asym, min_eig = 0.0, np.inf
for _ in range(200):
    qv = rng.uniform(-np.pi, np.pi, 5)
    Mv = d.mass_matrix(qv)
    worst_asym = max(worst_asym, np.abs(Mv - Mv.T).max())
    min_eig = min(min_eig, np.linalg.eigvalsh(Mv).min())
print(f"Check 1: max |M - M^T| = {worst_asym:.1e}, smallest eigenvalue = {min_eig:.3f} (> 0 => pos. definite)")


# --- Check 2: kinetic energy vs an independent numerical computation ---
# Velocities by finite-differencing the (separate) kinematics.py positions.
def T_numeric(qv, qdv, h=1e-6):
    p_plus, p_minus = positions(qv + h * qdv), positions(qv - h * qdv)
    vel = {k: (p_plus[k] - p_minus[k]) / (2 * h) for k in ("r0", "c1", "c2", "phi1", "phi2")}
    w0 = qdv[2]
    return (0.5 * m0 * vel["r0"] @ vel["r0"] + 0.5 * I0 * w0**2
            + 0.5 * m1 * vel["c1"] @ vel["c1"] + 0.5 * I1 * vel["phi1"]**2
            + 0.5 * m2 * vel["c2"] @ vel["c2"] + 0.5 * I2 * vel["phi2"]**2)


worst = 0.0
for _ in range(200):
    qv, qdv = rng.uniform(-np.pi, np.pi, 5), rng.uniform(-1, 1, 5)
    worst = max(worst, abs(d.kinetic_energy(qv, qdv) - T_numeric(qv, qdv)))
print(f"Check 2: max |T_sympy - T_numeric| = {worst:.1e}  (SymPy model matches kinematics.py)")


# --- Check 3: Coriolis vector = dM/dt qd - dT/dq, by finite differences ---
def c_numeric(qv, qdv, h=1e-6):
    Mdot = (d.mass_matrix(qv + h * qdv) - d.mass_matrix(qv - h * qdv)) / (2 * h)
    dTdq = np.array([(d.kinetic_energy(qv + h * e, qdv) - d.kinetic_energy(qv - h * e, qdv)) / (2 * h)
                     for e in np.eye(5)])
    return Mdot @ qdv - dTdq


worst = 0.0
for _ in range(200):
    qv, qdv = rng.uniform(-np.pi, np.pi, 5), rng.uniform(-1, 1, 5)
    worst = max(worst, np.abs(d.coriolis(qv, qdv) - c_numeric(qv, qdv)).max())
print(f"Check 3: max |c_sympy - c_numeric| = {worst:.1e}")

# --- Quick demo: push joint 1 with 1 N m while at rest ---
qdd = d.accelerations(q0, np.zeros(5), [1.0, 0.0])
print()
print("Joint-1 torque of +1 N m at rest gives accelerations")
print("  [x0, y0, th0, q1, q2]dd =", qdd)
print("  -> the base gets a NEGATIVE spin (reaction) while the arm accelerates.")
