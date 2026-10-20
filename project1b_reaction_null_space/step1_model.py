"""Step 1: n-link free-floating model, checked against project 1 and independently.

Checks
  A. the 2-link robot built by the new general code reproduces project 1's M and c
  B. 3-link robot: M symmetric, positive definite
  C. 3-link kinetic energy  vs  finite-differenced plain-NumPy kinematics
  D. 3-link Coriolis vector vs  finite differences  Mdot qd - dT/dq
  E. end-effector position  vs  kinematics.py
  F. a short torque-driven simulation conserves P and L
and draws the 3-link robot in a few configurations.
"""
import os
import subprocess
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
from scipy.integrate import solve_ivp

import params as p
from kinematics import positions, rot, system_com
from model import PlanarFreeFloatingRobot, build

HERE = os.path.dirname(os.path.abspath(__file__))
P1 = os.path.join(os.path.dirname(HERE), "project1_free_floating")
rng = np.random.default_rng(1)
np.set_printoptions(precision=3, suppress=True)

# ---------------------------------------------------------------- A. project 1
# project 1's modules have the same names as ours, so load its params.py under another name
import importlib.util  # noqa: E402

spec = importlib.util.spec_from_file_location("p1_params", os.path.join(P1, "params.py"))
p1 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(p1)
two = PlanarFreeFloatingRobot([p1.l1, p1.l2], [p1.m1, p1.m2], [p1.I1, p1.I2], p1.m0, p1.I0, p1.b0)

Q = rng.uniform(-np.pi, np.pi, (50, 5))
QD = rng.uniform(-1, 1, (50, 5))
np.savez(os.path.join(HERE, "_checkA.npz"), Q=Q, QD=QD)
# run project 1's own dynamics.py in a separate process (its modules have the same names)
code = ("import numpy as np, dynamics as d; z=np.load(r'%s');"
        "np.savez(r'%s', M=[d.mass_matrix(q) for q in z['Q']],"
        " c=[d.coriolis(q,qd) for q,qd in zip(z['Q'],z['QD'])])"
        % (os.path.join(HERE, "_checkA.npz"), os.path.join(HERE, "_checkA_out.npz")))
subprocess.run([sys.executable, "-c", code], cwd=P1, check=True)
with np.load(os.path.join(HERE, "_checkA_out.npz")) as z:
    ref = {k: z[k] for k in z.files}
errM = max(np.abs(two.mass_matrix(q) - Mr).max() for q, Mr in zip(Q, ref["M"]))
errc = max(np.abs(two.coriolis(q, qd) - cr).max() for q, qd, cr in zip(Q, QD, ref["c"]))
for f in ("_checkA.npz", "_checkA_out.npz"):
    os.remove(os.path.join(HERE, f))
print(f"A. 2-link general model vs project 1:  max|dM| = {errM:.1e},  max|dc| = {errc:.1e}")

# ---------------------------------------------------------------- 3-link robot
t0 = time.time()
rob = build(p)
print(f"\n3-link robot: {rob.n + 3} generalized coordinates, SymPy build {time.time() - t0:.1f} s")
print("M at q = 0:\n", rob.mass_matrix(np.zeros(6)))
print("M depends on:", sorted(str(s) for s in rob.M_sym.free_symbols))

# B
asym, min_eig = 0.0, np.inf
for _ in range(500):
    M = rob.mass_matrix(rng.uniform(-np.pi, np.pi, 6))
    asym = max(asym, np.abs(M - M.T).max())
    min_eig = min(min_eig, np.linalg.eigvalsh(M).min())
print(f"\nB. max|M - M^T| = {asym:.1e},  smallest eigenvalue over 500 configs = {min_eig:.3f} > 0")


# C
def T_numeric(q, qd, h=1e-6):
    pp = positions(q + h * qd, p.LENGTHS, p.b0)
    pm = positions(q - h * qd, p.LENGTHS, p.b0)
    v0 = (pp["r0"] - pm["r0"]) / (2 * h)
    vc = (pp["centres"] - pm["centres"]) / (2 * h)
    w = (pp["phi"] - pm["phi"]) / (2 * h)
    return (0.5 * p.m0 * v0 @ v0 + 0.5 * p.I0 * qd[2] ** 2
            + 0.5 * np.sum(p.MASSES * np.sum(vc**2, axis=1)) + 0.5 * np.sum(p.INERTIAS * w**2))


errT = 0.0
for _ in range(300):
    q, qd = rng.uniform(-np.pi, np.pi, 6), rng.uniform(-1, 1, 6)
    errT = max(errT, abs(rob.kinetic_energy(q, qd) - T_numeric(q, qd)))
print(f"C. max|T_sympy - T_numeric| = {errT:.1e}")


# D
def c_numeric(q, qd, h=1e-6):
    Mdot = (rob.mass_matrix(q + h * qd) - rob.mass_matrix(q - h * qd)) / (2 * h)
    dTdq = np.array([(rob.kinetic_energy(q + h * e, qd) - rob.kinetic_energy(q - h * e, qd)) / (2 * h)
                     for e in np.eye(6)])
    return Mdot @ qd - dTdq


errc3 = 0.0
for _ in range(300):
    q, qd = rng.uniform(-np.pi, np.pi, 6), rng.uniform(-1, 1, 6)
    errc3 = max(errc3, np.abs(rob.coriolis(q, qd) - c_numeric(q, qd)).max())
print(f"D. max|c_sympy - c_numeric| = {errc3:.1e}")

# E
errE = max(np.abs(rob.end_effector(q) - positions(q, p.LENGTHS, p.b0)["ee"]).max()
           for q in rng.uniform(-np.pi, np.pi, (300, 6)))
print(f"E. max|ee_sympy - ee_numpy| = {errE:.1e}")
print("   straight arm, q = 0:  ee =", rob.end_effector(np.zeros(6)),
      " (hand check: 0.5 + 1 + 1 + 0.5 = 3.0 m)")
print("   system CoM at q = 0:", system_com(np.zeros(6), p.LENGTHS, p.MASSES, p.m0, p.b0),
      " (hand check: (10*1 + 10*2 + 5*2.75)/125 = 0.35 m)")

# F: drive the joints with arbitrary torques, starting with a drifting/spinning base
tau = lambda t: np.array([2 * np.sin(t), -1.5 * np.cos(0.7 * t), 0.5 * np.sin(2 * t)])
x0 = np.concatenate([[0, 0, 0.1, 0.3, -0.5, 0.8], [0.02, -0.01, 0.03, 0, 0, 0]])
sol = solve_ivp(lambda t, x: np.concatenate([x[6:], rob.accelerations(x[:6], x[6:], tau(t))]),
                (0, 10), x0, method="DOP853", rtol=1e-11, atol=1e-12, t_eval=np.linspace(0, 10, 201))
h = np.array([rob.momentum(x[:6], x[6:]) for x in sol.y.T])
print(f"F. 10 s with joint torques:  max|P - P(0)| = {np.abs(h[:, :2] - h[0, :2]).max():.1e} kg m/s,"
      f"  max|L - L(0)| = {np.abs(h[:, 2] - h[0, 2]).max():.1e} kg m^2/s"
      f"   (P(0) = {h[0, :2]}, L(0) = {h[0, 2]:.3f})")

# ---------------------------------------------------------------- figure
fig, ax = plt.subplots(figsize=(8, 5))
configs = {"arm straight": [0, 0, 0, 0, 0, 0],
           "folded wrist": [0, 0, 0, np.pi / 4, np.pi / 2, np.pi / 2],
           "base rotated": [0, 0, np.pi / 6, -np.pi / 3, np.pi / 2, -np.pi / 4]}
for (name, q), col in zip(configs.items(), ["C0", "C1", "C2"]):
    q = np.array(q, float)
    pos = positions(q, p.LENGTHS, p.b0)
    hh = p.a0 / 2
    corners = (rot(q[2]) @ np.array([[hh, hh], [-hh, hh], [-hh, -hh], [hh, -hh]]).T).T + pos["r0"]
    ax.add_patch(Polygon(corners, closed=True, fc=col, alpha=0.2, ec=col))
    ax.plot(*pos["joints"].T, "-o", color=col, lw=3, label=name)
    ax.plot(*pos["ee"], "s", color=col, ms=8)
    ax.plot(*system_com(q, p.LENGTHS, p.MASSES, p.m0, p.b0), "x", color="k", ms=10, mew=2)
ax.set_aspect("equal")
ax.grid(alpha=0.3)
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_title("Redundant planar free-floating robot: base + 3-link arm (x = system CoM)")
ax.legend()
os.makedirs(os.path.join(HERE, "figures"), exist_ok=True)
out = os.path.join(HERE, "figures", "step1_robot.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
print("saved", out)
