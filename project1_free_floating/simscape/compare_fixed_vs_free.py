"""The same 2-link arm on a fixed base and on a free-floating satellite.

Both arms get the same joint reference (0,0) -> (90,-90) deg in 10 s and the same PD gains.
  * Simscape fixed base  (fixed_base_results.mat, run_fixed_base.m)
  * Simscape free-floating (simscape_results.mat, run_simscape_crosscheck.m)
  * Python fixed base: the joint rows of the free-floating equations with the base frozen,
        M_mm(q) qdd_m + c_m(q, [0, qd_m]) = tau
    i.e. the fixed-base robot is the free-floating one minus the base coordinates. M_bb and
    M_bm are exactly what space adds.

Output: figures/fixed_vs_free.png
"""
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp
from scipy.io import loadmat

HERE = os.path.dirname(os.path.abspath(__file__))
P1 = os.path.dirname(HERE)
sys.path.insert(0, P1)

import dynamics as d  # noqa: E402
import simulation as s  # noqa: E402
from kinematics import positions  # noqa: E402
from params import l1, l2  # noqa: E402
from step1_draw import draw_robot  # noqa: E402

deg = np.degrees
fx = loadmat(os.path.join(HERE, "fixed_base_results.mat"))
ff = loadmat(os.path.join(HERE, "simscape_results.mat"))
t = fx["t"].ravel()
assert np.allclose(t, ff["t"].ravel())


# ---- Python fixed-base model: joint rows of M qdd + c = tau, base frozen at 0 --------
def rhs(tt, x):
    qm, qdm = x[:2], x[2:]
    q, qd = np.r_[0, 0, 0, qm], np.r_[0, 0, 0, qdm]
    tau = s.controller(tt, q, qd)
    M = d.mass_matrix(q)[3:, 3:]
    c = d.coriolis(q, qd)[3:]
    return np.r_[qdm, np.linalg.solve(M, tau - c)]


sol = solve_ivp(rhs, (0, t[-1]), np.zeros(4), t_eval=t, method="DOP853", rtol=1e-10, atol=1e-12)
qm_py = sol.y[:2].T
print("Fixed base, Simscape vs Python (M_mm, c_m of the free-floating model, base frozen):")
print(f"  max joint difference  : {deg(np.abs(fx['q'][:, 3:] - qm_py).max()):.1e} deg")

# ---- fixed vs free ------------------------------------------------------------------
ee_fx = np.array([positions(q)["ee"] for q in fx["q"]])
ee_ff = np.array([positions(q)["ee"] for q in ff["q"]])
print("Same joint command, fixed base vs free-floating (both Simscape):")
print(f"  final joints  fixed {deg(fx['q'][-1, 3:]).round(3)} deg, free {deg(ff['q'][-1, 3:]).round(3)} deg")
print(f"  final base    fixed (0, 0, 0), free ({ff['q'][-1, 0]*100:.2f} cm, {ff['q'][-1, 1]*100:.2f} cm,"
      f" {deg(ff['q'][-1, 2]):.2f} deg)")
print(f"  final end effector fixed {ee_fx[-1].round(3)} m, free {ee_ff[-1].round(3)} m,"
      f" {np.linalg.norm(ee_fx[-1] - ee_ff[-1])*100:.1f} cm apart")
print(f"  max |tau|     fixed {np.abs(fx['tau']).max(axis=0).round(3)} N m,"
      f" free {np.abs(ff['tau']).max(axis=0).round(3)} N m")
print(f"  max joint tracking error fixed {deg(np.abs(fx['q'][:, 3:] - np.array([s.desired(x)[0] for x in t])).max()):.3f} deg,"
      f" free {deg(np.abs(ff['q'][:, 3:] - np.array([s.desired(x)[0] for x in t])).max()):.3f} deg")

# ---- why: the inertia the joint motors feel ---------------------------------------------
# fixed base: M_mm;  free-floating: H* = ([M^-1]_mm)^-1 (the base gives way, project 3)
M0 = d.mass_matrix(np.zeros(5))
H0 = np.linalg.inv(np.linalg.inv(M0)[3:, 3:])
print("Inertia felt by the joints at the start (arm straight):")
print(f"  fixed base M_mm = {M0[3:, 3:].round(2).tolist()} kg m^2")
print(f"  free-floating H* = {H0.round(2).tolist()} kg m^2"
      f"  (joint 1: {M0[3, 3] / H0[0, 0]:.1f}x lighter)")

# ---- figure ---------------------------------------------------------------------------
fig = plt.figure(figsize=(14, 8.5))
gs = fig.add_gridspec(2, 3)
ax0 = fig.add_subplot(gs[:, 0])
draw_robot(ax0, fx["q"][0], color="0.6", label="start (both)")
draw_robot(ax0, fx["q"][-1], color="C0", label="fixed base, end")
draw_robot(ax0, ff["q"][-1], color="C3", label="free-floating, end")
ax0.plot(*ee_fx.T, "C0--", lw=1.2)
ax0.plot(*ee_ff.T, "C3--", lw=1.2)
ax0.set_aspect("equal"); ax0.grid(alpha=0.3); ax0.legend(loc="lower left", fontsize=8)
ax0.set_title("Same joint motion (0,0)→(90,−90)°\ndashed: end-effector paths")
ax0.set_xlabel("x [m]"); ax0.set_ylabel("y [m]")

ax = fig.add_subplot(gs[0, 1])
ax.plot(t, deg(fx["q"][:, 2]), "C0", lw=2, label="fixed base")
ax.plot(t, deg(ff["q"][:, 2]), "C3", lw=2, label="free-floating")
ax.set_ylabel("θ0 [deg]"); ax.set_title("Base attitude"); ax.legend()

ax = fig.add_subplot(gs[0, 2])
ax.plot(t, np.linalg.norm(ee_fx - ee_ff, axis=1) * 100, "k")
ax.set_ylabel("[cm]"); ax.set_title("End-effector position: fixed vs free")

for j, axn in ((0, gs[1, 1]), (1, gs[1, 2])):
    ax = fig.add_subplot(axn)
    ax.plot(t, fx["tau"][:, j], "C0", lw=2, label="fixed base")
    ax.plot(t, ff["tau"][:, j], "C3", lw=2, label="free-floating")
    ax.set_ylabel(f"τ{j+1} [N m]"); ax.set_title(f"Joint {j+1} torque"); ax.legend()
    ax.set_xlabel("time [s]")
for a in fig.axes[1:]:
    a.grid(alpha=0.3)
fig.suptitle("2-link arm: fixed base vs free-floating base (Simscape Multibody, same command and gains)")
fig.tight_layout()
out = os.path.join(HERE, "figures", "fixed_vs_free.png")
fig.savefig(out, dpi=130)
print("saved", out)
