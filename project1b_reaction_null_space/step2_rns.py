"""Step 2: the Reaction Null Space -- moving the arm without rotating the base.

  1. b(q) and null(b) at the straight-arm configuration, with checks
  2. same desired joint rates, executed (a) as commanded, (b) projected onto null(b);
     integrate the zero-momentum motion and compare the base attitude
  3. map of det A(q) (A = [J*; b]): where can the end effector move in every
     direction with zero base rotation?
"""
import os

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
from scipy.integrate import solve_ivp

import params as p
import rns
from kinematics import positions, rot
from model import build

HERE = os.path.dirname(os.path.abspath(__file__))
np.set_printoptions(precision=4, suppress=True)
rob = build(p)

# ------------------------------------------------------------------ 1. b and null(b)
q0 = np.array([0, 0, 0, np.radians(30), np.radians(60), np.radians(-45)])
B = rns.base_coupling(rob, q0)
b = B[2]
print("q0 = [0, 0, 0, 30, 60, -45 deg]")
print("b(q0) = dth0/d(q1, q2, q3) =", b)
print("  -> all negative: turning any joint counter-clockwise turns the base clockwise")
Pn = rns.rns_projector(rob, q0)
U, s, Vt = np.linalg.svd(b[None, :])
N = Vt[1:].T                                         # orthonormal basis of null(b), 3 x 2
print("null(b) basis (columns):\n", N)
print(f"check |b N| = {np.abs(b @ N).max():.1e},  rank(P) = {np.linalg.matrix_rank(Pn)}")

# zero momentum check of the full rates and the system-CoM check
qd_m = np.array([0.2, -0.3, 0.4])
qd = rns.full_rates(rob, q0, qd_m)
print(f"momentum of qd = B qd_m: {rob.momentum(q0, qd)}  (should be 0)")


# ------------------------------------------------------------------ 2. simulate
QD_DES = np.array([0.20, -0.30, 0.40])      # rad/s, constant desired joint rates
T = 4.0


def run(project):
    def f(t, q):
        v = rns.rns_projector(rob, q) @ QD_DES if project else QD_DES
        return rns.full_rates(rob, q, v)
    return solve_ivp(f, (0, T), q0, method="DOP853", rtol=1e-11, atol=1e-12,
                     t_eval=np.linspace(0, T, 201))


free, rnsm = run(False), run(True)
for name, sol in (("as commanded", free), ("RNS-projected", rnsm)):
    th0 = sol.y[2]
    com = np.array([(p.m0 * q[:2] + p.MASSES @ positions(q, p.LENGTHS, p.b0)["centres"]) / p.M_TOTAL
                    for q in sol.y.T])
    print(f"{name:14s}: base rotation after {T:.0f} s = {np.degrees(th0[-1]):+9.4f} deg"
          f"  (max |th0| = {np.degrees(np.abs(th0).max()):.2e} deg),"
          f"  system CoM moved {np.abs(com - com[0]).max():.1e} m,"
          f"  joints moved {np.degrees(sol.y[3:, -1] - q0[3:]).round(1)} deg")
# hand check of the commanded case at t = 0:  w0 = b . qd
print(f"hand check: initial base rate b.qd = {b @ QD_DES:+.4f} rad/s ;"
      f" numeric = {(free.y[2, 1] - free.y[2, 0]) / (free.t[1] - free.t[0]):+.4f} rad/s")

# ------------------------------------------------------------------ 3. det A map
# det A does not depend on x0, y0, th0 (rotating the base rotates v_e, b is unchanged)
grid = np.radians(np.linspace(-180, 180, 145))
q1_slices = [0, 60, 120]
maps = []
for q1 in q1_slices:
    D = np.empty((grid.size, grid.size))
    for i, q3 in enumerate(grid):
        for j, q2 in enumerate(grid):
            D[i, j] = np.linalg.det(rns.extended_jacobian(rob, [0, 0, 0, np.radians(q1), q2, q3]))
    maps.append(D)
    print(f"q1 = {q1:3d} deg: det A in [{D.min():+.3f}, {D.max():+.3f}]")
# rotation invariance check
qa = np.array([0.3, -0.2, 0.0, 0.4, 1.0, -0.7])
qb = qa.copy(); qb[:3] = [1.5, 2.0, 1.1]
print(f"det A independent of base pose: {np.linalg.det(rns.extended_jacobian(rob, qa)):.6f} vs"
      f" {np.linalg.det(rns.extended_jacobian(rob, qb)):.6f}")


# ------------------------------------------------------------------ figures
def draw(ax, q, col, alpha=1.0, label=None):
    pos = positions(q, p.LENGTHS, p.b0)
    h = p.a0 / 2
    c = (rot(q[2]) @ np.array([[h, h], [-h, h], [-h, -h], [h, -h]]).T).T + pos["r0"]
    ax.add_patch(Polygon(c, closed=True, fc=col, alpha=0.15 * alpha, ec=col))
    # a mark on the base so its rotation is visible
    ax.plot(*np.array([pos["r0"], pos["r0"] + rot(q[2]) @ [0, h]]).T, color=col, lw=2, alpha=alpha)
    ax.plot(*pos["joints"].T, "-o", color=col, lw=2.5, ms=4, alpha=alpha, label=label)


fig, axs = plt.subplots(1, 3, figsize=(15, 4.6))
for ax, sol, col, title in ((axs[0], free, "C3", "joint rates as commanded"),
                            (axs[1], rnsm, "C2", "projected onto the Reaction Null Space")):
    for k, a in zip([0, 100, 200], [0.35, 0.65, 1.0]):
        draw(ax, sol.y[:, k], col, a, label=f"t = {sol.t[k]:.0f} s")
    ee = np.array([positions(q, p.LENGTHS, p.b0)["ee"] for q in sol.y.T])
    ax.plot(*ee.T, ":", color="k", lw=1, label="end-effector path")
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.set_title(title)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.legend(fontsize=8, loc="lower left")
    ax.set_xlim(-1, 3.2); ax.set_ylim(-1.2, 2.6)
ax = axs[2]
ax.plot(free.t, np.degrees(free.y[2]), "C3", label="as commanded")
ax.plot(rnsm.t, np.degrees(rnsm.y[2]), "C2", label="RNS-projected")
ax.set_xlabel("t [s]"); ax.set_ylabel(r"base attitude $\theta_0$ [deg]"); ax.grid(alpha=0.3)
ax.legend(); ax.set_title("base attitude")
fig.tight_layout()
out1 = os.path.join(HERE, "figures", "step2_rns_motion.png")
fig.savefig(out1, dpi=150, bbox_inches="tight")

fig, axs = plt.subplots(1, 3, figsize=(15, 4.6), sharey=True)
lim = max(np.abs(D).max() for D in maps)
for ax, D, q1 in zip(axs, maps, q1_slices):
    g = np.degrees(grid)
    im = ax.pcolormesh(g, g, D, cmap="RdBu_r", vmin=-lim, vmax=lim, shading="auto")
    ax.contour(g, g, D, levels=[0], colors="k", linewidths=1.5)
    ax.set_title(rf"$q_1$ = {q1}°")
    ax.set_xlabel(r"$q_2$ [deg]"); ax.set_aspect("equal")
axs[0].set_ylabel(r"$q_3$ [deg]")
fig.suptitle("Where can the end effector move in every direction with zero base rotation?  "
             "(black: det A = 0, RNS singular)", y=1.02)
fig.colorbar(im, ax=axs, label=r"det $[J^*;\ b]$  [m$^2$]", shrink=0.85)
out2 = os.path.join(HERE, "figures", "step2_rns_singularity_map.png")
fig.savefig(out2, dpi=150, bbox_inches="tight")
print("saved", out1, "\nsaved", out2)
