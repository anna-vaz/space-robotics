"""Step 3: following a straight end-effector path -- three ways to use the redundancy.

Path A: 0.94 m down-left, stays well away from det A = 0
Path B: 0.60 m to the left, the reactionless motion runs into det A = 0
Methods (see path_tracking.py): minnorm, exact RNS, task priority (damped RNS)
Also a sweep of the damping lam on path B (base rotation vs joint-rate trade-off).
"""
import os

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

import params as p
import rns
from kinematics import positions, rot
from model import build
from path_tracking import straight_line, track

HERE = os.path.dirname(os.path.abspath(__file__))
rob = build(p)
q0 = np.array([0, 0, 0, np.radians(30), np.radians(60), np.radians(-45)])
e0 = rob.end_effector(q0)
T_MOVE, T_END = 8.0, 10.0
PATHS = {"A": e0 + np.array([-0.8, -0.5]), "B": e0 + np.array([-0.6, 0.0])}
METHODS = {"minnorm": ("min-norm J*$^+$ (base ignored)", "C3"),
           "exact": ("exact RNS", "C0"),
           "priority": ("task priority + damped RNS", "C2")}

print(f"start: q = [30, 60, -45] deg, end effector at {e0.round(3)} m")
res = {}
for path, goal in PATHS.items():
    ref = straight_line(e0, goal, T_MOVE)
    print(f"\nPath {path}: to {goal.round(3)} ({np.linalg.norm(goal - e0):.2f} m in {T_MOVE:.0f} s, hold to {T_END:.0f} s)")
    print(f"  {'method':9s} {'max|th0|':>11s} {'th0(end)':>11s} {'max|qd_m|':>10s} {'max EE err':>11s}"
          f" {'min|det A|':>11s} {'det A(end)':>10s} {'smin(J*)':>8s}  note")
    for m in METHODS:
        r = track(rob, q0, ref, T_END, m)
        res[path, m] = r
        note = f"STOPPED at t = {r['t_stop']:.2f} s (joint rate > 10 rad/s)" if r["stopped"] else ""
        print(f"  {m:9s} {np.degrees(np.abs(r['q'][:, 2]).max()):9.4f} deg"
              f" {np.degrees(r['q'][-1, 2]):+9.4f} deg {np.abs(r['qd_m']).max():6.3f} r/s"
              f" {np.abs(r['ee'] - r['ee_des']).max():9.1e} m {np.abs(r['detA']).min():9.4f}"
              f" {r['detA'][-1]:+10.4f} {min(np.linalg.svd(rns.generalized_jacobian(rob, q), compute_uv=False)[-1] for q in r['q']):8.3f}  {note}")

# ---------------------------------------------------------------- damping sweep, path B
import path_tracking as pt  # noqa: E402

refB = straight_line(e0, PATHS["B"], T_MOVE)
lams = [0.002, 0.005, 0.01, 0.02, 0.04, 0.08]
sweep = []
print("\nPath B, damping sweep (task priority):")
for lam in lams:
    pt.LAM = lam
    r = track(rob, q0, refB, T_END, "priority", rate_limit=50.0)
    sweep.append((np.degrees(np.abs(r["q"][:, 2]).max()), np.abs(r["qd_m"]).max(), r["stopped"]))
    print(f"  lam = {lam:6.3f} m^2: max|th0| = {sweep[-1][0]:7.3f} deg, max joint rate = {sweep[-1][1]:6.3f} rad/s"
          f"{'  STOPPED' if r['stopped'] else ''}")
pt.LAM = 0.02


# ---------------------------------------------------------------- figures
def draw(ax, q, col, alpha=1.0, label=None):
    pos = positions(q, p.LENGTHS, p.b0)
    h = p.a0 / 2
    c = (rot(q[2]) @ np.array([[h, h], [-h, h], [-h, -h], [h, -h]]).T).T + pos["r0"]
    ax.add_patch(Polygon(c, closed=True, fc="none", ec=col, alpha=alpha, lw=1.5))
    ax.plot(*np.array([pos["r0"], pos["r0"] + rot(q[2]) @ [0, h]]).T, color=col, lw=2, alpha=alpha)
    ax.plot(*pos["joints"].T, "-o", color=col, lw=2.5, ms=4, alpha=alpha, label=label)


fig, axs = plt.subplots(2, 3, figsize=(16, 9.5))
for row, path in enumerate(PATHS):
    ax = axs[row, 0]
    draw(ax, q0, "0.6", 0.6, label="start")
    for m, (lab, col) in METHODS.items():
        r = res[path, m]
        draw(ax, r["q"][-1], col, 0.9, label=lab + (" (stopped)" if r["stopped"] else ""))
        ax.plot(*r["ee"].T, color=col, lw=1)
    ax.plot(*np.array([e0, PATHS[path]]).T, "k--", lw=1, label="desired path")
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend(fontsize=7, loc="lower left")
    ax.set_xlim(-1.2, 3.0); ax.set_ylim(-1.3, 2.6)
    ax.set_title(f"Path {path}: final configurations"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")

    ax = axs[row, 1]
    for m, (lab, col) in METHODS.items():
        r = res[path, m]
        ax.plot(r["t"], np.degrees(r["q"][:, 2]), color=col, label=lab)
    ax.set_title(f"Path {path}: base attitude"); ax.set_xlabel("t [s]"); ax.set_ylabel(r"$\theta_0$ [deg]")
    ax.grid(alpha=0.3); ax.legend(fontsize=8)

    ax = axs[row, 2]
    for m, (lab, col) in METHODS.items():
        r = res[path, m]
        ax.plot(r["t"], r["detA"], color=col, label=f"det A, {m}")
        ax.plot(r["t"], np.abs(r["qd_m"]).max(axis=1) / 10, ":", color=col)
    ax.axhline(0, color="k", lw=0.8)
    ax.set_title(f"Path {path}: det A (solid) and max joint rate / 10 [rad/s] (dotted)", fontsize=10)
    ax.set_xlabel("t [s]"); ax.grid(alpha=0.3); ax.legend(fontsize=8)
fig.tight_layout()
out1 = os.path.join(HERE, "figures", "step3_path_tracking.png")
fig.savefig(out1, dpi=130, bbox_inches="tight")

fig, ax = plt.subplots(figsize=(6.5, 4.5))
th, rate = np.array([s[0] for s in sweep]), np.array([s[1] for s in sweep])
ax.plot(rate, th, "o-", color="C2")
for lam, x, y in zip(lams, rate, th):
    ax.annotate(rf"$\lambda$={lam}", (x, y), textcoords="offset points", xytext=(6, 4), fontsize=8)
ax.set_xlabel("max joint rate [rad/s]"); ax.set_ylabel(r"max base rotation $|\theta_0|$ [deg]")
ax.set_title("Path B: damping trades base rotation against joint rate"); ax.grid(alpha=0.3)
out2 = os.path.join(HERE, "figures", "step3_damping_tradeoff.png")
fig.savefig(out2, dpi=150, bbox_inches="tight")
print("saved", out1, "\nsaved", out2)
