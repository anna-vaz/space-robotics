"""Project 3, step 5b: animation of the full capture (3 deg/s target).

Uses results_step2.npz (reach, 0-32 s) and results_step4.npz (grasp + detumble).
Time steps are not uniform: 0.4 s per frame while reaching, 0.25 s around the
grasp, 10 s per frame during the slow detumble.
"""
import os

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon

import capture as cp
import target as tg
from kinematics import positions, rot          # project 1 (path set by capture -> gen_jacobian)
from params import a0

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "figures", "capture.gif")
r2 = np.load(os.path.join(HERE, "results_step2.npz"))
r4 = np.load(os.path.join(HERE, "results_step4.npz"))
tg.set_spin(3.0); tg.set_target(50.0, 1.0)

frames = []                                     # (time, q, target [x, y, th], wheel h, phase)
for ts in np.arange(0, cp.T_GRASP, 0.4):
    k = int(np.argmin(np.abs(r2["t"] - ts)))
    c, th, _, _ = tg.state(r2["t"][k])
    frames.append((r2["t"][k], r2["q"][k], np.array([*c, th]), 0.0, "1 reach the handle"))
t4, X4 = r4["t"], r4["X"]
for ts in np.concatenate([np.arange(0, 10, 0.25), np.arange(10, t4[-1], 10)]):
    k = int(np.argmin(np.abs(t4 - ts)))
    ph = "2 grasp" if ts < 10 else "3 detumble with the reaction wheel"
    frames.append((cp.T_GRASP + t4[k], X4[k, :5], X4[k, 10:13], cp.I_W * X4[k, 16], ph))
frames += [frames[-1]] * 15                     # pause at the end

fig, (ax, axb) = plt.subplots(1, 2, figsize=(10, 5.6), gridspec_kw=dict(width_ratios=[5, 1]))
h0 = a0 / 2
BASE = np.array([[h0, h0], [-h0, h0], [-h0, -h0], [h0, -h0]])
TGT = np.array([[1, 1], [-1, 1], [-1, -1], [1, -1]]) * tg.A_T / 2


def update(n):
    t, q, xt, h, phase = frames[n]
    ax.cla(); axb.cla()
    p = positions(q)
    ax.add_patch(Polygon((rot(q[2]) @ BASE.T).T + p["r0"], closed=True, fc="C0", alpha=0.3, ec="C0", lw=2))
    ax.plot(*np.array([p["r0"], p["r0"] + 0.45 * np.array([-np.sin(q[2]), np.cos(q[2])])]).T, "C3", lw=2)
    arm = np.array([p["j1"], p["j2"], p["ee"]])
    ax.plot(arm[:, 0], arm[:, 1], "-o", color="C0", lw=4, ms=6)
    R = rot(xt[2])
    ax.add_patch(Polygon((R @ TGT.T).T + xt[:2], closed=True, fc="0.6", alpha=0.6, ec="0.3", lw=2))
    n_h = R @ np.array([1.0, 0.0])
    ax.plot(*np.array([xt[:2] + 0.5 * n_h, xt[:2] + tg.HANDLE_R * n_h]).T, color="k", lw=5)
    ax.set_xlim(-1.2, 3.0); ax.set_ylim(-1.0, 2.6); ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title(f"Phase {phase}")
    ax.text(0.02, 0.98, f"t = {t:6.1f} s\nbase attitude {np.degrees(q[2]):6.1f} deg\n"
            f"target angle  {np.degrees(xt[2]) % 360:6.1f} deg", transform=ax.transAxes,
            va="top", family="monospace", fontsize=9)
    axb.bar([0], [h], color="C2" if abs(h) < cp.H_MAX * 0.999 else "C3", width=0.6)
    axb.axhline(cp.H_MAX, color="k", ls=":"); axb.set_ylim(0, 1.1); axb.set_xticks([])
    axb.set_title("wheel momentum\n[N m s]", fontsize=9)
    return []


fig.subplots_adjust(left=0.08, right=0.97, bottom=0.1, top=0.9, wspace=0.25)
anim = FuncAnimation(fig, update, frames=len(frames))
anim.save(OUT, writer=PillowWriter(fps=12), dpi=70)
print(f"saved {OUT} ({len(frames)} frames, {os.path.getsize(OUT) / 1e6:.1f} MB)")
