"""Step 5: animation of the Gazebo capture runs (top view), RNS vs plain J*.

Plays back the logged Gazebo data of step 4 (no re-simulation): base, arm, target and
handle, with a base-attitude trace underneath. A heading tick on the base shows its
rotation; the label shows the phase (approach / spin synchronisation / captured).

    .venv\\Scripts\\python project5_ros2_gazebo\\step5_animate.py
"""
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from step4_compare import A_T, D, R, load, rob, state  # noqa: E402

FIG = os.path.join(HERE, "figures")
RUNS = [("rns", "Reactionless (RNS) approach", "C0"), ("minnorm", "Plain generalized-Jacobian approach", "C3")]
data = {m: load(m) for m, _, _ in RUNS}
T_END = min(d["j_t"][-1] for d in data.values()) - 0.1
times = np.arange(0.0, T_END, 0.4)
SQ = np.array([[0.5, 0.5], [-0.5, 0.5], [-0.5, -0.5], [0.5, -0.5]])
LINK_COLS = ["C0", "C2", "C3"]          # same colours as figures/explain_arm.png


def rot(a):
    return np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])


fig = plt.figure(figsize=(12, 7.2))
gs = fig.add_gridspec(2, 2, height_ratios=[3.2, 1.0])
art = {}
for k, (m, title, col) in enumerate(RUNS):
    ax = fig.add_subplot(gs[0, k])
    ax.set_aspect("equal")
    ax.set_xlim(-0.9, 2.7)
    ax.set_ylim(-0.9, 2.2)
    ax.grid(alpha=0.3)
    ax.set_title(title, fontsize=11)
    base = Polygon(np.zeros((4, 2)), closed=True, fc="goldenrod", ec="k", alpha=0.85)
    tgt = Polygon(np.zeros((4, 2)), closed=True, fc="0.75", ec="k")
    ax.add_patch(base)
    ax.add_patch(tgt)
    heading, = ax.plot([], [], "k-", lw=2)
    links = [ax.plot([], [], "-", color=lc, lw=5, solid_capstyle="round", label=lab)[0]
             for lc, lab in zip(LINK_COLS, ("link 1 (1.0 m)", "link 2 (1.0 m)", "link 3 (0.5 m)"))]
    joints_dots, = ax.plot([], [], "ko", ms=4)
    if k == 0:
        ax.legend(loc="lower right", fontsize=8)
    handle, = ax.plot([], [], "s", color="C3", ms=8)
    trail, = ax.plot([], [], ":", color="k", lw=1)
    txt = ax.text(0.02, 0.97, "", transform=ax.transAxes, va="top", fontsize=10, family="monospace")
    art[m] = dict(base=base, tgt=tgt, heading=heading, links=links, dots=joints_dots, handle=handle,
                  trail=trail, txt=txt, ee=[])

axb = fig.add_subplot(gs[1, :])
for m, title, col in RUNS:
    d = data[m]
    axb.plot(d["b_t"], D(d["base"][:, 2]), col, lw=2, label=title)
axb.axvspan(R.T1, R.T2, color="0.9")
axb.text((R.T1 + R.T2) / 2, axb.get_ylim()[1] * 0.85, "spin sync", ha="center", fontsize=8)
axb.axvline(R.T2, color="k", ls=":", lw=1)
axb.set(xlabel="t [s]", ylabel=r"base $\theta_0$ [deg]", xlim=(0, T_END))
axb.legend(fontsize=8, loc="lower left")
cursor = axb.axvline(0, color="k", lw=1)
fig.suptitle("ROS 2 + Gazebo: capturing a tumbling target with a free-floating 3-link arm", fontsize=13)
fig.tight_layout()


def phase(t, tc):
    if t >= tc:
        return "CAPTURED - rigid stack"
    if t >= R.T1:
        return "final approach + spin sync"
    return "approach"


def update(i):
    t = times[i]
    out = [cursor]
    cursor.set_xdata([t, t])
    for m, _, _ in RUNS:
        d, a = data[m], art[m]
        q, _, c, th, _, _ = state(d, t)
        r0, joints, _, _, ee = rob._chain(q)
        a["base"].set_xy(r0 + SQ @ rot(q[2]).T)
        a["heading"].set_data(*np.array([r0, r0 + 0.45 * np.array([np.cos(q[2]), np.sin(q[2])])]).T)
        pts = np.array(joints + [ee])
        for i, ln in enumerate(a["links"]):
            ln.set_data(pts[i:i + 2, 0], pts[i:i + 2, 1])
        a["dots"].set_data(pts[:3, 0], pts[:3, 1])
        a["tgt"].set_xy(c + (A_T * SQ) @ rot(th).T)
        a["handle"].set_data([c[0] + R.HANDLE_R * np.cos(th)], [c[1] + R.HANDLE_R * np.sin(th)])
        a["ee"].append(ee)
        a["trail"].set_data(*np.array(a["ee"]).T)
        a["txt"].set_text(f"t = {t:5.1f} s   {phase(t, d['t_capture'])}\nbase angle {D(q[2]):+6.2f} deg")
        out += [a["base"], a["heading"], *a["links"], a["dots"], a["tgt"], a["handle"], a["trail"], a["txt"]]
    return out


anim = FuncAnimation(fig, update, frames=len(times), blit=False)
out = os.path.join(FIG, "zero_g_capture.gif")
anim.save(out, writer=PillowWriter(fps=10), dpi=70)
fig.savefig(os.path.join(FIG, "step5_final_frame.png"), dpi=110)   # inspection only (gitignored)
print("saved", out, f"({os.path.getsize(out) / 1e6:.1f} MB, {len(times)} frames)")
