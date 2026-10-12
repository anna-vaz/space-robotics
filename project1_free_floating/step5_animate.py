"""Step 5: animate the free-floating robot (saved as an animated GIF)."""
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon

import simulation as s
from kinematics import positions, system_com, rot
from params import a0

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "figures", "free_floating.gif")

STRIDE = 5                      # use every 5th sample -> 0.05 s per frame
FPS = 20                        # 20 frames/s -> plays back in real time

t, q, qd, tau = s.simulate()
t, q = t[::STRIDE], q[::STRIDE]
ee = np.array([positions(qi)["ee"] for qi in q])
com = system_com(q[0])

fig, (ax, ax2) = plt.subplots(1, 2, figsize=(11, 4.4), gridspec_kw=dict(width_ratios=[1.2, 1]))

# ---- Left: the robot ----
ax.set_aspect("equal")
ax.set_xlim(-1.0, 2.8); ax.set_ylim(-1.1, 1.3)
ax.grid(alpha=0.3); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.set_title("Free-floating space robot")
ghost = Polygon(np.zeros((4, 2)), closed=True, fc="none", ec="gray", ls="--", lw=1)
ax.add_patch(ghost)              # initial base pose, for reference
base = Polygon(np.zeros((4, 2)), closed=True, fc="C0", alpha=0.3, ec="C0", lw=2)
ax.add_patch(base)
arm, = ax.plot([], [], "-o", color="C0", lw=4, ms=6)
trace, = ax.plot([], [], "k:", lw=1)
ax.plot(*com, "kx", ms=10, mew=2, label="system CoM (fixed)")
heading, = ax.plot([], [], "C3", lw=2, label="base heading")
time_txt = ax.text(0.02, 0.95, "", transform=ax.transAxes, va="top")
ax.legend(loc="lower right", fontsize=8)

h = a0 / 2
corners_body = np.array([[h, h], [-h, h], [-h, -h], [h, -h]])
ghost.set_xy((rot(q[0, 2]) @ corners_body.T).T + q[0, :2])

# ---- Right: base attitude over time ----
ax2.set_xlim(0, t[-1]); ax2.set_ylim(np.degrees(q[:, 2]).min() - 5, 5)
ax2.grid(alpha=0.3); ax2.set_xlabel("time [s]"); ax2.set_ylabel("base attitude theta0 [deg]")
ax2.set_title("Base rotates in reaction to the arm")
ax2.axvline(s.T_MOVE, color="gray", ls="--", lw=1)
ax2.text(s.T_MOVE + 0.2, -2, "arm stops", color="gray", fontsize=8)
th_line, = ax2.plot([], [], "C3", lw=2)
fig.tight_layout()


def update(k):
    qk = q[k]
    p = positions(qk)
    base.set_xy((rot(qk[2]) @ corners_body.T).T + p["r0"])
    arm.set_data(*np.array([p["j1"], p["j2"], p["ee"]]).T)
    trace.set_data(ee[:k + 1, 0], ee[:k + 1, 1])
    tip = p["r0"] + 0.45 * np.array([np.cos(qk[2] + np.pi / 2), np.sin(qk[2] + np.pi / 2)])
    heading.set_data([p["r0"][0], tip[0]], [p["r0"][1], tip[1]])
    th_line.set_data(t[:k + 1], np.degrees(q[:k + 1, 2]))
    time_txt.set_text(f"t = {t[k]:4.1f} s\ntheta0 = {np.degrees(qk[2]):6.1f} deg")
    return base, arm, trace, heading, th_line, time_txt


anim = FuncAnimation(fig, update, frames=len(t), interval=1000 / FPS, blit=True)
anim.save(OUT, writer=PillowWriter(fps=FPS), dpi=80)
print(f"saved {OUT} ({len(t)} frames, {os.path.getsize(OUT) / 1e6:.1f} MB)")
