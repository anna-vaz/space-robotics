"""Step 5b: animation -- the same end-effector path, three ways.

  left:   min-norm (base ignored), nominal start
  middle: task priority + damped RNS, nominal start   (meets det A = 0, small residual)
  right:  exact RNS from a better starting wrist angle (phi3 = 25 deg): zero base rotation
Kinematic zero-momentum model (step 4 showed the torque-level result is identical).
"""
import os

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon

import params as p
from kinematics import ik_with_wrist_angle, positions, rot
from model import build
from path_tracking import straight_line, track

HERE = os.path.dirname(os.path.abspath(__file__))
rob = build(p)
q_nom = np.array([0, 0, 0, np.radians(30), np.radians(60), np.radians(-45)])
e0 = rob.end_effector(q_nom)
ref = straight_line(e0, e0 + [-0.6, 0.0], 8.0)
q_best = ik_with_wrist_angle(e0, np.radians(25), p.LENGTHS, p.b0)

runs = [("min-norm (base ignored)", track(rob, q_nom, ref, 10.0, "minnorm", n_points=101), "C3"),
        ("task priority, nominal start", track(rob, q_nom, ref, 10.0, "priority", n_points=101), "C2"),
        (r"exact RNS, wrist starts at 25$\degree$", track(rob, q_best, ref, 10.0, "exact", n_points=101), "C0")]
for name, r, _ in runs:
    print(f"{name:40s} th0(end) = {np.degrees(r['q'][-1, 2]):+.4f} deg, stopped = {r['stopped']}")

fig, axs = plt.subplots(1, 3, figsize=(13, 4.8))
artists = []
for ax, (name, r, col) in zip(axs, runs):
    ax.set_aspect("equal"); ax.set_xlim(-0.9, 2.4); ax.set_ylim(-0.8, 2.3); ax.grid(alpha=0.3)
    ax.set_title(name, fontsize=11)
    ax.plot(*np.array([e0, e0 + [-0.6, 0]]).T, "k--", lw=1)
    base = Polygon(np.zeros((4, 2)), closed=True, fc=col, alpha=0.2, ec=col)
    ax.add_patch(base)
    mark, = ax.plot([], [], color=col, lw=2.5)
    arm, = ax.plot([], [], "-o", color=col, lw=3, ms=5)
    trail, = ax.plot([], [], color="k", lw=1)
    txt = ax.text(0.03, 0.95, "", transform=ax.transAxes, va="top", fontsize=11, family="monospace")
    artists.append((r, base, mark, arm, trail, txt))
fig.suptitle("Same end-effector path, different use of the redundant joint", fontsize=12)
fig.tight_layout()
h = p.a0 / 2
square = np.array([[h, h], [-h, h], [-h, -h], [h, -h]])


def frame(k):
    out = []
    for r, base, mark, arm, trail, txt in artists:
        q = r["q"][min(k, len(r["q"]) - 1)]
        pos = positions(q, p.LENGTHS, p.b0)
        base.set_xy((rot(q[2]) @ square.T).T + pos["r0"])
        mark.set_data(*np.array([pos["r0"], pos["r0"] + rot(q[2]) @ [0, h]]).T)
        arm.set_data(*pos["joints"].T)
        ee = r["ee"][:k + 1]
        trail.set_data(ee[:, 0], ee[:, 1])
        txt.set_text(f"t = {r['t'][min(k, len(r['t']) - 1)]:4.1f} s\nbase {round(np.degrees(q[2]), 2) + 0.0:+6.2f}°")
        out += [base, mark, arm, trail, txt]
    return out


anim = FuncAnimation(fig, frame, frames=101 + 15, interval=100, blit=False)
out = os.path.join(HERE, "figures", "reaction_null_space.gif")
anim.save(out, writer=PillowWriter(fps=10), dpi=70)
frame(100)
fig.savefig(os.path.join(HERE, "figures", "step5_final_frame.png"), dpi=110)  # inspection only (gitignored)
print("saved", out, f"({os.path.getsize(out) / 1e6:.1f} MB)")
