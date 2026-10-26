"""Explainer figure: the folded 3-link arm in the step-4 RNS capture, links labelled.

    .venv\\Scripts\\python project5_ros2_gazebo\\explain_arm.py
"""
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from step4_compare import A_T, D, R, load, rob, state  # noqa: E402

d = load("rns")
SQ = np.array([[0.5, 0.5], [-0.5, 0.5], [-0.5, -0.5], [0.5, -0.5]])
COLS = ["C0", "C2", "C3"]


def rot(a):
    return np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])


fig, axs = plt.subplots(1, 3, figsize=(15, 5.2))
for ax, t, title in zip(axs, (0.0, R.T1, d["t_capture"]),
                        ("t = 0 s: start (folded arm)", "t = 20 s: at the stand-off point",
                         "t = 28 s: capture")):
    q, _, c, th, _, _ = state(d, t)
    r0, joints, _, _, ee = rob._chain(q)
    ax.add_patch(Polygon(r0 + SQ @ rot(q[2]).T, closed=True, fc="goldenrod", ec="k", alpha=0.8))
    ax.plot(*np.array([r0, r0 + 0.45 * np.array([np.cos(q[2]), np.sin(q[2])])]).T, "k-", lw=2)
    ax.text(*(r0 + [-0.35, -0.3]), f"base\n{D(q[2]):+.2f} deg", fontsize=9)
    ax.add_patch(Polygon(c + (A_T * SQ) @ rot(th).T, closed=True, fc="0.8", ec="k"))
    h = c + R.HANDLE_R * np.array([np.cos(th), np.sin(th)])
    ax.plot(*h, "s", color="C3", ms=9)
    ax.text(*(c + [-0.2, 0.05]), "target", fontsize=9)
    pts = joints + [ee]
    lengths = ["1.0 m", "1.0 m", "0.5 m"]
    for i in range(3):
        a, b = pts[i], pts[i + 1]
        ax.plot(*np.array([a, b]).T, "-", color=COLS[i], lw=6, solid_capstyle="round")
        mid = (a + b) / 2
        ax.text(*(mid + [0.04, 0.04]), f"link {i+1}\n({lengths[i]})", color=COLS[i], fontsize=9,
                fontweight="bold")
    for name, p in zip(("shoulder", "elbow", "wrist"), joints):
        ax.plot(*p, "ko", ms=6)
        ax.text(*(p + [0.03, -0.12]), name, fontsize=8)
    ax.plot(*ee, "k^", ms=8)
    ax.text(*(ee + [0.05, 0.05]), "gripper", fontsize=8)
    ax.set_aspect("equal")
    ax.set_xlim(-0.8, 2.6)
    ax.set_ylim(-0.8, 2.1)
    ax.grid(alpha=0.3)
    ax.set_title(f"{title}\nq = ({D(q[3]):.0f}, {D(q[4]):.0f}, {D(q[5]):.0f}) deg", fontsize=10)
fig.suptitle("The 3-link arm in the RNS capture run (Gazebo data): links 1 and 2 fold into a 'V', "
             "link 3 is the short wrist", fontsize=12)
fig.tight_layout()
out = os.path.join(HERE, "figures", "explain_arm.png")
fig.savefig(out, dpi=120)
print(out)
