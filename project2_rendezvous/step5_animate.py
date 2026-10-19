"""Step 5: 3D animation of the docking mission (from the step 4 results).

The camera zooms with the range: far away at the start, down to the docking ports
at the end. Time steps are not uniform: 20 s per frame during the approach,
30 s while holding, 2 s during the final approach.
Run step4_docking.py first (it writes results_step4.npz).
"""
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import attitude as att
import mission as m
import orbit as o

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "figures", "docking.gif")
d = np.load(os.path.join(HERE, "results_step4.npz"))
t, X, rel, point_err = d["t"], d["X"], d["rel"], d["point_err"]

# ---- frame schedule ---------------------------------------------------------
frames = np.concatenate([np.arange(0, 3000, 20), np.arange(3000, 3600, 30),
                         np.arange(3600, t[-1], 2), [t[-1]]])
idx = np.unique(np.searchsorted(t, frames).clip(0, len(t) - 1))
idx = np.concatenate([idx, [idx[-1]] * 24])          # pause ~2 s on the docked state


def box(center, C, half):
    """Faces of a box (half sizes `half`) at `center`, axes = columns of C (in LVLH)."""
    s = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]) * half
    v = center + s @ C.T
    faces = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]
    return [[v[i] for i in f] for f in faces]


def to_plot(p):
    """LVLH (x radial, y along-track, z cross) -> plot axes (along-track, cross, radial-up)."""
    p = np.asarray(p)
    return p[..., [1, 2, 0]]


fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(projection="3d")
fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
info = fig.text(0.02, 0.97, "", va="top", family="monospace", fontsize=10)
fig.text(0.98, 0.97, "grey: target (holds LVLH attitude)\nblue: chaser (1.5 m cube)\n"
         "red: chaser docking axis b1\ngreen: target docking port\nblack: path flown",
         fontsize=9, va="top", ha="right")
trail = to_plot(rel[:, :3])


def phase(tk):
    if tk >= t[-1]:
        return "CONTACT - docked"
    if tk < 3000:
        return "1 approach"
    if tk < m.T_HOLD_END:
        return "2 hold at 10 m"
    return "3 final approach"


def update(n):
    k = idx[n]
    ax.cla()
    tk, r = t[k], rel[k, :3]
    LN = o.eci_to_lvlh_dcm(*o.target_state(tk))
    C_LB = LN @ att.ep2dcm(X[k, 6:10]).T                 # body axes in LVLH (columns)

    # target (2 m cube) + its docking port on the -y face
    ax.add_collection3d(Poly3DCollection([to_plot(f) for f in box(np.zeros(3), np.eye(3), 1.0)],
                                         facecolor="0.7", edgecolor="0.3", alpha=0.6))
    port_t = to_plot([0, m.TARGET_PORT_Y, 0])
    ax.scatter(*port_t, color="g", s=40)
    # chaser (1.5 m cube) + docking axis
    ax.add_collection3d(Poly3DCollection([to_plot(f) for f in box(r, C_LB, 0.75)],
                                         facecolor="C0", edgecolor="navy", alpha=0.7))
    ax.scatter(*to_plot(r), color="C0", s=25, depthshade=False)   # visible even when far
    ax.scatter(0, 0, 0, color="0.4", s=25, depthshade=False)
    b1 = C_LB[:, 0]
    ax.plot(*to_plot(np.array([r, r + max(1.5, 0.08 * np.linalg.norm(r)) * b1])).T, color="r", lw=2.5)
    # trail of the past trajectory
    ax.plot(*trail[:k + 1].T, color="k", lw=0.8, alpha=0.6)

    # camera: zoom with the range
    rng_ = np.linalg.norm(r)
    half = max(3.0, 0.6 * rng_)
    c = to_plot(r / 2)
    ax.set_xlim(c[0] - half, c[0] + half); ax.set_ylim(c[1] - half, c[1] + half)
    ax.set_zlim(c[2] - half, c[2] + half)
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel("along-track y [m]"); ax.set_ylabel("cross-track z [m]"); ax.set_zlabel("radial x [m]")
    ax.view_init(elev=18, azim=-60)
    info.set_text(f"t = {tk/60:5.1f} min   phase {phase(tk)}\n"
                  f"range       {rng_:8.2f} m\n"
                  f"speed       {np.linalg.norm(rel[k, 3:])*100:8.2f} cm/s\n"
                  f"pointing err {np.degrees(point_err[k]):7.2f} deg")
    return []


anim = FuncAnimation(fig, update, frames=len(idx), interval=80)
anim.save(OUT, writer=PillowWriter(fps=12), dpi=70)
print(f"saved {OUT} ({len(idx)} frames, {os.path.getsize(OUT)/1e6:.1f} MB)")
