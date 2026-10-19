"""Small drawing helper: the planar free-floating robot (project 1 geometry)."""
import numpy as np
from matplotlib.patches import Polygon

from kinematics import positions, rot     # project 1
from params import a0                       # project 1


def draw(ax, q, color="C0", alpha=1.0):
    p = positions(q)
    h = a0 / 2
    corners = (rot(q[2]) @ np.array([[h, h], [-h, h], [-h, -h], [h, -h]]).T).T + p["r0"]
    ax.add_patch(Polygon(corners, closed=True, fc=color, alpha=0.15 * alpha, ec=color))
    arm = np.array([p["j1"], p["j2"], p["ee"]])
    ax.plot(arm[:, 0], arm[:, 1], "-o", color=color, lw=2.5, ms=4, alpha=alpha)
