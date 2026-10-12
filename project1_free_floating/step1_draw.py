"""Step 1: draw the robot in a few configurations and mark its centre of mass."""
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

from params import a0
from kinematics import positions, system_com, rot

HERE = os.path.dirname(os.path.abspath(__file__))


def draw_robot(ax, q, color="C0", label=None):
    p = positions(q)
    # base square: corners in the base frame, rotated and shifted to the inertial frame
    h = a0 / 2
    corners = np.array([[h, h], [-h, h], [-h, -h], [h, -h]])
    corners = (rot(q[2]) @ corners.T).T + p["r0"]
    ax.add_patch(Polygon(corners, closed=True, fc=color, alpha=0.25, ec=color))
    # arm
    arm = np.array([p["j1"], p["j2"], p["ee"]])
    ax.plot(arm[:, 0], arm[:, 1], "-o", color=color, lw=3, label=label)
    # centres of mass
    ax.plot(*p["r0"], "+", color=color, ms=10)
    com = system_com(q)
    ax.plot(*com, "x", color="k", ms=10, mew=2)


if __name__ == "__main__":
    configs = {
        "arm straight":   [0, 0, 0, 0, 0],
        "elbow up":       [0, 0, 0, np.pi / 4, np.pi / 2],
        "base rotated":   [0, 0, np.pi / 6, -np.pi / 3, np.pi / 2],
    }
    fig, ax = plt.subplots(figsize=(7, 5))
    for (name, q), c in zip(configs.items(), ["C0", "C1", "C2"]):
        draw_robot(ax, np.array(q, float), color=c, label=name)
        print(f"{name:14s} end effector = {positions(q)['ee'].round(3)}, "
              f"system CoM = {system_com(q).round(3)}")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Planar free-floating robot (+ base CoM, x system CoM)")
    ax.legend()
    os.makedirs(os.path.join(HERE, "figures"), exist_ok=True)
    out = os.path.join(HERE, "figures", "step1_configurations.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print("saved", out)
