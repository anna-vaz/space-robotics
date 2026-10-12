"""Step 3: simulate the arm motion and plot how the base reacts."""
import os
import numpy as np
import matplotlib.pyplot as plt

import simulation as s
from kinematics import positions
from step1_draw import draw_robot

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, "figures")
os.makedirs(FIG, exist_ok=True)

t, q, qd, tau = s.simulate()
np.savez(os.path.join(HERE, "results_step3.npz"), t=t, q=q, qd=qd, tau=tau)

q_des = np.array([s.desired(ti)[0] for ti in t])
deg = np.degrees

print(f"Simulated {t[-1]:.0f} s, {len(t)} samples")
print(f"Final joint angles  : {deg(q[-1, 3:]).round(2)} deg (goal {deg(s.Q_GOAL)})")
print(f"Final base attitude : {deg(q[-1, 2]):.2f} deg (started at 0)")
print(f"Final base position : {q[-1, :2].round(4)} m (started at [0, 0])")
print(f"Max joint error     : {deg(np.abs(q[:, 3:] - q_des).max()):.2f} deg")
print(f"Max torques         : {np.abs(tau).max(axis=0).round(2)} N m")
print(f"Base rates at end   : {qd[-1, :3].round(6)}  (should be ~0 once the arm stops)")

# ---- Figure 1: time histories ---------------------------------------------
fig, ax = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
for j, c in zip((0, 1), ("C0", "C1")):
    ax[0, 0].plot(t, deg(q[:, 3 + j]), c, label=f"q{j+1}")
    ax[0, 0].plot(t, deg(q_des[:, j]), c + "--", lw=1, label=f"q{j+1} desired")
ax[0, 0].set_ylabel("joint angle [deg]"); ax[0, 0].set_title("Arm joints (controlled)")

ax[0, 1].plot(t, deg(q[:, 2]), "C3")
ax[0, 1].set_ylabel("theta0 [deg]"); ax[0, 1].set_title("Base attitude (NOT controlled)")

ax[1, 0].plot(t, q[:, 0] * 100, label="x0"); ax[1, 0].plot(t, q[:, 1] * 100, label="y0")
ax[1, 0].set_ylabel("base position [cm]"); ax[1, 0].set_title("Base position (NOT controlled)")

ax[1, 1].plot(t, tau[:, 0], label="tau1"); ax[1, 1].plot(t, tau[:, 1], label="tau2")
ax[1, 1].set_ylabel("torque [N m]"); ax[1, 1].set_title("Joint torques")
for a in ax.flat:
    a.grid(alpha=0.3); a.legend(fontsize=8) if a.get_legend_handles_labels()[0] else None
for a in ax[1]:
    a.set_xlabel("time [s]")
fig.tight_layout()
fig.savefig(os.path.join(FIG, "step3_time_histories.png"), dpi=150)

# ---- Figure 2: snapshots of the motion ------------------------------------
fig, ax = plt.subplots(figsize=(7, 6))
idx = np.linspace(0, np.searchsorted(t, s.T_MOVE), 5).astype(int)
for k, i in enumerate(idx):
    draw_robot(ax, q[i], color=plt.cm.viridis(k / (len(idx) - 1)), label=f"t = {t[i]:.1f} s")
ee = np.array([positions(qi)["ee"] for qi in q])
ax.plot(ee[:, 0], ee[:, 1], "k:", lw=1, label="end-effector path")
ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend(fontsize=8)
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.set_title("Arm motion and base reaction")
fig.savefig(os.path.join(FIG, "step3_snapshots.png"), dpi=150, bbox_inches="tight")
print("saved figures to", FIG)
