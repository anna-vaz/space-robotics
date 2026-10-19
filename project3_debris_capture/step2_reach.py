"""Project 3, step 2: reach a tumbling target's handle with the free-floating arm."""
import os

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
from scipy.integrate import solve_ivp

import gen_jacobian as gj
import dynamics as d
import reach_control as rc
import target as tg
from step1_helpers import draw

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, "figures")
deg = np.degrees

q0 = np.radians([0, 0, 0, 30, 100]); q0[:2] = 0
guid = rc.Guidance(gj.end_effector(q0))


def rhs(t, x):
    q, qd = x[:5], x[5:]
    tau, _ = rc.joint_torques(t, q, qd, guid)
    return np.concatenate([qd, d.accelerations(q, qd, tau)])


t = np.linspace(0, rc.T_END, 1751)
sol = solve_ivp(rhs, (0, rc.T_END), np.concatenate([q0, np.zeros(5)]), t_eval=t,
                method="DOP853", rtol=1e-9, atol=1e-11, max_step=0.05)
q, qd = sol.y[:5].T, sol.y[5:].T
np.savez(os.path.join(HERE, "results_step2.npz"), t=t, q=q, qd=qd)

ee = np.array([gj.end_effector(qk) for qk in q])
v_ee = np.array([gj.generalized_jacobian(qk) @ qdk[3:] for qk, qdk in zip(q, qd)])
H = np.array([tg.handle(tk)[0] for tk in t]); VH = np.array([tg.handle(tk)[1] for tk in t])
ref = np.array([guid.reference(tk)[0] for tk in t])
out = [rc.joint_torques(tk, qk, qdk, guid) for tk, qk, qdk in zip(t, q, qd)]
tau = np.array([o[0] for o in out]); w = np.array([o[1] for o in out])
dist = np.linalg.norm(ee - H, axis=1)
track = np.linalg.norm(ee - ref, axis=1)
vrel = np.linalg.norm(v_ee - VH, axis=1)
fin = t >= rc.T_CLOSE

print(f"Target: {tg.M_T:.0f} kg plate, spinning {deg(tg.W_T):.0f} deg/s, handle speed "
      f"{100*tg.HANDLE_R*tg.W_T:.1f} cm/s")
print(f"Max tracking error of the reference: {100*track.max():.2f} cm "
      f"(after 2 s: {100*track[t > 2].max():.2f} cm)")
print(f"Hand-handle distance during final tracking (30-35 s): max {1000*dist[fin].max():.2f} mm")
print(f"Relative velocity during final tracking: max {1000*vrel[fin].max():.2f} mm/s")
print(f"Min manipulability of J*: {w.min():.3f}  (damping active below {rc.W_DAMP})")
print(f"Peak joint torques: {np.abs(tau).max(axis=0).round(3)} N m (limit {rc.TAU_MAX})")
print(f"Base: rotated {deg(q[-1, 2]):.2f} deg, moved {100*np.linalg.norm(q[-1, :2]):.2f} cm")
print(f"Momentum check: max |qd_base - B qd_m| = "
      f"{max(np.abs(qdk[:3] - gj.base_coupling(qk) @ qdk[3:]).max() for qk, qdk in zip(q, qd)):.1e}")

# ---- Figure 1: snapshots ---------------------------------------------------------
fig, ax = plt.subplots(figsize=(8, 7))
snaps = [0, 10, 20, 25, 30]
cols = plt.cm.viridis(np.linspace(0, 0.9, len(snaps)))
for ts, c in zip(snaps, cols):
    k = np.searchsorted(t, ts)
    ax.add_patch(Polygon(tg.corners(ts), closed=True, fc=c, alpha=0.12, ec=c))
    hp, _, _, n, _ = tg.handle(ts)
    c_t = tg.state(ts)[0]
    ax.plot(*np.array([c_t + 0.5 * n, hp]).T, color=c, lw=4)       # handle stub
    draw(ax, q[k], c, alpha=0.9)
    ax.plot([], [], color=c, lw=3, label=f"t = {ts:.0f} s")
ax.plot(H[:, 0], H[:, 1], "k:", lw=1, label="handle path")
ax.plot(ee[:, 0], ee[:, 1], "r-", lw=1.5, label="hand path")
ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend(fontsize=8, loc="lower right")
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.set_title("Reaching the handle of a tumbling target (3 deg/s)")
fig.savefig(os.path.join(FIG, "step2_snapshots.png"), dpi=150, bbox_inches="tight")

# ---- Figure 2: time histories ----------------------------------------------------
fig, ax = plt.subplots(2, 2, figsize=(12, 7), sharex=True)
ax[0, 0].semilogy(t, 100 * dist, label="hand - handle distance")
ax[0, 0].semilogy(t, 100 * track + 1e-6, label="hand - reference (tracking error)")
ax[0, 0].set_ylabel("cm"); ax[0, 0].set_title("Position errors"); ax[0, 0].legend(fontsize=8)
ax[0, 1].plot(t, 100 * vrel); ax[0, 1].set_ylabel("cm/s")
ax[0, 1].set_title("Hand velocity relative to the handle")
ax[1, 0].plot(t, tau[:, 0], label="tau1"); ax[1, 0].plot(t, tau[:, 1], label="tau2")
ax[1, 0].set_ylabel("N m"); ax[1, 0].set_title("Joint torques"); ax[1, 0].legend(fontsize=8)
ax[1, 1].plot(t, deg(q[:, 2]), "C3", label="base attitude [deg]")
ax[1, 1].plot(t, 100 * w, "C2", label="manipulability of J* (x100)")
ax[1, 1].set_title("Base reaction and distance from singularity"); ax[1, 1].legend(fontsize=8)
for a in ax.flat:
    a.grid(alpha=0.3, which="both")
    for tp in (rc.T_APPROACH, rc.T_SYNC, rc.T_CLOSE):
        a.axvline(tp, color="gray", ls="--", lw=0.8)
for a in ax[1]:
    a.set_xlabel("time [s]   (dashed: end of approach, start of close-in, hand on handle)")
fig.tight_layout()
fig.savefig(os.path.join(FIG, "step2_time_histories.png"), dpi=150)
print("saved figures to", FIG)
