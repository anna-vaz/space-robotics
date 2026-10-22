"""Step 4: torque-level simulation with the full free-floating dynamics.

Runs (start at rest, zero momentum):
  1. path A, exact RNS          -- compare with the kinematic result of step 3
  2. path B, task priority      -- compare with step 3
  3. path B, task priority, joints start 3 deg off (dispersion: feedback must work)
  4. path B, min-norm           -- reference (base ignored)
Checks: momentum P, L conserved;  work-energy  T(t) - T(0) = integral tau . qd_m dt.
"""
import os
import time

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import cumulative_trapezoid

import params as p
import torque_control as tc
from model import build
from path_tracking import straight_line, track

HERE = os.path.dirname(os.path.abspath(__file__))
rob = build(p)
q0 = np.array([0, 0, 0, np.radians(30), np.radians(60), np.radians(-45)])
e0 = rob.end_effector(q0)
T_MOVE, T_END = 8.0, 10.0
ref = {"A": straight_line(e0, e0 + [-0.8, -0.5], T_MOVE), "B": straight_line(e0, e0 + [-0.6, 0.0], T_MOVE)}
d0 = np.radians([3.0, -3.0, 3.0])
q0d = q0.copy(); q0d[3:] += d0
RUNS = [("A, exact RNS", "A", "exact", q0),
        ("B, task priority", "B", "priority", q0),
        ("B, task priority, 3 deg joint error", "B", "priority", q0d),
        ("B, min-norm", "B", "minnorm", q0)]

res = {}
for name, path, method, qs in RUNS:
    t0 = time.time()
    r = tc.simulate(rob, qs, ref[path], T_END, method)
    kin = track(rob, qs, ref[path], T_END, method, n_points=1001)
    work = cumulative_trapezoid(np.sum(r["tau"] * r["qd"][:, 3:], axis=1), r["t"], initial=0)
    res[name] = (r, kin)
    print(f"\n{name}   ({time.time() - t0:.0f} s)")
    print(f"  base attitude: max|th0| = {np.degrees(np.abs(r['q'][:, 2]).max()):.4f} deg,"
          f" th0(end) = {np.degrees(r['q'][-1, 2]):+.4f} deg   (kinematic step 3: {np.degrees(kin['q'][-1, 2]):+.4f} deg,"
          f" max diff {np.degrees(np.abs(r['q'][:, 2] - kin['q'][:, 2]).max()):.1e} deg)")
    err = np.linalg.norm(r["ee"] - r["ee_des"], axis=1)
    print(f"  end-effector error: max {1e3 * err.max():.3f} mm, after t = 2 s {1e3 * err[r['t'] > 2].max():.4f} mm,"
          f" at end {1e3 * err[-1]:.2e} mm")
    print(f"  joint-rate loop error max|qd_des - qd_m| = {np.abs(r['qd_des'] - r['qd'][:, 3:]).max():.1e} rad/s")
    print(f"  torques: max|tau| = {np.abs(r['tau']).max(axis=0).round(3)} N m")
    print(f"  momentum: max|P| = {np.abs(r['h'][:, :2]).max():.1e} kg m/s, max|L| = {np.abs(r['h'][:, 2]).max():.1e} kg m^2/s")
    print(f"  work-energy: max|T - T0 - W| = {np.abs(r['T'] - r['T'][0] - work).max():.1e} J"
          f"  (max T = {r['T'].max():.3f} J)")

# ---------------------------------------------------------------- figure
fig, axs = plt.subplots(2, 2, figsize=(13, 8.5))
cols = ["C0", "C2", "C1", "C3"]
ax = axs[0, 0]
for (name, (r, kin)), c in zip(res.items(), cols):
    ax.plot(r["t"], np.degrees(r["q"][:, 2]), color=c, label=name)
    ax.plot(kin["t"], np.degrees(kin["q"][:, 2]), "k:", lw=1)
ax.set_title("base attitude (dotted: kinematic model of step 3)"); ax.set_ylabel(r"$\theta_0$ [deg]")
ax.legend(fontsize=8); ax.grid(alpha=0.3)
ax = axs[0, 1]
for (name, (r, _)), c in zip(res.items(), cols):
    ax.semilogy(r["t"], 1e3 * np.linalg.norm(r["ee"] - r["ee_des"], axis=1) + 1e-12, color=c, label=name)
ax.set_title("end-effector tracking error"); ax.set_ylabel("[mm]"); ax.set_ylim(1e-9, 1e3)
ax.legend(fontsize=8); ax.grid(alpha=0.3)
ax = axs[1, 0]
r = res["B, task priority"][0]
for j in range(3):
    ax.plot(r["t"], r["tau"][:, j], label=rf"$\tau_{j + 1}$, task priority")
r = res["B, min-norm"][0]
for j in range(3):
    ax.plot(r["t"], r["tau"][:, j], ":", color=f"C{j}", label=rf"$\tau_{j + 1}$, min-norm")
ax.set_title("path B: joint torques"); ax.set_ylabel("[N m]"); ax.set_xlabel("t [s]")
ax.legend(fontsize=8, ncol=2); ax.grid(alpha=0.3)
ax = axs[1, 1]
for (name, (r, _)), c in zip(res.items(), cols):
    h0 = r["h"][0]
    ax.semilogy(r["t"], np.abs(r["h"] - h0).max(axis=1) + 1e-18, color=c, label=name)
ax.set_title("momentum conservation: max|[P, L] - [P, L](0)|"); ax.set_xlabel("t [s]")
ax.legend(fontsize=8); ax.grid(alpha=0.3)
fig.tight_layout()
out = os.path.join(HERE, "figures", "step4_torque_level.png")
fig.savefig(out, dpi=140, bbox_inches="tight")
np.savez(os.path.join(HERE, "results_step4.npz"),
         **{f"{k.split(',')[0]}_{i}_q": v[0]["q"] for i, (k, v) in enumerate(res.items())},
         t=res["A, exact RNS"][0]["t"])
print("saved", out)
