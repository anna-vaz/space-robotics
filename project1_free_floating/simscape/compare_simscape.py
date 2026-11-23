"""Compare the Simscape Multibody model with the hand-derived Python model of project 1.

Inputs:
  ../results_step3.npz       Python run (step3_simulate.py; regenerated here if missing)
  simscape_results.mat       Simscape run (run_simscape_crosscheck.m)

Checks:
  1. same time histories (base x0, y0, th0, joints, torques) on the same 0.01 s grid
  2. the Simscape trajectory itself conserves momentum: with the robot starting at rest
     the system centre of mass must stay at its start point and P = L = 0
     (evaluated with the kinematics only -- positions and velocities of the body CoMs).
"""
import os
import subprocess
import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.io import loadmat

HERE = os.path.dirname(os.path.abspath(__file__))
P1 = os.path.dirname(HERE)
sys.path.insert(0, P1)

from kinematics import system_com  # noqa: E402
from params import I0, I1, I2, b0, l1, l2, m0, m1, m2  # noqa: E402

npz = os.path.join(P1, "results_step3.npz")
if not os.path.exists(npz):
    subprocess.run([sys.executable, "step3_simulate.py"], cwd=P1, check=True)
with np.load(npz) as d:
    t_py, q_py, qd_py, tau_py = d["t"], d["q"], d["qd"], d["tau"]

sm = loadmat(os.path.join(HERE, "simscape_results.mat"))
t_sm, q_sm, qd_sm, tau_sm = sm["t"].ravel(), sm["q"], sm["qd"], sm["tau"]
assert np.allclose(t_py, t_sm), "different time grids"
deg = np.degrees

# ---- 1. time histories ----------------------------------------------------------
print("Simscape Multibody vs hand-derived Lagrange model (Python), 15 s, 1501 samples")
print(f"  final th0   : Simscape {deg(q_sm[-1, 2]):.5f} deg, Python {deg(q_py[-1, 2]):.5f} deg")
print(f"  final x0, y0: Simscape {q_sm[-1, :2].round(5)} m, Python {q_py[-1, :2].round(5)} m")
err = {
    "base x0, y0 [m]": np.abs(q_sm[:, :2] - q_py[:, :2]).max(),
    "base th0 [deg]": deg(np.abs(q_sm[:, 2] - q_py[:, 2]).max()),
    "joints q1, q2 [deg]": deg(np.abs(q_sm[:, 3:] - q_py[:, 3:]).max()),
    "base rates [rad/s, m/s]": np.abs(qd_sm[:, :3] - qd_py[:, :3]).max(),
    "torques [N m]": np.abs(tau_sm - tau_py).max(),
}
for k, v in err.items():
    print(f"  max difference, {k:24s}: {v:.1e}")


# ---- 2. momentum of the Simscape trajectory (kinematics only) -------------------
def momentum(q, qd):
    """P and L about the origin from the CoM velocities (planar, exact)."""
    x0, y0, th, q1, q2 = q
    dx, dy, dth, dq1, dq2 = qd
    u = lambda a: np.array([np.cos(a), np.sin(a)])          # noqa: E731
    n = lambda a: np.array([-np.sin(a), np.cos(a)])         # noqa: E731  d u/d a
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    p1, p2 = th + q1, th + q1 + q2
    w0, w1, w2 = dth, dth + dq1, dth + dq1 + dq2
    r0 = np.array([x0, y0]); v0 = np.array([dx, dy])
    j1 = r0 + R @ np.array(b0); vj1 = v0 + w0 * (R @ np.array([-b0[1], b0[0]]))
    c1 = j1 + l1 / 2 * u(p1);   v1 = vj1 + l1 / 2 * w1 * n(p1)
    j2 = j1 + l1 * u(p1);       vj2 = vj1 + l1 * w1 * n(p1)
    c2 = j2 + l2 / 2 * u(p2);   v2 = vj2 + l2 / 2 * w2 * n(p2)
    P = m0 * v0 + m1 * v1 + m2 * v2
    cross = lambda r, v: r[0] * v[1] - r[1] * v[0]          # noqa: E731
    L = (m0 * cross(r0, v0) + I0 * w0 + m1 * cross(c1, v1) + I1 * w1
         + m2 * cross(c2, v2) + I2 * w2)
    return P, L


PL = [momentum(qi, qdi) for qi, qdi in zip(q_sm, qd_sm)]
P = np.array([p for p, _ in PL]); L = np.array([lv for _, lv in PL])
com = np.array([system_com(qi) for qi in q_sm])
print("Momentum of the Simscape trajectory (starts at rest, so all should stay 0):")
print(f"  max |P|              : {np.abs(P).max():.1e} kg m/s")
print(f"  max |L|              : {np.abs(L).max():.1e} kg m^2/s")
print(f"  max CoM displacement : {np.linalg.norm(com - com[0], axis=1).max():.1e} m")

# ---- figure ------------------------------------------------------------------------
fig, ax = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
ax[0, 0].plot(t_py, deg(q_py[:, 2]), "C3", lw=3, alpha=0.4, label="Python (Lagrange)")
ax[0, 0].plot(t_sm, deg(q_sm[:, 2]), "k--", lw=1.2, label="Simscape Multibody")
ax[0, 0].set_ylabel("θ0 [deg]"); ax[0, 0].set_title("Base attitude (not controlled)"); ax[0, 0].legend()
for j, c in ((0, "C0"), (1, "C1")):
    ax[0, 1].plot(t_py, q_py[:, j] * 100, c, lw=3, alpha=0.4, label=f"{'xy'[j]}0 Python")
    ax[0, 1].plot(t_sm, q_sm[:, j] * 100, "k--", lw=1.2)
ax[0, 1].set_ylabel("base position [cm]"); ax[0, 1].set_title("Base position (dashed: Simscape)")
ax[0, 1].legend()
ax[1, 0].semilogy(t_sm, np.maximum(deg(np.abs(q_sm[:, 2] - q_py[:, 2])), 1e-16), "C3", label="θ0 [deg]")
ax[1, 0].semilogy(t_sm, np.maximum(deg(np.abs(q_sm[:, 3:] - q_py[:, 3:]).max(axis=1)), 1e-16), "C2",
                  label="q1, q2 [deg]")
ax[1, 0].semilogy(t_sm, np.maximum(np.abs(q_sm[:, :2] - q_py[:, :2]).max(axis=1), 1e-16), "C0",
                  label="x0, y0 [m]")
ax[1, 0].set_ylabel("|Simscape − Python|"); ax[1, 0].set_title("Difference between the two models")
ax[1, 0].legend(); ax[1, 0].set_xlabel("time [s]")
ax[1, 1].semilogy(t_sm, np.maximum(np.linalg.norm(P, axis=1), 1e-18), label="|P| [kg m/s]")
ax[1, 1].semilogy(t_sm, np.maximum(np.abs(L), 1e-18), label="|L| [kg m²/s]")
ax[1, 1].set_title("Momentum of the Simscape run (should be 0)"); ax[1, 1].legend()
ax[1, 1].set_xlabel("time [s]")
for a in ax.flat:
    a.grid(alpha=0.3)
fig.suptitle("Project 1 – Simscape Multibody model vs hand-derived Lagrange equations")
fig.tight_layout()
os.makedirs(os.path.join(HERE, "figures"), exist_ok=True)
out = os.path.join(HERE, "figures", "simscape_vs_python.png")
fig.savefig(out, dpi=130)
print("saved", out)
