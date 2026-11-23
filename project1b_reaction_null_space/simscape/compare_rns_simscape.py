"""Compare project 1b on the Simscape Multibody plant with the Python (zero-momentum) model.

Python: path_tracking.track() integrates qd = [B(q) qd_m, qd_m], i.e. the base motion is
PREDICTED from momentum conservation with the hand-derived inertia matrix.
Simscape: the joints are driven by the same control law and the base moves however the
multibody physics says it must. If the Reaction Null Space theory and the model behind it
are right, the two base histories agree -- in particular th0 stays at 0 for exact RNS.

Inputs: rns_simscape_results.mat (run_rns_simscape.m). Output: figures/rns_simscape_vs_python.png
"""
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.io import loadmat

HERE = os.path.dirname(os.path.abspath(__file__))
P1B = os.path.dirname(HERE)
sys.path.insert(0, P1B)
os.chdir(P1B)

import params as p  # noqa: E402
from model import build  # noqa: E402
from path_tracking import straight_line, track  # noqa: E402

rob = build(p)
q0 = np.array([0, 0, 0, np.radians(30), np.radians(60), np.radians(-45)])
e0 = rob.end_effector(q0)
GOALS = {"A": e0 + np.array([-0.8, -0.5]), "B": e0 + np.array([-0.6, 0.0])}
CASES = [("A", "minnorm"), ("A", "exact"), ("A", "priority"),
         ("B", "minnorm"), ("B", "exact"), ("B", "priority")]
LABEL = {"minnorm": "min-norm", "exact": "exact RNS", "priority": "task priority"}
COL = {"minnorm": "C3", "exact": "C0", "priority": "C2"}

sm = loadmat(os.path.join(HERE, "rns_simscape_results.mat"), squeeze_me=True, struct_as_record=False)
deg = np.degrees
res = {}
print(f"{'case':16s} {'max|th0| Simscape':>18s} {'Python':>10s} {'max diff th0':>13s}"
      f" {'base x,y':>9s} {'joints':>9s} {'det A':>8s}")
for path, meth in CASES:
    s = sm[f"{path}_{meth}"]
    t, Y = np.ravel(s.t), np.atleast_2d(s.Y)
    r = track(rob, q0, straight_line(e0, GOALS[path], 8.0), t[-1], meth, rate_limit=1e9,
              n_points=len(t))
    assert np.allclose(r["t"], t)
    q_py = r["q"]
    d_th = deg(np.abs(Y[:, 2] - q_py[:, 2]).max())
    d_xy = np.abs(Y[:, :2] - q_py[:, :2]).max()
    d_qm = deg(np.abs(Y[:, 3:6] - q_py[:, 3:]).max())
    d_det = np.abs(Y[:, 12] - r["detA"]).max()
    res[path, meth] = (t, Y, r)
    print(f"{path} {LABEL[meth]:14s} {deg(np.abs(Y[:, 2]).max()):14.4f} deg {deg(np.abs(q_py[:, 2]).max()):7.4f} deg"
          f" {d_th:9.1e} deg {d_xy:7.1e} m {d_qm:5.1e} deg {d_det:8.1e}")

fig, ax = plt.subplots(2, 2, figsize=(12, 7.5), sharex=True)
for row, path in enumerate("AB"):
    for meth in LABEL:
        t, Y, r = res[path, meth]
        ax[row, 0].plot(r["t"], deg(r["q"][:, 2]), COL[meth], lw=3.5, alpha=0.35)
        ax[row, 0].plot(t, deg(Y[:, 2]), "--", color=COL[meth], lw=1.3,
                        label=LABEL[meth] + (" (stopped at 6.3 s)" if path == "B" and meth == "exact" else ""))
        ax[row, 1].semilogy(t, np.maximum(deg(np.abs(Y[:, 2] - r["q"][:, 2])), 1e-14), COL[meth],
                            label=LABEL[meth])
    ax[row, 0].set_ylabel("θ0 [deg]")
    ax[row, 0].set_title(f"Path {path}: base attitude (thick: Python prediction, dashed: Simscape)")
    ax[row, 0].legend(fontsize=8)
    ax[row, 1].set_ylabel("|θ0 Simscape − Python| [deg]")
    ax[row, 1].set_title(f"Path {path}: difference")
    ax[row, 1].legend(fontsize=8)
for a in ax.flat:
    a.grid(alpha=0.3)
ax[1, 0].set_xlabel("time [s]"); ax[1, 1].set_xlabel("time [s]")
fig.suptitle("Project 1b – Reaction Null Space control on a Simscape Multibody plant")
fig.tight_layout()
out = os.path.join(HERE, "figures", "rns_simscape_vs_python.png")
os.makedirs(os.path.dirname(out), exist_ok=True)
fig.savefig(out, dpi=130)
print("saved", out)
