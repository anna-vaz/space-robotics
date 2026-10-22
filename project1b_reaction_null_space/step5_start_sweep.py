"""Step 5a: does the starting arm configuration decide whether the base must rotate?

Path B (0.6 m to the left) always starts from the SAME end-effector point.  The arm
is redundant, so there is a one-parameter family of start configurations reaching
that point: choose the absolute wrist angle phi3, then solve the 2-link inverse
kinematics for q1, q2 (elbow on the same side as the nominal start).
For each start we run the three methods of step 3 (kinematic, zero momentum).
"""
import json
import os

import matplotlib.pyplot as plt
import numpy as np

import params as p
import rns
from kinematics import ik_with_wrist_angle
from model import build
from path_tracking import straight_line, track

HERE = os.path.dirname(os.path.abspath(__file__))
rob = build(p)
q_nom = np.array([0, 0, 0, np.radians(30), np.radians(60), np.radians(-45)])
e0 = rob.end_effector(q_nom)
ref = straight_line(e0, e0 + [-0.6, 0.0], 8.0)


def start_config(phi3):
    """Joint angles reaching e0 with the wrist (link 3) at absolute angle phi3."""
    return ik_with_wrist_angle(e0, phi3, p.LENGTHS, p.b0)


phi_nom = q_nom[3:].sum()
phis = np.radians(np.arange(-180, 180, 5.0))
rows = []
for phi in phis:
    q0 = start_config(phi)
    if q0 is None:
        continue
    assert np.allclose(rob.end_effector(q0), e0, atol=1e-12)
    row = dict(phi3=float(np.degrees(phi)), detA0=float(np.linalg.det(rns.extended_jacobian(rob, q0))))
    for m in ("minnorm", "exact", "priority"):
        r = track(rob, q0, ref, 10.0, m, n_points=201)
        smin = min(np.linalg.svd(rns.generalized_jacobian(rob, q), compute_uv=False)[-1] for q in r["q"])
        row[m] = dict(max_th0=float(np.degrees(np.abs(r["q"][:, 2]).max())),
                      end_th0=float(np.degrees(r["q"][-1, 2])), max_rate=float(np.abs(r["qd_m"]).max()),
                      min_detA=float(np.abs(r["detA"]).min()), stopped=bool(r["stopped"]), smin=float(smin))
    rows.append(row)
    pr, ex = row["priority"], row["exact"]
    print(f"phi3 = {row['phi3']:+6.1f} deg  det A(0) = {row['detA0']:+.3f}  "
          f"min-norm {row['minnorm']['max_th0']:6.2f} deg | exact {'FAILS' if ex['stopped'] else 'ok   '} | "
          f"priority max {pr['max_th0']:6.3f} deg, end {pr['end_th0']:+7.3f} deg, rate {pr['max_rate']:.2f},"
          f" min|det A| {pr['min_detA']:.3f}, smin(J*) {pr['smin']:.3f}")

with open(os.path.join(HERE, "results_step5_sweep.json"), "w") as f:
    json.dump(rows, f, indent=1)

ok = [r for r in rows if not r["minnorm"]["stopped"] and r["priority"]["smin"] > 0.05]
n_exact = sum(not r["exact"]["stopped"] for r in ok)
best = min(ok, key=lambda r: r["priority"]["max_th0"])
print(f"\nreachable starts: {len(rows)}, path feasible for the arm (smin(J*) > 0.05): {len(ok)}")
print(f"exact RNS completes from {n_exact}/{len(ok)} starts")
print(f"nominal start phi3 = {np.degrees(phi_nom):.0f} deg; best start phi3 = {best['phi3']:.0f} deg "
      f"(priority max |th0| = {best['priority']['max_th0']:.4f} deg)")
print(f"min-norm max |th0| over feasible starts: {min(r['minnorm']['max_th0'] for r in ok):.2f} .. "
      f"{max(r['minnorm']['max_th0'] for r in ok):.2f} deg")

# ---------------------------------------------------------------- figure
x = np.array([r["phi3"] for r in ok])
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
ax1.semilogy(x, [r["minnorm"]["max_th0"] for r in ok], "o-", color="C3", ms=4, label="min-norm (base ignored)")
ax1.semilogy(x, [r["priority"]["max_th0"] for r in ok], "o-", color="C2", ms=4,
             label="task priority + damped RNS")
xe = [r["phi3"] for r in ok if r["exact"]["stopped"]]
ax1.plot(xe, [3e-3] * len(xe), "x", color="C0", ms=7, mew=2, label="exact RNS fails (det A -> 0)")
ax1.axvline(np.degrees(phi_nom), color="k", ls=":", lw=1)
ax1.text(np.degrees(phi_nom) + 2, 1e-2, "nominal start", fontsize=8)
ax1.set_ylabel(r"max base rotation $|\theta_0|$ [deg]"); ax1.set_ylim(1e-3, 100)
ax1.legend(fontsize=8, loc="center left"); ax1.grid(alpha=0.3, which="both")
ax1.set_title("Path B from the same end-effector point, different starting arm configurations")
ax2.plot(x, [r["detA0"] for r in ok], "o-", color="C1", ms=4, label="det A at the start")
ax2.plot(x, [r["priority"]["min_detA"] for r in ok], "o-", color="C2", ms=4,
         label="min |det A| along the path (task priority)")
ax2.axhline(0, color="k", lw=0.8); ax2.axvline(np.degrees(phi_nom), color="k", ls=":", lw=1)
ax2.set_xlabel(r"initial absolute wrist angle $\varphi_3$ [deg]"); ax2.set_ylabel(r"[m$^2$]")
ax2.legend(fontsize=8); ax2.grid(alpha=0.3)
fig.tight_layout()
out = os.path.join(HERE, "figures", "step5_start_sweep.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
print("saved", out)
