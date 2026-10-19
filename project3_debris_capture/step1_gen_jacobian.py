"""Project 3, step 1: generalized Jacobian of the free-floating robot.

A. Verify v_e = J* qd_m on the full dynamic simulation of project 1.
B. Compare how "capable" the arm is in space vs on the ground (manipulability maps).
C. Why it matters for capture: drive the end effector along a straight line using
   the fixed-base Jacobian (wrong in space) and the generalized Jacobian (right).
D. Dynamic singularity: a straight line that a fixed-base arm could follow, but the
   free-floating arm cannot (Papadopoulos & Dubowsky): J* becomes singular while
   the fixed-base J is not.
"""
import os

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp

import gen_jacobian as gj
import simulation as sim1          # project 1 (found through gen_jacobian's sys.path)

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, "figures")
os.makedirs(FIG, exist_ok=True)
deg = np.degrees

# ---------------- A. verification on project 1's dynamic simulation -------------
t, q, qd, _ = sim1.simulate()
err_v = err_b = 0.0
for qk, qdk in zip(q, qd):
    Jb, Jm = gj.jacobians(qk)
    v_true = Jb @ qdk[:3] + Jm @ qdk[3:]              # chain rule, all 5 coordinates
    err_v = max(err_v, np.abs(v_true - gj.generalized_jacobian(qk) @ qdk[3:]).max())
    err_b = max(err_b, np.abs(qdk[:3] - gj.base_coupling(qk) @ qdk[3:]).max())
ee = np.array([gj.end_effector(qk) for qk in q])
v_fd = np.gradient(ee, t, axis=0)                      # independent: finite differences
v_gj = np.array([gj.generalized_jacobian(qk) @ qdk[3:] for qk, qdk in zip(q, qd)])
print("A. Verification on the project 1 simulation (15 s, PD-controlled arm)")
print(f"   max |base velocity - (-Mbb^-1 Mbm qd_m)|     = {err_b:.1e}")
print(f"   max |v_e (all coordinates) - J* qd_m|          = {err_v:.1e} m/s")
print(f"   max |v_e (finite differences) - J* qd_m|       = {np.abs(v_fd[2:-2] - v_gj[2:-2]).max():.1e} m/s"
      f"   (peak |v_e| = {np.linalg.norm(v_gj, axis=1).max():.3f} m/s)")

q_demo = np.radians([0, 0, 0, 45, -90])
q_demo[:2] = 0
print(f"\n   at q1 = 45 deg, q2 = -90 deg:")
print("   fixed-base J =\n", gj.fixed_base_jacobian(q_demo).round(3))
print("   generalized J* =\n", gj.generalized_jacobian(q_demo).round(3))

# ---------------- B. manipulability maps ----------------------------------------
q1s = np.radians(np.linspace(-180, 180, 181))
q2s = np.radians(np.linspace(-180, 180, 181))
W_fixed = np.zeros((len(q2s), len(q1s)))
W_free = np.zeros_like(W_fixed)
for i, a2 in enumerate(q2s):
    for j, a1 in enumerate(q1s):
        qq = np.array([0, 0, 0, a1, a2])
        W_fixed[i, j] = gj.manipulability(gj.fixed_base_jacobian(qq))
        W_free[i, j] = gj.manipulability(gj.generalized_jacobian(qq))
lost = np.mean(W_free < W_fixed)
print(f"\nB. Manipulability: lower in space than on the ground in {100*lost:.0f} % of joint space; "
      f"mean {W_free.mean():.2f} vs {W_fixed.mean():.2f}")

fig, ax = plt.subplots(1, 3, figsize=(15, 4.3))
ext = [-180, 180, -180, 180]
for a, W, ttl in ((ax[0], W_fixed, "Fixed base (on the ground)"), (ax[1], W_free, "Free-floating (in space)")):
    im = a.imshow(W, origin="lower", extent=ext, cmap="viridis", vmin=0, vmax=W_fixed.max())
    a.set_title(f"Manipulability: {ttl}"); fig.colorbar(im, ax=a, shrink=0.85)
Q1, Q2 = np.meshgrid(deg(q1s), deg(q2s))
ax[2].contour(Q1, Q2, W_fixed, levels=[0.03], colors="C3", linewidths=2)
ax[2].contour(Q1, Q2, W_free, levels=[0.03], colors="C0", linewidths=2)
ax[2].plot([], [], "C3", lw=2, label="fixed base: arm straight (q2 = 0, 180)")
ax[2].plot([], [], "C0", lw=2, label="free-floating: dynamic singularities")
ax[2].set_xlim(-180, 180); ax[2].set_ylim(-180, 180); ax[2].grid(alpha=0.3)
ax[2].legend(fontsize=8, loc="center", bbox_to_anchor=(0.5, 0.3))
ax[2].set_title("Near-singular bands (manipulability < 0.03)")
for a in ax:
    a.set_xlabel("q1 [deg]"); a.set_ylabel("q2 [deg]")
fig.tight_layout()
fig.savefig(os.path.join(FIG, "step1_manipulability.png"), dpi=150)

# ---------------- C. straight-line end-effector motion ---------------------------
q0 = np.radians([0, 0, 0, 30, 100])
q0[:2] = 0
p0 = gj.end_effector(q0)
DP = np.array([-0.4, 0.3])         # move 50 cm up-left
T = 20.0


def path(t, DP=DP):
    x = np.clip(t / T, 0, 1)
    s = 10 * x**3 - 15 * x**4 + 6 * x**5
    ds = (30 * x**2 - 60 * x**3 + 30 * x**4) / T if t < T else 0.0
    return p0 + s * DP, ds * DP


def kinematic_rhs(t, qq, jac, DP=DP):
    """Zero-momentum motion: joints follow J^-1 v_des (open loop), base reacts."""
    _, v_des = path(t, DP)
    qd_m = np.linalg.solve(jac(qq), v_des)
    return np.concatenate([gj.base_coupling(qq) @ qd_m, qd_m])


tt = np.linspace(0, T, 801)
res = {}
print("\nC. Straight line of 50 cm, open loop (joint rates from the Jacobian only):")
for name, jac in (("fixed-base J (ground model)", gj.fixed_base_jacobian),
                  ("generalized J* (space model)", gj.generalized_jacobian)):
    s = solve_ivp(kinematic_rhs, (0, T), q0, t_eval=tt, args=(jac,), method="DOP853", rtol=1e-10, atol=1e-12)
    e = np.array([gj.end_effector(qk) for qk in s.y.T])
    want = np.array([path(tk)[0] for tk in tt])
    res[name] = (s.y.T, e)
    print(f"   {name:30s}: max path error {100*np.linalg.norm(e - want, axis=1).max():6.2f} cm, "
          f"final error {100*np.linalg.norm(e[-1] - want[-1]):6.2f} cm, "
          f"base rotation {deg(s.y[2, -1]):6.2f} deg, "
          f"min manipulability {min(gj.manipulability(gj.generalized_jacobian(qk)) for qk in s.y.T):.3f}")

# ---------------- D. dynamic singularity ------------------------------------------
DP_B = np.array([0.0, 0.6])        # 60 cm straight up from the same start


def near_singular(t, qq, jac, DP):
    return gj.manipulability(gj.generalized_jacobian(qq)) - 0.01
near_singular.terminal = True


sB = solve_ivp(kinematic_rhs, (0, T), q0, args=(gj.generalized_jacobian, DP_B), events=near_singular,
               method="DOP853", rtol=1e-10, atol=1e-12, max_step=0.05)
qs_B = sB.y.T
w_free_B = np.array([gj.manipulability(gj.generalized_jacobian(qk)) for qk in qs_B])
w_fix_B = np.array([gj.manipulability(gj.fixed_base_jacobian(qk)) for qk in qs_B])
eB = np.array([gj.end_effector(qk) for qk in qs_B])
print(f"\nD. Straight line of 60 cm upwards with J*: stopped at t = {sB.t[-1]:.2f} s of {T:.0f} s "
      f"after {100*np.linalg.norm(eB[-1]-p0):.1f} cm")
print(f"   there: q2 = {deg(qs_B[-1, 4]):.1f} deg, manipulability J* = {w_free_B[-1]:.3f}, "
      f"fixed-base J = {w_fix_B[-1]:.3f}  -> DYNAMIC singularity (arm not straight)")
shoulder0 = np.array([0.5, 0.0])   # shoulder position at the start (base at origin, th0 = 0)
reach_fixed = np.linalg.norm(p0 + DP_B - shoulder0)
print(f"   the goal is {reach_fixed:.2f} m from the shoulder (fixed-base reach 2.00 m): a ground arm could do it")

from step1_helpers import draw  # noqa: E402

fig, ax = plt.subplots(1, 3, figsize=(17, 5))
draw(ax[0], res["fixed-base J (ground model)"][0][0], "0.5", alpha=0.5)       # common start
for (name, (qs, e)), c in zip(res.items(), ("C3", "C0")):
    draw(ax[0], qs[-1], c, alpha=0.6)
    ax[1].plot(tt, deg(qs[:, 2]), c, lw=2, label=name)
want = np.array([path(tk)[0] for tk in tt])
ax[0].plot(want[:, 0], want[:, 1], "k--", lw=2, label="desired straight line")
for (name, (qs, e)), c in zip(res.items(), ("C3", "C0")):
    ax[0].plot(e[:, 0], e[:, 1], c, lw=3.5, label=f"end effector, {name}")
    ax[0].plot(*e[-1], "o", color=c, ms=9)
ax[0].plot(*p0, "ks", ms=8, label="start")
ax[0].set_aspect("equal"); ax[0].grid(alpha=0.3); ax[0].legend(fontsize=8, loc="lower right")
ax[0].set_xlabel("x [m]"); ax[0].set_ylabel("y [m]")
ax[0].set_title("C: end-effector path (grey: start pose)")
ax[1].set_xlabel("time [s]"); ax[1].set_ylabel("base attitude [deg]"); ax[1].grid(alpha=0.3)
ax[1].set_title("C: base rotation caused by the arm"); ax[1].legend(fontsize=8)
ax[2].plot(sB.t, w_fix_B, "C3", lw=2, label="fixed-base J")
ax[2].plot(sB.t, w_free_B, "C0", lw=2, label="generalized J*")
ax[2].axvline(sB.t[-1], color="k", ls=":", lw=1)
ax[2].text(sB.t[-1], 0.9 * w_fix_B.max(), "  J* singular:\n  motion impossible", fontsize=9, va="top")
ax[2].set_xlabel("time [s]"); ax[2].set_ylabel("manipulability"); ax[2].grid(alpha=0.3)
ax[2].set_title("D: dynamic singularity on a 60 cm straight line up"); ax[2].legend(fontsize=8)
ax[2].set_xlim(0, T)
fig.tight_layout()
fig.savefig(os.path.join(FIG, "step1_straight_line.png"), dpi=150)
print("saved figures to", FIG)
