"""Project 3, step 3: closing the gripper on the tumbling target.

Start: the state of step 2 at t = 32 s (hand on the handle, tracking it).
At that instant the gripper closes. After the grasp the arm joints act as
dampers (tau = -D qd_m): they let the arm give way and absorb the relative motion.
Compared: compliant grasp with three gripper stiffnesses, and the rigid-impact limit.
"""
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp, cumulative_trapezoid

import contact as ct
import dynamics as d
import gen_jacobian as gj
import target as tg

sys.path.insert(0, gj.P1)
from step4_conservation import _P, _L     # noqa: E402  project 1: chaser momentum (exact)

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, "figures")
deg = np.degrees
T_GRASP, T_AFTER = 32.0, 30.0
D_JOINT, TAU_MAX = 5.0, 10.0

r2 = np.load(os.path.join(HERE, "results_step2.npz"))
k0 = int(np.argmin(np.abs(r2["t"] - T_GRASP)))
q0, qd0 = r2["q"][k0], r2["qd"][k0]
c, th, v, w = tg.state(r2["t"][k0])
xt0 = np.array([*c, th, *v, w])


def joint_torque(t, q, qd):
    return np.clip(-D_JOINT * qd[3:], -TAU_MAX, TAU_MAX)


def momentum(q, qd, xt):
    P = np.ravel(_P(q, qd)) + tg.M_T * xt[3:5]
    L = float(_L(q, qd)) + tg.M_T * (xt[0] * xt[4] - xt[1] * xt[3]) + tg.I_T * xt[5]
    return P, L


p_h, v_h, _ = ct.target_handle(xt0)
print(f"At grasp (t = {T_GRASP:.0f} s): hand-handle gap {1000*np.linalg.norm(p_h - gj.end_effector(q0)):.2f} mm, "
      f"relative speed {1000*np.linalg.norm(v_h - ct.hand_jacobian(q0) @ qd0):.2f} mm/s, "
      f"relative ROTATION rate {deg(xt0[5] - ct.JPHI2 @ qd0):.2f} deg/s")

# ---- rigid impact -------------------------------------------------------------
u_plus, imp, rel_before = ct.rigid_impact(q0, qd0, xt0)
xt_plus = np.concatenate([xt0[:3], u_plus[5:]])
print(f"\nRigid impact: impulse on the hand = [{imp[0]:.4f}, {imp[1]:.4f}] N s, {imp[2]:.4f} N m s")
print(f"  target spin {deg(xt0[5]):.3f} -> {deg(u_plus[7]):.3f} deg/s, "
      f"base rate {deg(qd0[2]):.3f} -> {deg(u_plus[2]):.3f} deg/s")
P0, L0 = momentum(q0, qd0, xt0)
P1, L1 = momentum(q0, u_plus[:5], xt_plus)
print(f"  momentum before/after: |dP| = {np.abs(P1 - P0).max():.1e}, |dL| = {abs(L1 - L0):.1e}")

# ---- compliant grasp, three stiffnesses ---------------------------------------
t = np.linspace(0, T_AFTER, 6001)
results = {}
print("\nCompliant grasp (joints damped, D = 5 N m s):")
for k in (1e2, 1e3, 1e4):
    grip = ct.Gripper(k)
    grip.d0 = xt0[2] - ct.JPHI2 @ q0                     # lock the current relative angle
    x0 = np.concatenate([q0, qd0, xt0])
    sol = solve_ivp(ct.coupled_rhs, (0, T_AFTER), x0, t_eval=t, args=(grip, joint_torque),
                    method="DOP853", rtol=1e-9, atol=1e-11, max_step=0.01)
    X = sol.y.T
    FG = np.array([grip.wrench(x[:5], x[5:10], x[10:]) for x in X], dtype=object)
    F = np.array([f[0] for f in FG], dtype=float); G = np.array([f[1] for f in FG], dtype=float)
    tau = np.array([joint_torque(0, x[:5], x[5:10]) for x in X])
    mom = [momentum(x[:5], x[5:10], x[10:]) for x in X]
    dP = max(np.abs(m_[0] - mom[0][0]).max() for m_ in mom)
    dL = max(abs(m_[1] - mom[0][1]) for m_ in mom)
    rel_rot = deg(X[:, 15] - X[:, 5:10] @ ct.JPHI2)
    results[k] = dict(X=X, F=F, G=G, tau=tau, rel_rot=rel_rot)
    print(f"  k = {k:7.0f} N/m: peak |F| {np.linalg.norm(F, axis=1).max():7.3f} N, peak |Gamma| "
          f"{np.abs(G).max():6.3f} N m, "
          f"max stretch {1000*np.max([np.linalg.norm(ct.target_handle(x[10:])[0]-gj.end_effector(x[:5])) for x in X]):.2f} mm")
    print(f"               momentum drift: dP {dP:.1e} kg m/s, dL {dL:.1e} kg m^2/s;  "
          f"target spin 1 s after grasp {deg(X[np.searchsorted(t, 1.0), 15]):.3f} deg/s")

# expected final state: chaser + target spin together as one rigid body
X = results[1e4]["X"][-1]
cm, I_sys, m_tot = ct.system_inertia(X[:5], X[10:])
P_end, L_end = momentum(X[:5], X[5:10], X[10:])
v_cm = P_end / m_tot
L_cm = L_end - m_tot * (cm[0] * v_cm[1] - cm[1] * v_cm[0])     # angular momentum about the CoM
print(f"\nRigid-body prediction for the final common spin: L_cm / I_sys = {L_cm:.4f} / {I_sys:.1f} "
      f"= {deg(L_cm / I_sys):.4f} deg/s")
print(f"\nAfter {T_AFTER:.0f} s (k = 1e4): target spin {deg(X[15]):.4f} deg/s, base rate {deg(X[7]):.4f} deg/s, "
      f"joint rates {deg(X[8:10]).round(4)} deg/s")
print(f"  total angular momentum is still L = {L_end:.4f} kg m^2/s (it was {L0:.4f}): "
      f"the arm cannot remove it, only share it")

# ---- figures ----------------------------------------------------------------------
fig, ax = plt.subplots(2, 2, figsize=(12, 7))
for (k, r), col in zip(results.items(), ("C0", "C1", "C3")):
    lab = f"k = {k:.0e} N/m"
    ax[0, 0].plot(t, np.linalg.norm(r["F"], axis=1), col, label=lab)
    ax[0, 1].plot(t, r["G"], col, label=lab)
    ax[1, 0].plot(t, r["rel_rot"], col, label=lab)
    ax[1, 1].plot(t, deg(r["X"][:, 15]), col, label=f"target spin, {lab}")
ax[1, 1].plot(t, deg(results[1e4]["X"][:, 7]), "k--", label="chaser base rate (k = 1e4)")
ax[1, 1].axhline(deg(u_plus[7]), color="gray", ls=":", label="rigid impact: target spin just after")
ax[0, 0].set_title("Grasp force |F|"); ax[0, 0].set_ylabel("N"); ax[0, 0].set_xlim(0, 10)
ax[0, 1].set_title("Grasp torque"); ax[0, 1].set_ylabel("N m"); ax[0, 1].set_xlim(0, 10)
ax[1, 0].set_title("Target spin relative to the gripper"); ax[1, 0].set_ylabel("deg/s"); ax[1, 0].set_xlim(0, 10)
ax[1, 1].set_title("Spin rates: the target slows, the chaser picks it up"); ax[1, 1].set_ylabel("deg/s")
for a in ax.flat:
    a.grid(alpha=0.3); a.legend(fontsize=7); a.set_xlabel("time after grasp [s]")
fig.tight_layout()
fig.savefig(os.path.join(FIG, "step3_grasp.png"), dpi=150)
print("saved", os.path.join(FIG, "step3_grasp.png"))
