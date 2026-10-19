"""Step 3: chaser attitude with Euler parameters.

Part A - torque-free tumbling: verify the attitude model
   * |beta| stays 1, kinetic energy and inertial angular momentum stay constant
   * the EP solution matches an independent DCM integration ([BN]' = -[w~][BN])
Part B - closed-loop pointing: the chaser is at the V-bar hold point (10 m behind
   the target) and must point its docking port (body axis b1) at the target.
   It starts tumbling with a 150 deg attitude error.
"""
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

import attitude as att
import orbit as o

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, "figures")
TOL = dict(method="DOP853", rtol=1e-11, atol=1e-12)
deg = np.degrees

# ============================ Part A ==========================================
beta0 = att.prv2ep([1, 2, -1], np.radians(40))
omega0 = np.radians([3.0, -2.0, 5.0])                 # deg/s -> rad/s


def free_rhs(t, x):
    beta, omega, C = x[:4], x[4:7], x[7:].reshape(3, 3)
    bd, wd = att.rigid_body_rhs(beta, omega, np.zeros(3))
    Cd = -att.tilde(omega) @ C                         # DCM kinematics
    return np.concatenate([bd, wd, Cd.ravel()])


tA = np.linspace(0, 600, 3001)
x0 = np.concatenate([beta0, omega0, att.ep2dcm(beta0).ravel()])
solA = solve_ivp(free_rhs, (0, tA[-1]), x0, t_eval=tA, **TOL)
beta, omega, C = solA.y[:4].T, solA.y[4:7].T, solA.y[7:].T.reshape(-1, 3, 3)

norm_err = np.abs(np.linalg.norm(beta, axis=1) - 1).max()
T = 0.5 * np.einsum("ki,ij,kj->k", omega, att.J, omega)
H_N = np.array([att.ep2dcm(b).T @ (att.J @ w) for b, w in zip(beta, omega)])   # H in N frame
dcm_err = max(np.abs(att.ep2dcm(b) - Ck).max() for b, Ck in zip(beta, C))
print("Part A: torque-free tumbling for 600 s")
print(f"  max | |beta| - 1 |                 = {norm_err:.1e}")
print(f"  max |T(t) - T(0)| / T(0)           = {np.abs(T - T[0]).max() / T[0]:.1e}")
print(f"  max |H_N(t) - H_N(0)| / |H|        = {np.abs(H_N - H_N[0]).max() / np.linalg.norm(H_N[0]):.1e}")
print(f"  max |[BN] from EPs - [BN] from DCM eq| = {dcm_err:.1e}")

# ============================ Part B ==========================================
# Reference frame R at the V-bar hold point (target is straight ahead, +y LVLH):
#   r1 = +y_LVLH (docking axis points at the target), r3 = orbit normal, r2 = r3 x r1
# LVLH rotates at n about the orbit normal, so omega_r = n z_N, omega_r_dot = 0.
def reference(t):
    rt, vt = o.target_state(t)
    LN = o.eci_to_lvlh_dcm(rt, vt)                    # rows: x, y, z of LVLH in N
    r1, r3 = LN[1], LN[2]
    RN = np.vstack([r1, np.cross(r3, r1), r3])
    return RN, np.array([0, 0, o.N]), np.zeros(3)


def closed_rhs(t, x, saturate=True):
    beta, omega = x[:4], x[4:]
    RN, w_r, w_r_dot = reference(t)
    u = att.control_torque(beta, omega, RN, w_r, w_r_dot, saturate)
    bd, wd = att.rigid_body_rhs(beta, omega, u)
    return np.concatenate([bd, wd])


RN0 = reference(0)[0]
beta0 = att.dcm2ep(att.ep2dcm(att.prv2ep([0.3, -1, 0.5], np.radians(150))) @ RN0)  # 150 deg off
omega0 = np.radians([1.0, -0.5, 0.8])                 # tumbling ~1.4 deg/s
tB = np.linspace(0, 600, 3001)
print(f"\nPart B: pointing control, K = {att.K_GAIN:.3f} N m, P = {att.P_GAIN:.2f} N m s, "
      f"torque limit {att.U_MAX} N m/axis")
results = {}
for label, sat in (("with torque limit", True), ("no limit (ideal)", False)):
    sol = solve_ivp(lambda t, x: closed_rhs(t, x, sat), (0, tB[-1]),
                    np.concatenate([beta0, omega0]), t_eval=tB, **TOL)
    be, om = sol.y[:4].T, sol.y[4:].T
    ang = np.array([att.error_angle(b, reference(t)[0]) for t, b in zip(tB, be)])
    u = np.array([att.control_torque(b, w, *reference(t), saturate=sat) for t, b, w in zip(tB, be, om)])
    V = np.array([att.lyapunov(b, w, reference(t)[0], reference(t)[1]) for t, b, w in zip(tB, be, om)])
    dw = np.array([w - att.ep2dcm(b) @ reference(t)[1] for t, b, w in zip(tB, be, om)])
    t_settle = tB[np.where(deg(ang) > 0.1)[0][-1] + 1]
    results[label] = dict(ang=ang, u=u, V=V, dw=dw, sat=sat)
    print(f"  {label:18s}: error {deg(ang[0]):.0f} deg -> {deg(ang[-1]):.1e} deg, "
          f"< 0.1 deg after {t_settle:.0f} s, peak torque {np.abs(u).max():.2f} N m, "
          f"final rate error {np.linalg.norm(dw[-1]):.1e} rad/s, "
          f"V ever increases? {'yes' if np.any(np.diff(V) > 1e-9) else 'no'}")
    if sat:
        print(f"  {'':18s}  time at torque limit: {100*np.mean(np.any(np.abs(u) >= att.U_MAX*0.999, axis=1)):.0f} %")
    if sat:
        b_final = be[-1]
        b1_N = att.ep2dcm(b_final)[0]                 # body axis b1 in N components
        to_target_N = o.eci_to_lvlh_dcm(*o.target_state(tB[-1]))[1]
        print(f"  docking axis vs line-of-sight angle at t = 600 s: "
              f"{deg(np.arctan2(np.linalg.norm(np.cross(b1_N, to_target_N)), b1_N @ to_target_N)):.1e} deg")
        print(f"  |beta| drift: {np.abs(np.linalg.norm(be, axis=1) - 1).max():.1e}")

# ---- Figures --------------------------------------------------------------------
fig, ax = plt.subplots(2, 2, figsize=(12, 7))
tm = tA / 60
ax[0, 0].plot(tA, beta); ax[0, 0].set_title("A: torque-free tumbling, Euler parameters")
ax[0, 0].legend([r"$\beta_0$", r"$\beta_1$", r"$\beta_2$", r"$\beta_3$"], fontsize=8, ncol=4)
ax[0, 0].set_xlabel("time [s]")
ax[0, 1].semilogy(tA, np.abs(np.linalg.norm(beta, axis=1) - 1) + 1e-17, label="| |beta| - 1 |")
ax[0, 1].semilogy(tA, np.abs(T - T[0]) / T[0] + 1e-17, label="energy drift (rel.)")
ax[0, 1].semilogy(tA, np.linalg.norm(H_N - H_N[0], axis=1) / np.linalg.norm(H_N[0]) + 1e-17, label="momentum drift (rel.)")
ax[0, 1].set_title("A: conserved quantities"); ax[0, 1].legend(fontsize=8); ax[0, 1].set_xlabel("time [s]")
for label, c in zip(results, ("C3", "C0")):
    r = results[label]
    ax[1, 0].semilogy(tB, deg(r["ang"]) + 1e-12, c, label=label)
ax[1, 0].axhline(0.1, color="k", ls=":", lw=1, label="0.1 deg")
ax[1, 0].set_title("B: attitude error angle"); ax[1, 0].set_ylabel("deg"); ax[1, 0].set_xlabel("time [s]")
ax[1, 0].legend(fontsize=8)
r = results["with torque limit"]
for k in range(3):
    ax[1, 1].plot(tB, r["u"][:, k], f"C{k}", label=f"u{k+1}")
ax[1, 1].axhline(att.U_MAX, color="k", ls=":", lw=1); ax[1, 1].axhline(-att.U_MAX, color="k", ls=":", lw=1)
ax[1, 1].set_xlim(0, 400); ax[1, 1].set_title("B: control torque (with limit)")
ax[1, 1].set_ylabel("N m"); ax[1, 1].set_xlabel("time [s]"); ax[1, 1].legend(fontsize=8)
for a in ax.flat:
    a.grid(alpha=0.3, which="both")
fig.tight_layout()
fig.savefig(os.path.join(FIG, "step3_attitude.png"), dpi=150)
print("saved", os.path.join(FIG, "step3_attitude.png"))
