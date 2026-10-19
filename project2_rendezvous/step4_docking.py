"""Step 4: full docking mission, translation + attitude together, nonlinear orbit.

Truth model : two-body orbital motion (ECI) + rigid-body attitude (EPs)
Coupling    : the thrusters are fixed to the chaser body. The LQR asks for an
              acceleration in LVLH; it is rotated into BODY axes and each body
              axis is limited to 4 N. So a wrong attitude limits/rotates the thrust.
Checks      : approach-cone corridor, contact conditions, delta-v.
"""
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp, cumulative_trapezoid

import attitude as att
import mission as m
import orbit as o
import translation_control as tc

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, "figures")
deg = np.degrees


def commands(t, X):
    """Thrust (ECI and body) and torque for the full state X = [r, v, beta, omega]."""
    beta, omega = X[6:10], X[10:13]
    rel = o.eci_to_lvlh(X[:6], t)
    s_ref, u_ff = m.translation_reference(t)
    a_L = u_ff - tc.K @ (rel - s_ref)                    # LQR in LVLH
    LN = o.eci_to_lvlh_dcm(*o.target_state(t))
    BN = att.ep2dcm(beta)
    a_B = np.clip(BN @ LN.T @ a_L, -tc.A_MAX, tc.A_MAX)  # body-fixed thrusters
    RN, w_r, w_r_dot = m.attitude_reference(t)
    torque = att.control_torque(beta, omega, RN, w_r, w_r_dot)
    return BN.T @ a_B, a_B, torque, rel


def rhs(t, X):
    a_N, _, torque, _ = commands(t, X)
    d = np.empty(13)
    d[:6] = o.two_body_rhs(t, X[:6])
    d[3:6] += a_N
    d[6:10], d[10:13] = att.rigid_body_rhs(X[6:10], X[10:13], torque)
    return d


def contact(t, X):
    return o.eci_to_lvlh(X[:6], t)[1] - m.Y_CONTACT
contact.terminal, contact.direction = True, 1


# ---- Initial state: translation as in step 2, attitude as in step 3 -----------
RN0 = m.RN_reference(0.0)
beta0 = att.dcm2ep(att.ep2dcm(att.prv2ep([0.3, -1, 0.5], np.radians(150))) @ RN0)
omega0 = np.radians([1.0, -0.5, 0.8])
X0 = np.concatenate([o.lvlh_to_eci(tc.S_TRUE0, 0.0), beta0, omega0])

T_MAX = m.T_CONTACT_PLAN + 200
t_eval = np.arange(0, T_MAX, 1.0)
sol = solve_ivp(rhs, (0, T_MAX), X0, t_eval=t_eval, events=contact,
                method="DOP853", rtol=1e-10, atol=1e-10, max_step=2.0)
assert sol.status == 1, "no contact happened!"
t = sol.t
X = sol.y.T
t_c, X_c = sol.t_events[0][0], sol.y_events[0][0]

# ---- Post-processing ---------------------------------------------------------
cmd = [commands(tk, Xk) for tk, Xk in zip(t, X)]
a_B = np.array([c[1] for c in cmd])
rel = np.array([c[3] for c in cmd])
s_ref = np.array([m.translation_reference(tk)[0] for tk in t])
point_err = np.array([att.error_angle(Xk[6:10], m.RN_reference(tk)) for tk, Xk in zip(t, X)])
dv = cumulative_trapezoid(np.abs(a_B).sum(axis=1), t, initial=0)


def port_geometry(tk, Xk):
    """Chaser docking-port position in LVLH (CoM + 0.75 m along body b1)."""
    r = o.eci_to_lvlh(Xk[:6], tk)
    LN = o.eci_to_lvlh_dcm(*o.target_state(tk))
    b1_L = LN @ att.ep2dcm(Xk[6:10])[0]
    return r[:3] + m.CHASER_PORT * b1_L, b1_L


ports = np.array([port_geometry(tk, Xk)[0] for tk, Xk in zip(t, X)])
axial = m.TARGET_PORT_Y - ports[:, 1]                    # distance to target port along -y
lateral = np.hypot(ports[:, 0], ports[:, 2])
inside = axial > 0.5
cone = np.arctan2(lateral[inside], axial[inside])

# Contact conditions
r_c = o.eci_to_lvlh(X_c[:6], t_c)
port_c, b1_c = port_geometry(t_c, X_c)
LN_c = o.eci_to_lvlh_dcm(*o.target_state(t_c))
dock_RN = np.vstack([[0, 1, 0], [-1, 0, 0], [0, 0, 1]]) @ LN_c   # ideal docked attitude
ang_c = att.error_angle(X_c[6:10], dock_RN)
w_rel = X_c[10:13] - att.ep2dcm(X_c[6:10]) @ np.array([0, 0, o.N])  # rate vs LVLH
checks = {
    "closing speed [m/s]":        (r_c[4], m.REQ["v_axial"][0] <= r_c[4] <= m.REQ["v_axial"][1], f"{m.REQ['v_axial'][0]}-{m.REQ['v_axial'][1]}"),
    "lateral offset [m]":         (np.hypot(port_c[0], port_c[2]), np.hypot(port_c[0], port_c[2]) <= m.REQ["lateral"], f"< {m.REQ['lateral']}"),
    "lateral speed [m/s]":        (np.hypot(r_c[3], r_c[5]), np.hypot(r_c[3], r_c[5]) <= m.REQ["v_lateral"], f"< {m.REQ['v_lateral']}"),
    "angular misalignment [deg]": (deg(ang_c), deg(ang_c) <= m.REQ["angle"], f"< {m.REQ['angle']}"),
    "angular rate [deg/s]":       (deg(np.linalg.norm(w_rel)), deg(np.linalg.norm(w_rel)) <= m.REQ["rate"], f"< {m.REQ['rate']}"),
}

print(f"Contact at t = {t_c:.1f} s (planned {m.T_CONTACT_PLAN:.0f} s)")
print("Contact conditions:")
for k, (val, ok, req) in checks.items():
    print(f"  {k:28s} {val:10.2e}   required {req:10s}  {'PASS' if ok else 'FAIL'}")
print(f"Approach cone (10 deg): max angle {deg(cone.max()):.2f} deg  -> "
      f"{'PASS' if cone.max() <= m.CONE_HALF_ANGLE else 'FAIL'}")
print(f"Pointing error: {deg(point_err[0]):.0f} deg at start, < 1 deg after "
      f"{t[np.where(deg(point_err) > 1)[0][-1] + 1]:.0f} s, max during final approach "
      f"{deg(point_err[t > m.T_HOLD_END].max()):.1e} deg")
print(f"Thrusters at limit: {100*np.mean(np.any(np.abs(a_B) >= tc.A_MAX*0.999, axis=1)):.1f} % of the time")
print(f"Total delta-v: {dv[-1]:.3f} m/s  (approach {dv[t <= 3000][-1]:.3f}, "
      f"hold {dv[t <= m.T_HOLD_END][-1] - dv[t <= 3000][-1]:.4f}, "
      f"final {dv[-1] - dv[t <= m.T_HOLD_END][-1]:.4f})")
np.savez(os.path.join(HERE, "results_step4.npz"), t=t, X=X, rel=rel, ports=ports,
         point_err=point_err, t_c=t_c)

# ---- Figure 1: mission overview -----------------------------------------------
fig, ax = plt.subplots(2, 2, figsize=(12, 7.5))
tm = t / 60
ax[0, 0].semilogy(tm, np.linalg.norm(rel[:, :3], axis=1))
ax[0, 0].set_ylabel("range to target [m]"); ax[0, 0].set_title("Range")
ax[0, 1].semilogy(tm, deg(point_err) + 1e-12)
ax[0, 1].set_ylabel("deg"); ax[0, 1].set_title("Pointing error (docking axis vs reference)")
for k, lab in enumerate(("b1", "b2", "b3")):
    ax[1, 0].plot(tm, a_B[:, k] * 1e3, f"C{k}", lw=1, label=f"thrust along {lab}")
ax[1, 0].axhline(tc.A_MAX * 1e3, color="k", ls=":", lw=1); ax[1, 0].axhline(-tc.A_MAX * 1e3, color="k", ls=":", lw=1)
ax[1, 0].set_ylabel("mm/s^2"); ax[1, 0].set_title("Thrust in body axes"); ax[1, 0].legend(fontsize=8)
ax[1, 0].set_ylim(-9, 9)
ax[1, 1].plot(tm, dv); ax[1, 1].set_ylabel("m/s"); ax[1, 1].set_title("Cumulative delta-v")
for a in ax.flat:
    a.grid(alpha=0.3, which="both")
    for tp in (3000, m.T_HOLD_END):
        a.axvline(tp / 60, color="gray", ls="--", lw=0.8)
for a in ax[1]:
    a.set_xlabel("time [min]   (dashed: start of hold, start of final approach)")
fig.tight_layout()
fig.savefig(os.path.join(FIG, "step4_mission_overview.png"), dpi=150)

# ---- Figure 2: final approach inside the cone ---------------------------------
sel = t >= 3000
fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
for a, k, lab in ((ax[0], 0, "x: radial"), (ax[1], 2, "z: cross-track")):
    yy = np.linspace(-12, m.TARGET_PORT_Y, 50)
    half = np.tan(m.CONE_HALF_ANGLE) * (m.TARGET_PORT_Y - yy) * 100     # [cm]
    a.fill_between(yy, -half, half, color="C2", alpha=0.15, label="10 deg approach cone")
    a.plot(ports[sel, 1], ports[sel, k] * 100, "C0", lw=2, label="chaser docking port")
    a.plot(port_c[1], port_c[k] * 100, "ro", label="contact")
    a.axvline(m.TARGET_PORT_Y, color="k", lw=2)
    a.set_xlabel("y: along-track [m]   (target port at y = -1 m)")
    a.set_ylabel(f"{lab} [cm]"); a.set_ylim(-60, 60)
    a.grid(alpha=0.3); a.legend(fontsize=8, loc="lower left")
ax[0].set_title("Final approach, in-plane"); ax[1].set_title("Final approach, out-of-plane")
fig.tight_layout()
fig.savefig(os.path.join(FIG, "step4_final_approach.png"), dpi=150)
print("saved figures to", FIG)
