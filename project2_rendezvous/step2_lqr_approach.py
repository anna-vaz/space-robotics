"""Step 2: LQR approach along the V-bar, tested on the CW model and the full
nonlinear orbit model, with thruster limits and delta-v (fuel) bookkeeping."""
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp, cumulative_trapezoid

import orbit as o
import translation_control as tc

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, "figures")
TOL = dict(method="DOP853", rtol=1e-10, atol=1e-10, max_step=5.0)
t = np.linspace(0, tc.T_END, 4001)

# ---- Controller design summary -----------------------------------------------
A, B = o.cw_matrices()
eig = np.linalg.eigvals(A - B @ tc.K)
print(f"Chaser: {tc.MASS:.0f} kg, {tc.F_MAX:.0f} N per axis -> a_max = {tc.A_MAX*1e3:.1f} mm/s^2")
print("Open-loop CW eigenvalues  :", np.round(np.linalg.eigvals(A), 6))
print("Closed-loop eigenvalues   :", np.round(eig, 4))
print(f"Slowest closed-loop time constant: {1/np.abs(eig.real).min():.0f} s")

# ---- 1. Simulate on the CW model ----------------------------------------------
cw = solve_ivp(lambda tt, s: o.cw_rhs(tt, s, accel=tc.control), (0, tc.T_END),
               tc.S_TRUE0, t_eval=t, **TOL)


# ---- 2. Simulate on the nonlinear two-body model ----------------------------
#      (controller still uses the CW-based gains, fed with the true LVLH state)
def nonlinear_rhs(tt, chaser_eci):
    rel = o.eci_to_lvlh(chaser_eci, tt)
    u_lvlh = tc.control(tt, rel)
    rt, vt = o.target_state(tt)
    C = o.eci_to_lvlh_dcm(rt, vt)
    d = o.two_body_rhs(tt, chaser_eci)
    d[3:] += C.T @ u_lvlh                     # thrust, rotated LVLH -> ECI
    return d


nl_sol = solve_ivp(nonlinear_rhs, (0, tc.T_END), o.lvlh_to_eci(tc.S_TRUE0, 0.0),
                   t_eval=t, **TOL)
nl = np.array([o.eci_to_lvlh(nl_sol.y[:, k], tk) for k, tk in enumerate(t)]).T


# ---- Post-processing ---------------------------------------------------------
def analyse(name, S):
    u = np.array([tc.control(tk, S[:, k]) for k, tk in enumerate(t)])
    s_ref = np.array([tc.reference(tk)[0] for tk in t]).T
    err = np.linalg.norm(S[:3] - s_ref[:3], axis=0)
    dv_axes = cumulative_trapezoid(np.abs(u).sum(axis=1), t, initial=0)   # 3 axis thrusters
    sat = np.mean(np.any(np.abs(u) >= tc.A_MAX * 0.999, axis=1)) * 100
    final = S[:, -1]
    print(f"\n--- {name} ---")
    print(f"  final position  : {final[:3].round(3)} m   (hold point [0, {tc.Y_HOLD}, 0])")
    print(f"  final velocity  : {(final[3:]*1000).round(3)} mm/s")
    print(f"  initial error   : {err[0]:.2f} m -> below 0.1 m after {t[np.argmax(err < 0.1)]:.0f} s")
    print(f"  tracking err after 300 s: max {err[t > 300].max():.1e} m,  during hold: {err[t > tc.T_APPROACH].max():.1e} m")
    print(f"  peak thrust     : {(np.abs(u).max(axis=0) / tc.A_MAX * 100).round(1)} % of max per axis")
    print(f"  time saturated  : {sat:.1f} %")
    print(f"  total delta-v   : {dv_axes[-1]:.3f} m/s")
    return dict(u=u, s_ref=s_ref, err=err, dv=dv_axes)


r_cw = analyse("CW model", cw.y)
r_nl = analyse("Nonlinear two-body model", nl)
print(f"\nCW vs nonlinear trajectory difference: max {np.linalg.norm(cw.y[:3]-nl[:3], axis=0).max():.1e} m")
# Delta-v of the reference path itself (hand check):
#  radial: holding the V-bar while moving at y' needs a_x = -2 n y'  ->  2 n * 990 m
#  along-track: speed up then slow down  ->  2 * peak speed = 2 * 1.875 * 990 / T
n = o.N
print(f"Hand estimate: radial 2*n*990 = {2*n*990:.2f} m/s, along-track 2*1.875*990/T = {2*1.875*990/tc.T_APPROACH:.2f} m/s")

# ---- Figures -----------------------------------------------------------------
fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
ax[0].plot(cw.y[1], cw.y[0], "C0", lw=2, label="CW model")
ax[0].plot(nl[1], nl[0], "C3--", lw=1.5, label="nonlinear model")
ax[0].plot(r_cw["s_ref"][1], r_cw["s_ref"][0], "k:", lw=1, label="reference")
ax[0].plot(tc.S_TRUE0[1], tc.S_TRUE0[0], "ko"); ax[0].plot(0, 0, "k*", ms=14, label="target")
ax[0].set_xlabel("y: along-track [m]"); ax[0].set_ylabel("x: radial [m]")
ax[0].set_title("In-plane approach (V-bar)")
ax[1].plot(cw.y[1], cw.y[2], "C0", lw=2, label="CW model")
ax[1].plot(nl[1], nl[2], "C3--", lw=1.5, label="nonlinear model")
ax[1].plot(0, 0, "k*", ms=14)
ax[1].set_xlabel("y: along-track [m]"); ax[1].set_ylabel("z: cross-track [m]")
ax[1].set_title("Out-of-plane")
for a in ax:
    a.grid(alpha=0.3); a.legend(fontsize=8)
fig.tight_layout()
fig.savefig(os.path.join(FIG, "step2_approach_trajectory.png"), dpi=150)

fig, ax = plt.subplots(2, 2, figsize=(12, 7), sharex=True)
tm = t / 60
ax[0, 0].plot(tm, -cw.y[1], "C0", label="CW"); ax[0, 0].plot(tm, -nl[1], "C3--", label="nonlinear")
ax[0, 0].set_ylabel("distance behind target, -y [m]"); ax[0, 0].set_yscale("log")
ax[0, 0].set_title("Range to target")
ax[0, 1].semilogy(tm, r_cw["err"] + 1e-9, "C0", label="CW"); ax[0, 1].semilogy(tm, r_nl["err"] + 1e-9, "C3--", label="nonlinear")
ax[0, 1].set_ylabel("|position - reference| [m]"); ax[0, 1].set_title("Tracking error")
for k, lab in enumerate("xyz"):
    ax[1, 0].plot(tm, r_nl["u"][:, k] * 1e3, f"C{k}", label=f"a_{lab}")
ax[1, 0].axhline(tc.A_MAX * 1e3, color="k", ls=":", lw=1); ax[1, 0].axhline(-tc.A_MAX * 1e3, color="k", ls=":", lw=1, label="thruster limit")
ax[1, 0].set_ylabel("thrust accel [mm/s^2]"); ax[1, 0].set_title("Commanded thrust (nonlinear run)")
ax[1, 1].plot(tm, r_cw["dv"], "C0", label="CW"); ax[1, 1].plot(tm, r_nl["dv"], "C3--", label="nonlinear")
ax[1, 1].set_ylabel("cumulative delta-v [m/s]"); ax[1, 1].set_title("Fuel use")
for a in ax.flat:
    a.grid(alpha=0.3, which="both"); a.legend(fontsize=8)
    a.axvline(tc.T_APPROACH / 60, color="gray", ls="--", lw=0.8)
for a in ax[1]:
    a.set_xlabel("time [min]   (dashed line: start of hold)")
fig.tight_layout()
fig.savefig(os.path.join(FIG, "step2_time_histories.png"), dpi=150)
print("saved figures to", FIG)
