"""Step 1: relative orbital motion near the target (Clohessy-Wiltshire).

Part A - four classic "free" (no thrust) motions, using the CW model.
Part B - checks:
   1. numerical CW integration vs the closed-form CW solution (STM)
   2. CW (linear) vs full nonlinear two-body motion, for growing separations
"""
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

import orbit as o

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, "figures")
os.makedirs(FIG, exist_ok=True)
n = o.N
TOL = dict(method="DOP853", rtol=1e-12, atol=1e-12)

print(f"Target orbit: altitude {o.ALT/1e3:.0f} km, period {o.PERIOD/60:.2f} min, "
      f"n = {n:.6e} rad/s")

# ---- Part A: classic natural motions (state = [x, y, z, vx, vy, vz]) ----------
cases = {
    "V-bar hold (100 m behind)":   [0, -100, 0, 0, 0, 0],
    "Lower orbit drift (50 m below)": [-50, -400, 0, 0, -1.5 * n * -50, 0],
    "Safety ellipse (closed)":     [50, -300, 0, 0, -2 * n * 50, 0],
    "Thrust TOWARD target (+0.1 m/s)": [0, -200, 0, 0, 0.1, 0],
}
t_end = 2 * o.PERIOD
t = np.linspace(0, t_end, 2001)

fig, axes = plt.subplots(2, 2, figsize=(12, 8))
for (name, s0), c, ax in zip(cases.items(), ["C0", "C1", "C2", "C3"], axes.flat):
    sol = solve_ivp(o.cw_rhs, (0, t_end), s0, t_eval=t, **TOL)
    x, y = sol.y[0], sol.y[1]
    ax.plot(y, x, c, lw=2)
    ax.plot(y[0], x[0], "o", color=c, ms=8, label="start")
    ax.plot(y[-1], x[-1], "s", color=c, mfc="none", ms=10, label="after 2 orbits")
    ax.plot(0, 0, "k*", ms=14, label="target")
    ax.set_title(name)
    ax.set_xlabel("y: along-track [m]   (target moves ->)")
    ax.set_ylabel("x: radial [m]   (up = away from Earth)")
    ax.grid(alpha=0.3); ax.legend(fontsize=8)
    ax.margins(0.15)
    print(f"{name:34s} start (y, x) = ({y[0]:7.1f}, {x[0]:6.1f}) m  ->  "
          f"after 2 orbits ({y[-1]:8.1f}, {x[-1]:6.1f}) m")
fig.suptitle("Natural (unpowered) relative motion near the target, CW model")
fig.tight_layout()
fig.savefig(os.path.join(FIG, "step1_natural_motion.png"), dpi=150, bbox_inches="tight")

# ---- Part B1: numerical CW vs closed-form STM --------------------------------
rng = np.random.default_rng(1)
worst = 0.0
for _ in range(20):
    s0 = np.concatenate([rng.uniform(-500, 500, 3), rng.uniform(-0.5, 0.5, 3)])
    sol = solve_ivp(o.cw_rhs, (0, t_end), s0, t_eval=t[::50], **TOL)
    exact = np.array([o.cw_stm(tk) @ s0 for tk in sol.t]).T
    worst = max(worst, np.abs(sol.y[:3] - exact[:3]).max())
print(f"\nCheck 1: numerical CW vs closed-form STM, max position error = {worst:.1e} m")


# ---- Part B2: CW vs nonlinear two-body, for growing separations --------------
def nonlinear_lvlh(s0, t):
    chaser0 = o.lvlh_to_eci(np.asarray(s0, float), 0.0)
    sol = solve_ivp(o.two_body_rhs, (0, t[-1]), chaser0, t_eval=t, **TOL)
    return np.array([o.eci_to_lvlh(sol.y[:, k], tk) for k, tk in enumerate(t)]).T


# frame round-trip sanity check
s_test = np.array([30.0, -200.0, 10.0, 0.01, 0.02, -0.03])
rt = o.eci_to_lvlh(o.lvlh_to_eci(s_test, 123.0), 123.0)
print(f"Check 2: LVLH -> ECI -> LVLH round-trip error = {np.abs(rt - s_test).max():.1e}")

base = np.array([50, -300, 20, 0, -2 * n * 50, 0.0])   # safety ellipse + cross-track
fig2, ax2 = plt.subplots(figsize=(8, 4.5))
print("Check 3: CW vs nonlinear over 2 orbits (safety-ellipse motion, scaled)")
for scale, c in zip([1, 10, 100], ["C0", "C1", "C3"]):
    s0 = base * scale
    cw = solve_ivp(o.cw_rhs, (0, t_end), s0, t_eval=t, **TOL).y
    nl = nonlinear_lvlh(s0, t)
    err = np.linalg.norm(cw[:3] - nl[:3], axis=0)
    size = np.linalg.norm(cw[:3], axis=0).max()
    print(f"   size ~{size:8.0f} m : max error {err.max():9.3f} m  ({100 * err.max() / size:.3f} % of size)")
    ax2.semilogy(t / o.PERIOD, err + 1e-12, c, label=f"motion size ~{size/1e3:.2g} km")
ax2.set_xlabel("time [orbits]"); ax2.set_ylabel("|CW - nonlinear| position [m]")
ax2.set_title("Linear CW model error grows with distance from the target")
ax2.grid(alpha=0.3, which="both"); ax2.legend()
fig2.savefig(os.path.join(FIG, "step1_cw_vs_nonlinear.png"), dpi=150, bbox_inches="tight")
print("saved figures to", FIG)
