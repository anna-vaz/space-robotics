"""Project 3, step 4: detumbling the captured target with a reaction wheel.

Three tumble rates (3, 6, 9 deg/s), same chaser and same 1 N m s wheel.
Checks: total angular momentum (incl. the rotor) conserved; final spin and
slew rate match the momentum budget  L_total = I_stack * omega + h_wheel.
"""
import os
from multiprocessing import Pool

import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, "figures")
SPINS = (3.0, 6.0, 9.0)
T_DET = 900.0


def job(spin):
    import capture as cp
    import contact as ct
    import gen_jacobian as gj
    import target as tg
    r = cp.run(spin, t_detumble=T_DET, n_out=1801)
    X = r["X"]
    p_h, v_h, _ = ct.target_handle(X[0, 10:16])
    r["gap_mm"] = 1000 * np.linalg.norm(p_h - gj.end_effector(X[0, :5]))
    r["L"] = np.array([cp.total_L(x) for x in X])
    _, r["I_sys"], _ = ct.system_inertia(X[-1, :5], X[-1, 10:16])
    r["h"] = cp.I_W * X[:, 16]
    r["tau_w"] = np.array([cp.wheel_torque(x[:5], x[5:10], x[16]) for x in X])
    return r


if __name__ == "__main__":
    import capture as cp
    with Pool(len(SPINS)) as pool:
        runs = pool.map(job, SPINS)
    deg = np.degrees
    print(f"Wheel: capacity {cp.H_MAX} N m s, torque limit {cp.TAU_W_MAX} N m\n")
    for r in runs:
        t, X, L, h = r["t"], r["X"], r["L"], r["h"]
        L0, I = L[0], r["I_sys"]
        rate = deg(X[:, 7])
        sat = np.mean(np.abs(h) >= cp.H_MAX * 0.999) * 100
        print(f"--- target tumbling at {r['spin']:.0f} deg/s ---")
        print(f"  grasp: hand-handle gap {r['gap_mm']:.2f} mm; base attitude at grasp {deg(X[0, 2]):.1f} deg")
        print(f"  total angular momentum L = {L0:.4f} N m s (drift {np.ptp(L):.1e}); stack inertia {I:.1f} kg m^2")
        if L0 < cp.H_MAX:
            slew = deg((cp.H_MAX - L0) / I)
            satk = np.abs(h) >= cp.H_MAX * 0.999
            obs = f"{np.median(np.abs(rate[satk])):.3f} deg/s while saturated" if satk.any() else "never needed"
            print(f"  wheel CAN absorb it ({100*L0/cp.H_MAX:.0f} % of capacity). Max return slew rate "
                  f"(H_MAX - L)/I = {slew:.3f} deg/s; observed {obs}")
            print(f"  after {t[-1]:.0f} s: base attitude {deg(X[-1, 2]):.3f} deg, rate {rate[-1]:.1e} deg/s, "
                  f"wheel {h[-1]:.3f} N m s (= L, as expected); wheel saturated {sat:.0f} % of the time")
        else:
            resid = deg((L0 - cp.H_MAX) / I)
            print(f"  wheel CANNOT absorb it (needs {100*L0/cp.H_MAX:.0f} % of capacity).")
            print(f"  predicted residual spin (L - H_MAX)/I = {resid:.4f} deg/s; "
                  f"simulated at the end {rate[-1]:.4f} deg/s; wheel saturated {sat:.0f} % of the time")
    np.savez(os.path.join(HERE, "results_step4.npz"), t=runs[0]["t"], X=runs[0]["X"],
             q_grasp=runs[0]["q_grasp"])

    # ---- figure -------------------------------------------------------------------
    fig, ax = plt.subplots(2, 2, figsize=(12, 7.5))
    tm = runs[0]["t"] / 60
    for r, c in zip(runs, ("C0", "C1", "C3")):
        lab = f"{r['spin']:.0f} deg/s"
        ax[0, 0].plot(tm, deg(r["X"][:, 15]), c, label=f"target spin, {lab}")
        ax[0, 1].plot(tm, r["h"], c, label=lab)
        ax[1, 0].plot(tm, deg(r["X"][:, 2]), c, label=lab)
        ax[1, 1].plot(tm, r["tau_w"], c, label=lab, lw=1)
    ax[0, 1].axhline(cp.H_MAX, color="k", ls=":", label="capacity")
    ax[1, 1].axhline(cp.TAU_W_MAX, color="k", ls=":"); ax[1, 1].axhline(-cp.TAU_W_MAX, color="k", ls=":")
    ax[0, 0].set_ylabel("deg/s"); ax[0, 0].set_title("Spin of the captured target (first 2 min)")
    ax[0, 0].set_xlim(0, 2); ax[0, 0].set_xlabel("time after grasp [min]")
    ax[0, 1].set_ylabel("N m s"); ax[0, 1].set_title("Reaction-wheel momentum")
    ax[1, 0].set_ylabel("deg"); ax[1, 0].set_title("Chaser base attitude (goal: back to 0)")
    ax[1, 1].set_ylabel("N m"); ax[1, 1].set_title("Wheel motor torque")
    for a in ax.flat:
        a.grid(alpha=0.3); a.legend(fontsize=8)
    for a in ax[1]:
        a.set_xlabel("time after grasp [min]")
    fig.tight_layout()
    fig.savefig(os.path.join(FIG, "step4_detumble.png"), dpi=150)
    print("\nsaved", os.path.join(FIG, "step4_detumble.png"))
