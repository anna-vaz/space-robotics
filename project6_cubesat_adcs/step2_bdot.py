"""Project 6 step 2: B-dot detumbling of the 3U CubeSat after deployment.

Initial tumble 10 deg/s on every body axis (|omega| = 17.3 deg/s). Cases (5 orbits):
  A  continuous ideal law, no saturation, no disturbances  -> energy check
  B  continuous ideal law, no saturation, disturbances on   -> vs hand prediction
  C  continuous, 0.2 A m^2 saturation, disturbances
  D  flight software: 1 Hz magnetometer (100 nT noise), finite-difference b_dot,
     coils on 80 % of each second, saturation, disturbances
  E  as D without magnetometer noise
plus a gain sweep k/k_opt = 0.1 .. 10 (continuous, saturated).
Run:  ../.venv/Scripts/python.exe step2_bdot.py      (~4 min on 12 cores)
"""
import time
from multiprocessing import Pool
import numpy as np

import attitude as at
import bdot
import environment as en
import orbit as ob
import params as pr

N_ORB = 5
T_END = N_ORB * ob.PERIOD
BETA0 = at.prv2ep([1, 2, 3], np.radians(40))
W0 = np.radians([10.0, -10.0, 10.0])
M_MAX = pr.MTQ_MAX
J_RATIO = np.linalg.eigvalsh(pr.J)[-1] / np.linalg.eigvalsh(pr.J)[0]   # k_opt with J_max instead of J_min
GAINS = [0.1, 0.25, 0.5, 1.0, 2.0, 3.0, 4.0, round(J_RATIO, 2), 10.0]

_env = None


def get_env():
    global _env
    if _env is None:
        _env = en.OrbitEnvironment(T_END + 60)
    return _env


def run(case):
    env = get_env()
    k_opt, _ = bdot.gain_opt(env)
    name, kind, kmul, m_max, disturb, sigma = case
    tw = time.time()
    if kind == "cont":
        r = bdot.simulate_continuous(env, kmul * k_opt, m_max, BETA0, W0, T_END, disturb=disturb)
    else:
        r = bdot.simulate_flight(env, kmul * k_opt, m_max, BETA0, W0, T_END, sigma=sigma, disturb=disturb)
    r["wall"] = time.time() - tw
    return name, r


if __name__ == "__main__":
    t0 = time.time()
    env = get_env()
    k_opt, xi = bdot.gain_opt(env)
    n = np.sqrt(ob.MU / ob.A**3)
    print("== Gain")
    print(f"  orbit inclination w.r.t. geomagnetic equator xi_m = {xi:.1f} deg (mean over a day)")
    print(f"  k_opt = 2 n (1 + sin xi_m) J_min = 2 * {n:.4e} * {1+np.sin(np.radians(xi)):.3f} * "
          f"{np.linalg.eigvalsh(pr.J)[0]:.4f} = {k_opt:.3e} N m s")
    m0 = k_opt * np.linalg.norm(W0) / 30e-6
    print(f"  initial unsaturated dipole ~ k |omega| / |B| = {m0:.2f} A m^2  (limit {M_MAX} per axis)")

    cases = [("A", "cont", 1.0, None, False, 0), ("B", "cont", 1.0, None, True, 0),
             ("C", "cont", 1.0, M_MAX, True, 0), ("D", "flight", 1.0, M_MAX, True, 100e-9),
             ("E", "flight", 1.0, M_MAX, True, 0.0)]
    cases += [(f"k{g}", "cont", g, M_MAX, True, 0) for g in GAINS if g != 1.0]
    with Pool(min(len(cases), 12)) as pool:
        R = dict(pool.map(run, cases))
    R["k1.0"] = R["C"]

    def deg(w):
        return np.degrees(np.linalg.norm(w, axis=1))

    # ---- energy check (case A) ----
    A = R["A"]
    T = np.array([0.5 * w @ pr.J @ w for w in A["w"]])
    print("\n== A: energy balance without disturbances (T' = omega . tau_mtq = -k |omega_perp|^2)")
    print(f"  T(0) = {T[0]:.4e} J -> T(end) = {T[-1]:.3e} J")
    print(f"  |T(t) - T(0) - integral(omega . tau)| max = {np.abs(T - T[0] - A['E_diss']).max():.1e} J")
    print(f"  largest increase of T between samples: {max(np.diff(T).max(), 0):.1e} J (monotonic decrease)")

    # ---- hand prediction: isotropic field direction, omega_i decays with rate 2k/(3 J_i) ----
    Jp, V = np.linalg.eigh(pr.J)
    tt = R["B"]["t"]
    w_p0 = V.T @ W0
    w_hand = np.array([np.linalg.norm(w_p0 * np.exp(-2 * k_opt / (3 * Jp) * t)) for t in tt])
    t_hand = bdot.detumble_time(tt, np.column_stack([w_hand, 0 * tt, 0 * tt]))
    print("\n== Detumble time to |omega| < 0.5 deg/s (stays below)")
    print(f"  hand estimate (b isotropic, <I - b b^T> = 2/3 I): {t_hand/60:.0f} min = {t_hand/ob.PERIOD:.2f} orbits"
          f"  (slow axis tau = 3 J_max/(2k) = {3*Jp[-1]/(2*k_opt)/60:.0f} min)")
    labels = dict(A="A ideal, no sat, no dist", B="B ideal, no saturation", C="C ideal, 0.2 A m^2 saturation",
                  D="D flight 1 Hz, 100 nT noise", E="E flight 1 Hz, no noise (under D)")
    for c in "ABCDE":
        r = R[c]
        td = bdot.detumble_time(r["t"], r["w"])
        last = r["t"] > T_END - ob.PERIOD
        wl = r["w"][last]
        # direction of the residual rate vs the orbit normal (in ECI)
        hN = np.cross(env.r[0], env.v[0]); hN /= np.linalg.norm(hN)
        cosang = [abs((at.ep2dcm(b).T @ w) @ hN) / np.linalg.norm(w) for b, w in zip(r["beta"][last], wl)]
        print(f"  {labels[c]:32s}: {td/60:6.1f} min = {td/ob.PERIOD:.2f} orbits;  last orbit |omega| "
              f"{deg(wl).mean():.3f} deg/s (max {deg(wl).max():.3f}), angle to orbit normal "
              f"{np.degrees(np.arccos(np.mean(cosang))):.0f} deg  [{r['wall']:.0f} s]")
    print(f"  reference: 2n = {np.degrees(2*n):.3f} deg/s (field direction turns twice per orbit on a polar orbit)")

    print("\n== Gain sweep (continuous, saturated, disturbances)")
    sweep = []
    for g in GAINS:
        r = R[f"k{g}"]
        td = bdot.detumble_time(r["t"], r["w"])
        last = r["t"] > T_END - ob.PERIOD
        sweep.append((g, td, deg(r["w"][last]).mean()))
        print(f"  k = {g:5.2f} k_opt: detumble {td/ob.PERIOD:5.2f} orbits, last-orbit mean |omega| {sweep[-1][2]:.3f} deg/s")

    np.savez("results_step2.npz", **{f"{c}_{key}": R[c][key] for c in "BCDE" for key in ("t", "w")},
             D_m=R["D"]["m"], t_hand=tt, w_hand=w_hand, sweep=np.array(sweep))

    # ---------------------------------------------------------------- figures
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    P = ob.PERIOD
    fig, axs = plt.subplots(3, 1, figsize=(10, 10), gridspec_kw=dict(height_ratios=[1.4, 1, 1]))
    ax = axs[0]
    for c, col in zip("BCDE", ["C0", "C1", "C3", "0.4"]):
        ax.semilogy(R[c]["t"] / P, deg(R[c]["w"]), color=col, lw=1.1 if c != "E" else 0.8,
                    ls="-" if c != "E" else ":", label=labels[c])
    ax.semilogy(tt / P, np.degrees(w_hand), "k--", lw=1, label="hand estimate (isotropic field)")
    ax.axhline(0.5, color="g", lw=0.8, ls="-.", label="0.5 deg/s detumbled")
    ax.axhline(np.degrees(2 * n), color="m", lw=0.8, ls="-.", label="2n (field-rotation rate)")
    ax.set(ylabel="|omega| [deg/s]", ylim=(0.03, 30), xlim=(0, N_ORB),
           title="B-dot detumbling of a 3U CubeSat from 10 deg/s per axis (500 km SSO)")
    ax.legend(fontsize=8, ncol=2, loc="upper right"); ax.grid(alpha=0.3, which="both")
    D = R["D"]
    sel = D["t"] < 1.6 * P
    for i, col in enumerate("rgb"):
        axs[1].plot(D["t"][sel] / P, np.degrees(D["w"][sel, i]), col, lw=0.9, label=f"omega_{'xyz'[i]}")
        axs[2].plot(D["t"][sel] / P, D["m"][sel, i], col, lw=0.6, label=f"m_{'xyz'[i]}")
    axs[1].set(ylabel="body rates [deg/s]"); axs[1].legend(fontsize=8, ncol=3)
    axs[1].set_title("Case D (flight software) -- first 1.6 orbits", fontsize=10)
    axs[2].axhline(M_MAX, color="k", lw=0.6, ls="--"); axs[2].axhline(-M_MAX, color="k", lw=0.6, ls="--")
    axs[2].set(ylabel="dipole command [A m^2]", xlabel="time [orbits]", ylim=(-0.25, 0.25))
    axs[2].legend(fontsize=8, ncol=3, loc="upper right")
    for a in axs[1:]:
        a.grid(alpha=0.3); a.set_xlim(0, 1.6)
    fig.tight_layout(); fig.savefig("figures/step2_detumble.png", dpi=130)

    sw = np.array(sweep)
    fig, ax1 = plt.subplots(figsize=(7, 4))
    ax1.semilogx(sw[:, 0], sw[:, 1] / P, "o-", color="C0")
    ax1.set(xlabel="gain k / k_opt", ylabel="detumble time to 0.5 deg/s [orbits]",
            title="B-dot gain sweep (saturated at 0.2 A m^2)")
    ax2 = ax1.twinx()
    ax2.semilogx(sw[:, 0], sw[:, 2], "s--", color="C3")
    ax2.set_ylabel("mean |omega| over the last orbit [deg/s]", color="C3")
    for g, td, _ in sweep:
        if not np.isfinite(td):
            ax1.annotate("not detumbled\nin 5 orbits", (g, 2.5), ha="center", fontsize=8, color="C0")
    ax1.axvline(J_RATIO, color="0.5", ls=":", lw=1)
    ax1.text(J_RATIO / 1.06, 2.0, "k_opt formula\nwith J_max", fontsize=8, color="0.3", ha="right")
    ax1.grid(alpha=0.3, which="both")
    fig.tight_layout(); fig.savefig("figures/step2_gain_sweep.png", dpi=130)
    print(f"\nfigures written; total {time.time()-t0:.0f} s")
