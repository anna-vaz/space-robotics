"""Project 6 step 3: sensors + attitude determination (TRIAD and MEKF).

Truth: the detumbled CubeSat (|omega| ~ 0.4 deg/s, disturbances on) for 3 orbits,
starting in sunlight. Flight software: gyro at 10 Hz, magnetometer + coarse sun sensors
at 1 Hz, onboard field model IGRF degree 8 (truth: degree 13).
  * TRIAD (magnetometer primary) whenever the Sun is visible
  * MEKF with gyro-bias estimation, initialised from the first TRIAD
  * 50-run Monte Carlo (noise, initial bias and initial attitude error drawn from P0)
    -> filter consistency: average NEES of the 6 error states should be ~6
Run:  ../.venv/Scripts/python.exe step3_estimation.py     (~3 min on 12 cores)
"""
import time
from multiprocessing import Pool
import numpy as np
from scipy.integrate import solve_ivp
from scipy.interpolate import CubicSpline
from scipy.stats import chi2

import attitude as at
import environment as en
import estimation as es
import orbit as ob
import params as pr
import sensors as sn

N_ORB = 3
T_SIM = N_ORB * ob.PERIOD
W0 = np.radians([0.3, -0.2, 0.25])          # after detumbling (step 2 left ~0.2 deg/s)
BETA0 = at.prv2ep([1, -1, 2], np.radians(70))
DT = sn.DT_GYRO
N_SUB = int(round(1.0 / DT))                # gyro samples per 1 Hz measurement
P0_ATT = np.radians(3.0)                    # initial attitude sigma (TRIAD-level)
P0_BIAS = np.radians(0.3)                   # turn-on bias sigma per axis
N_MC = 36
# Measurement-noise settings of the filter (R):
#   naive : field-model error treated as white noise, CSS noise from the sensor spec
#   tuned : the deg-8 vs deg-13 field error is CORRELATED over minutes (wavelengths of
#           ~3000 km at 7.6 km/s), so a white-noise R must be inflated (~sqrt of the number of
#           correlated samples); CSS clipping near grazing angles adds error too.
FILTER_SETTINGS = dict(naive=dict(model_infl=1.0, sig_sun=0.015),
                       tuned=dict(model_infl=10.0, sig_sun=0.03))
TRUTH_FILE = "results_step3_truth.npz"


def make_truth():
    env = en.OrbitEnvironment(T_SIM + 3 * ob.PERIOD)          # extra margin to find sunlight
    i0 = np.argmax(~env.shadow & np.roll(env.shadow, 1))     # first eclipse exit
    t_start = env.t[i0] + 60.0
    t10 = t_start + np.arange(0, T_SIM, DT)

    def rhs(t, x):
        tq = sum(en.disturbance_torques(at.ep2dcm(x[:4]), env.at(t)).values())
        bd, wd = at.rigid_body_rhs(x[:4], x[4:], tq, pr.J, pr.J_INV)
        return np.concatenate([bd, wd])

    sol = solve_ivp(rhs, (t10[0], t10[-1]), np.concatenate([BETA0, W0]), method="DOP853",
                    rtol=1e-11, atol=1e-12, t_eval=t10)
    beta10, w10 = sol.y[:4].T, sol.y[4:].T
    beta10 /= np.linalg.norm(beta10, axis=1)[:, None]
    t1 = t10[::N_SUB]
    B13 = env._B(t1)                                           # truth field (degree 13), ECI
    B8_grid = en.igrf_eci(env.r, env.jd, env.field_date, max_degree=8)
    B8 = CubicSpline(env.t, B8_grid)(t1)                       # onboard model, ECI
    sun = env._sun(t1)
    sun /= np.linalg.norm(sun, axis=1)[:, None]
    r1 = env.sol.sol(t1)[:3].T
    shadow = en.in_shadow(r1, sun)
    BN1 = np.array([at.ep2dcm(b) for b in beta10[::N_SUB]])
    np.savez(TRUTH_FILE, t10=t10, w10=w10, beta10=beta10, t1=t1, BN1=BN1,
             B_body=np.einsum("kij,kj->ki", BN1, B13), B8=B8, B13=B13, sun_N=sun,
             sun_B=np.einsum("kij,kj->ki", BN1, sun), shadow=shadow)


def run(seed, setting="tuned", keep_history=False):
    fs = FILTER_SETTINGS[setting]
    d = dict(np.load(TRUTH_FILE))
    rng = np.random.default_rng(seed)
    nominal = seed == 0
    b0 = sn.GYRO_B0 if nominal else P0_BIAS * rng.standard_normal(3)
    w_m, b_true = sn.gyro_series(d["w10"], rng, b0=b0)
    sig_model = np.sqrt(np.mean((d["B13"] - d["B8"])**2))      # field-model error per axis
    n1 = len(d["t1"])
    hist = dict(err=np.zeros((n1, 3)), ang=np.zeros(n1), sig=np.zeros((n1, 3)),
                b_est=np.zeros((n1, 3)), b_true=np.zeros((n1, 3)), sigb=np.zeros((n1, 3)),
                nees=np.zeros(n1), triad=np.full(n1, np.nan), sun_ok=np.zeros(n1, bool))
    f = None
    for j in range(n1):
        k = j * N_SUB
        if j > 0:                                             # 10 gyro steps since last second
            for i in range(k - N_SUB, k):
                f.propagate(w_m[i], DT)
        Bm = sn.magnetometer(d["B_body"][j], rng)
        sm = sn.coarse_sun_sensors(d["sun_B"][j], d["shadow"][j], rng)
        BN_true = d["BN1"][j]
        mag_b, mag_n = Bm / np.linalg.norm(Bm), d["B8"][j] / np.linalg.norm(d["B8"][j])
        if sm is not None:
            BN_tr = es.triad(mag_b, sm, mag_n, d["sun_N"][j])
            hist["triad"][j] = es.attitude_error(BN_true, BN_tr)[1]
            hist["sun_ok"][j] = True
        if f is None:                                         # initialise from TRIAD
            if nominal:
                beta_init = at.dcm2ep(BN_tr)
            else:   # MC: start from truth perturbed by a draw from P0 (consistency test)
                da = P0_ATT * rng.standard_normal(3)
                beta_init = at.dcm2ep(at.ep2dcm(at.prv2ep(da, np.linalg.norm(da))) @ BN_true)
            f = es.MEKF(beta_init, np.zeros(3), np.diag([P0_ATT**2] * 3 + [P0_BIAS**2] * 3),
                        sn.SIGMA_V, sn.SIGMA_U)
        else:
            sig_mag = np.sqrt(sn.SIGMA_MAG**2 + (fs["model_infl"] * sig_model)**2) / np.linalg.norm(Bm)
            f.update(mag_b, mag_n, sig_mag)
            if sm is not None:
                f.update(sm, d["sun_N"][j], fs["sig_sun"])
        e, ang = es.attitude_error(BN_true, at.ep2dcm(f.beta))
        bt = b_true[k]
        x_err = np.concatenate([e, bt - f.b])
        hist["err"][j], hist["ang"][j] = e, ang
        hist["sig"][j] = np.sqrt(np.diag(f.P)[:3])
        hist["sigb"][j] = np.sqrt(np.diag(f.P)[3:])
        hist["b_est"][j], hist["b_true"][j] = f.b, bt
        hist["nees"][j] = x_err @ np.linalg.solve(f.P, x_err)
    hist["sig_model"] = sig_model
    return hist if keep_history else dict(nees=hist["nees"], ang=hist["ang"], triad=hist["triad"])


if __name__ == "__main__":
    t0 = time.time()
    make_truth()
    d = dict(np.load(TRUTH_FILE))
    print(f"truth: {len(d['t10'])} samples at 10 Hz, {time.time()-t0:.0f} s; eclipse fraction "
          f"{d['shadow'].mean():.2f}; |omega| {np.degrees(np.linalg.norm(d['w10'],axis=1)).min():.2f}"
          f"..{np.degrees(np.linalg.norm(d['w10'],axis=1)).max():.2f} deg/s")
    with Pool(12) as pool:
        out_all = pool.starmap(run, [(sd, st) for st in ("naive", "tuned") for sd in range(1, N_MC + 1)])
    out_naive, out = out_all[:N_MC], out_all[N_MC:]
    H = run(0, "tuned", keep_history=True)
    tmin = (d["t1"] - d["t1"][0]) / 60
    sun = ~d["shadow"]

    # ---- sensor-level numbers
    Bn = np.linalg.norm(d["B_body"], axis=1)
    print("\n== Sensors")
    print(f"  onboard field model (IGRF deg 8) error: {H['sig_model']*1e9:.1f} nT rms per axis; "
          f"direction noise mag ~ {np.degrees(np.sqrt(sn.SIGMA_MAG**2+H['sig_model']**2)/Bn.mean()):.2f} deg")
    print(f"  gyro: ARW {np.degrees(sn.SIGMA_V)*60:.2f} deg/sqrt(h), per-sample noise "
          f"{np.degrees(sn.SIGMA_V/np.sqrt(DT)):.4f} deg/s, turn-on bias {np.degrees(sn.GYRO_B0)} deg/s")
    tri = np.degrees(H["triad"][sun])
    print(f"  TRIAD (sunlit only): rms {np.sqrt(np.nanmean(tri**2)):.2f} deg, 95th pct {np.nanpercentile(tri,95):.2f} deg;"
          f"  not available {100*np.mean(~H['sun_ok']):.0f} % of the time (eclipse)")

    # ---- nominal MEKF
    late = tmin > 30
    ang = np.degrees(H["ang"])
    print("\n== MEKF, nominal run (t > 30 min)")
    print(f"  attitude error rms: sunlit {np.sqrt(np.mean(ang[late & sun]**2)):.3f} deg, "
          f"eclipse {np.sqrt(np.mean(ang[late & ~sun]**2)):.3f} deg, max {ang[late].max():.3f} deg")
    be = np.degrees(H["b_est"] - H["b_true"])
    print(f"  bias error at end: {be[-1]} deg/s (3-sigma {3*np.degrees(H['sigb'][-1])})")
    inside = np.abs(H["err"]) <= 3 * H["sig"]
    print(f"  attitude errors inside +-3 sigma: {100*inside[late].mean():.1f} % of samples (expect 99.7)")

    # ---- Monte Carlo consistency
    NE = np.array([o["nees"] for o in out])
    ANG = np.degrees(np.array([o["ang"] for o in out]))
    anees = NE.mean(axis=0)
    anees_naive = np.array([o["nees"] for o in out_naive]).mean(axis=0)
    ANG_naive = np.degrees(np.array([o["ang"] for o in out_naive]))
    lo, hi = chi2.ppf([0.025, 0.975], 6 * N_MC) / N_MC
    frac = np.mean((anees[late] > lo) & (anees[late] < hi))
    print(f"\n== Monte Carlo, {N_MC} runs: average NEES (6 states)")
    print(f"  NAIVE R (model error as white noise): time-mean ANEES {anees_naive[late].mean():.2f}, "
          f"attitude rms {np.sqrt(np.mean(ANG_naive[:, late]**2)):.3f} deg  -> over-confident")
    print("  TUNED R (model-error term x10, CSS 0.03 rad):")
    print(f"  time-mean ANEES {anees[late].mean():.2f} (ideal 6), sunlit {anees[late & sun].mean():.2f}, "
          f"eclipse {anees[late & ~sun].mean():.2f}")
    print(f"  95 % band for {N_MC} runs [{lo:.2f}, {hi:.2f}]: ANEES inside for {100*frac:.0f} % of samples")
    print(f"  attitude error over runs (t > 30 min): rms {np.sqrt(np.mean(ANG[:, late]**2)):.3f} deg, "
          f"99th pct {np.percentile(ANG[:, late], 99):.3f} deg")
    np.savez("results_step3.npz", tmin=tmin, anees=anees, ang_mc=ANG, **{k: v for k, v in H.items() if k != 'sig_model'})

    # ---------------------------------------------------------------- figures
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    def shade(ax):
        s = d["shadow"].astype(int)
        on = list(np.where(np.diff(s) == 1)[0] + 1); off = list(np.where(np.diff(s) == -1)[0] + 1)
        if s[0]: on = [0] + on
        if len(off) < len(on): off.append(len(s) - 1)
        for a0, a1 in zip(on, off):
            ax.axvspan(tmin[a0], tmin[a1], color="0.88", zorder=0)

    fig, axs = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    for i, c in enumerate("rgb"):
        axs[0].plot(tmin, np.degrees(H["err"][:, i]), c, lw=0.7, label=f"error about {'xyz'[i]}")
        axs[0].plot(tmin, 3 * np.degrees(H["sig"][:, i]), c + "--", lw=0.7)
        axs[0].plot(tmin, -3 * np.degrees(H["sig"][:, i]), c + "--", lw=0.7)
    axs[0].set(ylabel="MEKF error [deg]", ylim=(-1.5, 1.5),
               title="Attitude determination, nominal run (dashed = +-3 sigma, grey = eclipse)")
    axs[0].legend(fontsize=8, ncol=3, loc="upper right")
    axs[1].semilogy(tmin, np.degrees(H["triad"]), ".", color="C1", ms=1.5, label="TRIAD (mag + Sun)")
    axs[1].semilogy(tmin, ang, "k", lw=0.8, label="MEKF")
    axs[1].set(ylabel="total error angle [deg]", ylim=(5e-3, 20)); axs[1].legend(fontsize=8, loc="upper right")
    for i, c in enumerate("rgb"):
        axs[2].plot(tmin, np.degrees(H["b_true"][:, i]), c, lw=1.5, alpha=0.4)
        axs[2].plot(tmin, np.degrees(H["b_est"][:, i]), c, lw=0.8, label=f"b_{'xyz'[i]}")
    axs[2].set(ylabel="gyro bias [deg/s]", ylim=(-0.4, 0.3))
    axs[2].legend(fontsize=8, ncol=3, title="thin = estimate, thick = truth", title_fontsize=8, loc="lower right")
    axs[3].semilogy(tmin, anees_naive, color="C3", lw=0.6, label="naive R (field-model error as white noise)")
    axs[3].semilogy(tmin, anees, "k", lw=0.6, label=f"tuned R  ({N_MC}-run average NEES)")
    axs[3].axhline(6, color="g", lw=1, label="ideal (6 states)")
    axs[3].axhspan(lo, hi, color="g", alpha=0.15, label="95 % band")
    axs[3].set(ylabel="ANEES", xlabel="time [min]", ylim=(1, 100)); axs[3].legend(fontsize=8, loc="upper right")
    for a in axs:
        shade(a); a.grid(alpha=0.3, which="both")
    fig.tight_layout(); fig.savefig("figures/step3_estimation.png", dpi=130)
    print(f"\nfigure written; total {time.time()-t0:.0f} s")
