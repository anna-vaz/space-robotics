"""Project 6 step 4: reaction-wheel pointing + magnetic momentum unloading, full loop.

  1. Momentum budget (cheap): perfect nadir tracking for 1 day, wheel momentum from
     h_w' = tau_ext - w_r x (J w_r + h_w) - J w_r'   with / without unloading.
  2. Closed loop (truth -> sensors -> MEKF -> control -> wheels/coils), nadir, 3 orbits,
     starting from the detumbled state of step 3:
       a  perfect attitude knowledge (control error only)
       b  MEKF, unloading on
       c  MEKF, unloading off
  3. Ground-target pass: track Athens (37.98 N, 23.73 E) on the 31-deg daylight pass at
     t = 8.4 h (found by scanning one day of passes), nadir before and after.
Run:  ../.venv/Scripts/python.exe step4_pointing.py     (~5 min on 12 cores)
"""
import time
from multiprocessing import Pool
import numpy as np
from scipy.integrate import solve_ivp

import attitude as at
import closed_loop as cl
import controller as ct
import environment as en
import orbit as ob
import params as pr

BETA0 = at.prv2ep([1, -1, 2], np.radians(70))
W0 = np.radians([0.3, -0.2, 0.25])
N_ORB = 3
LAT, LON = np.radians(37.98), np.radians(23.73)
P_ATHENS = ob.RE * np.array([np.cos(LAT) * np.cos(LON), np.cos(LAT) * np.sin(LON), np.sin(LAT)])
T_AOS, T_LOS = 30280.0, 30940.0            # Athens pass (elevation > 0), from a 1-day scan
PASS_T0, PASS_T = T_AOS - 25 * 60, (T_LOS - T_AOS) + 30 * 60
ENV_T = 34000.0

_env = None


def get_env():
    global _env
    if _env is None:
        env = en.OrbitEnvironment(ENV_T)
        _env = (env, cl.onboard_field(env))
    return _env


def nadir_start(env):
    i0 = np.argmax(~env.shadow & np.roll(env.shadow, 1))
    return env.t[i0] + 60.0


def elevation(env, t):
    e = env.at(t)
    p = ct.target_eci(P_ATHENS, env.jd0 + t / 86400)
    los = e["r"] - p
    return np.degrees(np.arcsin(los @ p / np.linalg.norm(los) / np.linalg.norm(p)))


EL_SWITCH = -10.0     # start the slew before AOS so the camera is on target at elevation 0


def pass_mode(t):
    env, _ = get_env()
    return P_ATHENS if elevation(env, t) > EL_SWITCH else None


def run(case):
    env, B8 = get_env()
    tw = time.time()
    if case.startswith("nadir"):
        t0 = nadir_start(env)
        r = cl.simulate(env, B8, t0, N_ORB * ob.PERIOD, BETA0, W0, estimator=case != "nadir_truth",
                        unload=case != "nadir_nounload", seed=1)
    else:
        r = cl.simulate(env, B8, PASS_T0, PASS_T, BETA0, W0, mode_fn=pass_mode,
                        estimator=case != "pass_truth", seed=2)
        r["el"] = np.array([elevation(env, t) for t in r["t"]])
    r["wall"] = time.time() - tw
    return case, r


def momentum_budget(unload, days=1.0):
    env = en.OrbitEnvironment(days * 86400 + 60)
    ctrl = ct.PointingController(k_unload=ct.K_UNLOAD if unload else 0.0)

    def rhs(t, h):
        RN, wr, wrd = ct.reference(env, t)
        e = env.at(t)
        tq = sum(en.disturbance_torques(RN, e).values())
        BB = RN @ e["B"]
        m = ctrl.unloading_dipole(h, BB)
        tq = tq + ct.DUTY * np.cross(m, BB)
        return tq - np.cross(wr, pr.J @ wr + h) - pr.J @ wrd

    t_eval = np.arange(0, days * 86400, 60.0)
    sol = solve_ivp(rhs, (0, t_eval[-1]), np.zeros(3), method="DOP853", rtol=1e-8, atol=1e-12, t_eval=t_eval)
    return sol.t, sol.y.T


if __name__ == "__main__":
    t_start = time.time()
    cases = ["nadir_truth", "nadir_mekf", "nadir_nounload", "pass_mekf", "pass_truth"]
    with Pool(len(cases) + 2) as pool:
        async_res = pool.map_async(run, cases)
        bud = pool.map(momentum_budget, [False, True])
        R = dict(async_res.get())
    P = ob.PERIOD

    print("== 1. Momentum budget, perfect nadir tracking for 1 day")
    tb, hb0 = bud[0]; _, hb1 = bud[1]
    n0, n1 = np.linalg.norm(hb0, axis=1), np.linalg.norm(hb1, axis=1)
    over = np.where(np.abs(hb0).max(axis=1) > pr.RW_HMAX)[0]
    print(f"  no unloading: |h_w| reaches {n0.max()*1e3:.2f} mN m s; a wheel hits {pr.RW_HMAX*1e3} mN m s after "
          + (f"{tb[over[0]]/3600:.1f} h" if len(over) else "more than 1 day"))
    print(f"  unloading (k = {ct.K_UNLOAD} 1/s): max |h_w| {n1.max()*1e3:.3f} mN m s, "
          f"last-orbit mean {n1[tb > tb[-1]-P].mean()*1e3:.3f} mN m s")

    print("\n== 2. Nadir pointing, closed loop, 3 orbits from the detumbled state")
    for c in ["nadir_truth", "nadir_mekf", "nadir_nounload"]:
        r = R[c]
        tt = r["t"] - r["t"][0]
        pe = np.degrees(r["point"])
        acq = tt[np.where(pe > 1.0)[0][-1] + 1] if (pe > 1).any() else 0
        ss = tt > acq + 300
        print(f"  {c:15s}: acquired (< 1 deg) after {acq:.0f} s; then pointing rms {np.sqrt(np.mean(pe[ss]**2)):.3f} deg, "
              f"99th pct {np.percentile(pe[ss], 99):.3f}, max {pe[ss].max():.3f};  knowledge rms "
              f"{np.degrees(np.sqrt(np.mean(r['know'][ss]**2))):.3f} deg")
        print(f"  {'':15s}  max |h_w| {np.abs(r['hw']).max()*1e3:.3f} mN m s (end {np.linalg.norm(r['hw'][-1])*1e3:.3f}), "
              f"max |u| {np.abs(r['u']).max()*1e3:.3f} mN m, max |m| {np.abs(r['m']).max():.4f} A m^2, "
              f"momentum check {r['Hchk'].max():.1e} N m s  [{r['wall']:.0f} s]")

    print("\n== 3. Athens pass (daylight, max elevation 31 deg)")
    for c in ["pass_truth", "pass_mekf"]:
        r = R[c]
        vis = r["el"] > 10
        ze = np.degrees(r["zerr"][vis])
        wmax = np.degrees(np.linalg.norm(r["w"][r["mode"]], axis=1)).max()
        print(f"  {c:10s}: boresight error for elevation > 10 deg: rms {np.sqrt(np.mean(ze**2)):.3f} deg, max {ze.max():.3f} deg "
              f"({vis.sum()} s); peak body rate {wmax:.2f} deg/s; max |h_w| {np.abs(r['hw']).max()*1e3:.3f} mN m s")
    rp = R["pass_mekf"]
    sw = np.where(np.diff(rp["mode"].astype(int)) == 1)[0][0] + 1
    sw_end = np.where(np.diff(rp["mode"].astype(int)) == -1)[0][0] + 1
    ze = np.degrees(rp["zerr"])
    k1 = sw + np.where(ze[sw:sw_end] > 1.0)[0][-1] + 1
    print(f"  slew nadir -> Athens starts at elevation {EL_SWITCH:.0f} deg: {ze[sw]:.1f} deg away; boresight < 1 deg "
          f"{k1 - sw} s later (elevation {rp['el'][k1]:.1f} deg) and stays there until the switch back")
    np.savez("results_step4.npz", **{f"{c}_{k}": v for c, r in R.items() for k, v in r.items() if k != "wall"},
             tb=tb, hb0=hb0, hb1=hb1)

    # ---------------------------------------------------------------- figures
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(tb / 3600, n0 * 1e3, "C3", label="no unloading")
    ax.plot(tb / 3600, n1 * 1e3, "C0", label=f"cross-product unloading, k = {ct.K_UNLOAD} 1/s")
    ax.axhline(pr.RW_HMAX * 1e3, color="k", ls="--", lw=0.8, label="wheel capacity (per axis)")
    ax.set(xlabel="time [h]", ylabel="|h_wheels| [mN m s]",
           title="Momentum budget: perfect nadir pointing for one day (all disturbances)")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig("figures/step4_budget.png", dpi=130)

    fig, axs = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    rt, rm, rn = R["nadir_truth"], R["nadir_mekf"], R["nadir_nounload"]
    to = (rm["t"] - rm["t"][0]) / P
    axs[0].semilogy(to, np.degrees(rm["point"]), "k", lw=0.6, label="pointing error (MEKF in the loop)")
    axs[0].semilogy(to, np.degrees(rm["know"]), "C1", lw=0.5, alpha=0.8, label="knowledge error (MEKF)")
    axs[0].semilogy(to, np.degrees(rt["point"]), "C0", lw=0.6, label="pointing error, perfect knowledge")
    axs[0].set(ylabel="error [deg]", ylim=(1e-3, 200),
               title="Nadir pointing from the detumbled state (3 orbits, grey = eclipse)")
    axs[0].legend(fontsize=8, loc="upper right", ncol=3)
    for i, c in enumerate("rgb"):
        axs[1].plot(to, rm["hw"][:, i] * 1e3, c, lw=0.8, label=f"h_{'xyz'[i]} unloading")
        axs[1].plot(to, rn["hw"][:, i] * 1e3, c, lw=0.6, ls=":", label=f"h_{'xyz'[i]} no unloading")
    axs[1].axhline(pr.RW_HMAX * 1e3, color="k", lw=0.6, ls="--"); axs[1].axhline(-pr.RW_HMAX * 1e3, color="k", lw=0.6, ls="--")
    axs[1].set(ylabel="wheel momentum [mN m s]"); axs[1].legend(fontsize=7, ncol=6, loc="lower left")
    for i, c in enumerate("rgb"):
        axs[2].plot(to, rm["m"][:, i], c, lw=0.6, label=f"m_{'xyz'[i]}")
        axs[3].plot(to, rm["u"][:, i] * 1e3, c, lw=0.6, label=f"u_{'xyz'[i]}")
    axs[2].set(ylabel="unloading dipole [A m^2]"); axs[2].legend(fontsize=8, ncol=3)
    axs[3].set(ylabel="wheel torque [mN m]\n(zoom; acquisition peak 0.12)", xlabel="time [orbits]", ylim=(-0.03, 0.03)); axs[3].legend(fontsize=8, ncol=3)
    sh = np.array([not s for s in rm["sun"]]).astype(int)
    on = list(np.where(np.diff(sh) == 1)[0] + 1); off = list(np.where(np.diff(sh) == -1)[0] + 1)
    if sh[0]: on = [0] + on
    if len(off) < len(on): off.append(len(sh) - 1)
    for a in axs:
        for a0, a1 in zip(on, off):
            a.axvspan(to[a0], to[a1], color="0.9", zorder=0)
        a.grid(alpha=0.3, which="both")
    fig.tight_layout(); fig.savefig("figures/step4_nadir.png", dpi=130)

    fig, axs = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    rp, rq = R["pass_mekf"], R["pass_truth"]
    tp = (rp["t"] - T_AOS) / 60
    axs[0].plot(tp, rp["el"], "k"); axs[0].axhline(10, color="g", ls="--", lw=0.8)
    axs[0].text(tp[sw] - 0.3, 25, "target mode\n(orange lines)", ha="right", fontsize=8, color="C1")
    axs[0].set(ylabel="Athens elevation [deg]", ylim=(-30, 40),
               title="Ground-target pass over Athens: nadir -> track target (elevation > -10 deg) -> nadir")
    axs[1].semilogy(tp, np.degrees(rp["zerr"]), "k", lw=0.7, label="boresight error, MEKF in the loop")
    axs[1].semilogy(tp, np.degrees(rq["zerr"]), "C0", lw=0.7, label="boresight error, perfect knowledge")
    axs[1].set(ylabel="boresight error [deg]", ylim=(1e-3, 100)); axs[1].legend(fontsize=8, loc="upper left")
    for i, c in enumerate("rgb"):
        axs[2].plot(tp, np.degrees(rp["w"][:, i]), c, lw=0.8, label=f"omega_{'xyz'[i]}")
        axs[3].plot(tp, rp["hw"][:, i] * 1e3, c, lw=0.8, label=f"h_{'xyz'[i]}")
    axs[2].set(ylabel="body rate [deg/s]"); axs[2].legend(fontsize=8, ncol=3)
    axs[3].axhline(pr.RW_HMAX * 1e3, color="k", lw=0.6, ls="--"); axs[3].axhline(-pr.RW_HMAX * 1e3, color="k", lw=0.6, ls="--")
    axs[3].set(ylabel="wheel momentum [mN m s]", xlabel="minutes from AOS (elevation 0)"); axs[3].legend(fontsize=8, ncol=3)
    for a in axs:
        a.axvspan(0, (T_LOS - T_AOS) / 60, color="C2", alpha=0.08, zorder=0)
        a.axvline(tp[sw], color="C1", lw=0.8, ls=":"); a.axvline(tp[sw_end], color="C1", lw=0.8, ls=":")
        a.grid(alpha=0.3, which="both")
    axs[0].set_xlim(-25, tp[-1])
    fig.tight_layout(); fig.savefig("figures/step4_pass.png", dpi=130)
    print(f"\nfigures written; total {time.time()-t_start:.0f} s")
