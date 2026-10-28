"""Project 6 step 5a analysis: pointing budget from results_step5_mc.jsonl.
Run:  ../.venv/Scripts/python.exe step5_analysis.py
"""
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import params as pr

with open("results_step5_mc.jsonl") as fh:
    R = [json.loads(line) for line in fh if line.strip()]
R.sort(key=lambda r: r["seed"])
n = len(R)
get = lambda k: np.array([r[k] for r in R])

acq = get("acq")
steady_pe, steady_kn, steady_sun, per_rms, per_kn, per_max, exc = [], [], [], [], [], [], []
for r in R:
    pe, kn, sun = np.array(r["pe"]), np.array(r["know"]), np.array(r["sun"], bool)
    ss = np.arange(len(pe)) >= r["acq"] + 300          # tracking phase: 5 min after acquisition
    steady_pe.append(pe[ss]); steady_kn.append(kn[ss]); steady_sun.append(sun[ss])
    per_rms.append(np.sqrt(np.mean(pe[ss]**2))); per_kn.append(np.sqrt(np.mean(kn[ss]**2)))
    per_max.append(pe[ss].max()); exc.append(int(np.sum(pe[ss] > 1.0)))
PE, KN, SUN = map(np.concatenate, (steady_pe, steady_kn, steady_sun))
per_rms, per_kn, per_max, exc = map(np.array, (per_rms, per_kn, per_max, exc))

print(f"== Monte Carlo pointing budget: {n} runs, 1 orbit of nadir pointing each")
print(f"  acquisition (error < 1 deg for 5 min) from a random attitude, |omega| <= 0.5 deg/s: "
      f"median {np.median(acq):.0f} s, 95th pct {np.percentile(acq, 95):.0f} s, max {acq.max():.0f} s")
print(f"  tracking phase, pooled over runs ({len(PE)} s of flight):")
for p in (50, 95, 99, 99.73):
    print(f"    {p:5.2f} %  pointing <= {np.percentile(PE, p):.2f} deg   (knowledge <= {np.percentile(KN, p):.2f})")
print(f"    sunlit rms {np.sqrt(np.mean(PE[SUN]**2)):.3f} deg, eclipse rms {np.sqrt(np.mean(PE[~SUN]**2)):.3f} deg")
print(f"  time above 1 deg: {exc.sum()} s in total = {100*exc.sum()/len(PE):.3f} % ({np.sum(exc > 0)} of {n} runs; "
      f"worst run {exc.max()} s); worst instantaneous {per_max.max():.2f} deg")
print(f"  per-run rms: median {np.median(per_rms):.3f}, worst {per_rms.max():.3f} deg (seed {R[int(np.argmax(per_rms))]['seed']})")
hw = get("hw_max") * 1e3
print(f"  max wheel momentum: median {np.median(hw):.2f}, worst {hw.max():.2f} mN m s (capacity {pr.RW_HMAX*1e3})")
print(f"  end-of-orbit |h_w|: median {np.median(get('hw_end'))*1e3:.3f}, worst {get('hw_end').max()*1e3:.3f} mN m s")
print(f"  max unloading dipole {get('m_max').max():.4f} A m^2 (limit {pr.MTQ_MAX}); max wheel torque "
      f"{get('u_max').max()*1e3:.3f} mN m (limit {pr.RW_TMAX*1e3})")
print(f"  inertial momentum check, worst run: {get('Hchk').max():.1e} N m s")
print(f"  correlation of per-run pointing rms with: knowledge rms {np.corrcoef(per_kn, per_rms)[0,1]:+.2f}, "
      f"residual dipole {np.corrcoef(get('m_res'), per_rms)[0,1]:+.2f}, CoM offset {np.corrcoef(get('r_cm'), per_rms)[0,1]:+.2f}")

fig, axs = plt.subplots(1, 3, figsize=(13.5, 4))
xs = np.sort(PE)
axs[0].plot(xs, 100 * np.arange(1, len(xs) + 1) / len(xs), "k", label="pointing")
xk = np.sort(KN)
axs[0].plot(xk, 100 * np.arange(1, len(xk) + 1) / len(xk), "C1", lw=0.9, label="knowledge")
for p, c in ((95, "C0"), (99.73, "C3")):
    axs[0].axvline(np.percentile(PE, p), color=c, ls="--", lw=0.8, label=f"pointing {p} %: {np.percentile(PE, p):.2f} deg")
axs[0].set(xlabel="error [deg]", ylabel="fraction of time [%]", xlim=(0, 1.5),
           title=f"Pooled tracking error, {n} runs")
axs[0].legend(fontsize=8, loc="lower right"); axs[0].grid(alpha=0.3)
sc = axs[1].scatter(per_kn, per_rms, c=get("m_res") * 1e3, cmap="viridis", s=18)
plt.colorbar(sc, ax=axs[1], label="residual dipole [mA m^2]")
lim = [0, max(per_rms.max(), per_kn.max()) * 1.1]
axs[1].plot(lim, lim, "k:", lw=0.8)
axs[1].set(xlabel="knowledge error rms [deg]", ylabel="pointing error rms [deg]", xlim=lim, ylim=lim,
           title="Per run: pointing error ~ knowledge error")
axs[1].grid(alpha=0.3)
axs[2].hist(hw, bins=15, color="C0", alpha=0.8)
axs[2].axvline(pr.RW_HMAX * 1e3, color="k", ls="--", label="wheel capacity")
axs[2].set(xlabel="max wheel momentum per run [mN m s]", ylabel="runs", title="Wheel momentum (acquisition peak)")
axs[2].legend(fontsize=8); axs[2].grid(alpha=0.3)
fig.tight_layout(); fig.savefig("figures/step5_montecarlo.png", dpi=130)
print("figure written")
