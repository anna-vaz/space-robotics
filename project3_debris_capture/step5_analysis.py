"""Project 3, step 5a (analysis): Monte Carlo results -> statistics and figure."""
import json
import os

import matplotlib.pyplot as plt
import numpy as np

import capture as cp

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, "figures")
R = [json.loads(line) for line in open(os.path.join(HERE, "results_step5_mc.jsonl"))]
n = len(R)
ok = np.array([r["pass"] for r in R])
cap_ok = np.array([r["L_ratio"] < 1 for r in R])           # momentum budget says: wheel can hold it
print(f"{n} missions: {ok.sum()} passed ({100*ok.mean():.1f} %)")
print(f"  grasp held in {sum(r['grasp_ok'] for r in R)}/{n}; solver ok in {sum(r['solver_ok'] for r in R)}/{n}")
print(f"  momentum budget (L < wheel capacity) satisfied in {cap_ok.sum()}/{n}")
agree = np.sum(ok == cap_ok)
print(f"  pass/fail predicted by the budget alone: {agree}/{n} missions")


def return_time(r):
    """Refined budget: after absorbing L, the spare wheel momentum limits the slew back,
    (H - L)/I_sys; plus ~200 s for the attitude loop to settle."""
    if r["L"] >= cp.H_MAX:
        return np.inf
    slew = (cp.H_MAX - r["L"]) / r["I_sys"]
    return max(abs(np.radians(r["att_at_grasp_deg"])) / slew, 0.0) + 200.0


pred2 = np.array([return_time(r) < 900.0 for r in R])
print(f"  refined budget (also the time to slew back with the spare momentum): "
      f"{np.sum(ok == pred2)}/{n} missions predicted correctly")
for name, mask in (("budget OK but FAILED", cap_ok & ~ok), ("budget exceeded but PASSED", ~cap_ok & ok),
                   ("refined budget wrong", pred2 != ok)):
    sel = [r for r, m in zip(R, mask) if m]
    print(f"  {name}: {len(sel)}")
    for r in sel[:8]:
        print(f"     seed {r['seed']}: L/H {r['L_ratio']:.3f}, final rate {r['final_rate_deg']:.2e} deg/s, "
              f"final att {r['final_att_deg']:.2f} deg, att at grasp {r['att_at_grasp_deg']:.1f} deg, "
              f"predicted return time {return_time(r):.0f} s")
g = [r for r in R if r["grasp_ok"]]
print(f"  peak grasp force: max {max(r['peak_force'] for r in R):.2f} N (limit 10); "
      f"peak torque max {max(r['peak_torque'] for r in R):.2f} N m; "
      f"max stretch {1000*max(r['stretch'] for r in R):.2f} mm")
print(f"  angular momentum drift: max {max(r['L_drift'] for r in R):.1e}")
fails = [r for r in R if not r["pass"] and r["L_ratio"] > 1]
if fails:
    res = np.array([abs(r["final_rate_deg"]) for r in fails])
    pred = np.array([np.degrees((r["L"] - cp.H_MAX) / r["I_sys"]) for r in fails])
    print(f"  over-budget missions: residual spin vs (L - H)/I_sys: max difference "
          f"{np.abs(res - pred).max():.1e} deg/s")

# ---- figure: where capture works, in the (target inertia, spin) plane -----------
fig, ax = plt.subplots(1, 2, figsize=(13, 5))
I = np.array([r["I_t"] for r in R]); w = np.array([r["spin"] for r in R])
ax[0].scatter(I[ok], w[ok], c="C2", s=22, label="captured and stopped")
ax[0].scatter(I[~ok], w[~ok], c="C3", marker="x", s=30, label="failed")
Ig = np.linspace(I.min() * 0.95, I.max() * 1.05, 100)
ax[0].plot(Ig, np.degrees(cp.H_MAX / Ig), "k--", lw=1.5, label="I_target x spin = wheel capacity")
ax[0].set_xlabel("target inertia [kg m^2]"); ax[0].set_ylabel("tumble rate [deg/s]")
ax[0].set_title(f"Monte Carlo ({n} missions): capture envelope"); ax[0].grid(alpha=0.3)
ax[0].legend(fontsize=8)
Lr = np.array([r["L_ratio"] for r in R])
fr = np.array([abs(r["final_rate_deg"]) for r in R]) + 1e-12
ax[1].semilogy(Lr[ok], fr[ok], "o", color="C2", ms=4, label="passed")
ax[1].semilogy(Lr[~ok], fr[~ok], "x", color="C3", ms=6, label="failed")
ax[1].axvline(1.0, color="k", ls="--", lw=1, label="wheel capacity")
ax[1].axhline(0.01, color="gray", ls=":", label="success threshold 0.01 deg/s")
ax[1].set_xlabel("total angular momentum / wheel capacity"); ax[1].set_ylabel("spin after 900 s [deg/s]")
ax[1].set_title("Residual spin vs momentum budget"); ax[1].grid(alpha=0.3, which="both")
ax[1].legend(fontsize=8)
fig.tight_layout()
fig.savefig(os.path.join(FIG, "step5_montecarlo.png"), dpi=150)
print("saved", os.path.join(FIG, "step5_montecarlo.png"))
