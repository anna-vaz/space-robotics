"""Step 4b (analysis): statistics and figures from results_step4b.json."""
import json
import os
import numpy as np
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, "figures")
with open(os.path.join(HERE, "results_step4b.json")) as f:
    D = json.load(f)
REQ = D["req"]
CONE = 10.0

METRICS = [  # key, label, scale, upper limit (or window)
    ("v_axial", "closing speed [cm/s]", 100, (REQ["v_axial"][0] * 100, REQ["v_axial"][1] * 100)),
    ("lateral", "lateral offset [cm]", 100, REQ["lateral"] * 100),
    ("v_lateral", "lateral speed [mm/s]", 1000, REQ["v_lateral"] * 1000),
    ("angle", "angular misalignment [deg]", 1, REQ["angle"]),
    ("rate", "angular rate [deg/s]", 1, REQ["rate"]),
    ("max_cone", "max approach-cone angle [deg]", np.degrees(1), CONE),
]


def campaign(scale):
    return [r for r in D["runs"] if r["scale"] == scale]


def summary(name, runs):
    n = len(runs)
    n_pass = sum(r["pass"] for r in runs)
    print(f"\n=== {name}: {n_pass}/{n} missions passed all checks ({100 * n_pass / n:.1f} %) ===")
    if n_pass == n:
        print(f"    (zero failures in {n}: 95 % upper bound on the failure rate ~ 3/n = {300 / n:.1f} %)")
    contact = [r for r in runs if r["contact"]]
    print(f"    contact reached in {len(contact)}/{n}")
    for key, label, sc, lim in METRICS:
        v = np.array([r[key] for r in contact]) * sc
        hi = lim[1] if isinstance(lim, tuple) else lim
        lo_txt = f"{lim[0]:.1f}-" if isinstance(lim, tuple) else "< "
        print(f"    {label:32s} mean {v.mean():8.4f}  99th pct {np.percentile(v, 99):8.4f}  "
              f"worst {v.max():8.4f}  (req {lo_txt}{hi:g})  fails {np.sum(v > hi) + (np.sum(v < lim[0]) if isinstance(lim, tuple) else 0)}")
    dv = np.array([r["dv"] for r in runs])
    print(f"    delta-v [m/s]: mean {dv.mean():.2f}, 99th pct {np.percentile(dv, 99):.2f}, "
          f"max {dv.max():.2f}  (nominal {D['nominal']['dv']:.2f})")


A, B = campaign(1.0), campaign(3.0)
C_path = os.path.join(HERE, "results_step4b_retuned.json")
C = json.load(open(C_path))["runs"] if os.path.exists(C_path) else []
nom = D["nominal"]
print(f"Nominal (no errors, 1 Hz control): dv {nom['dv']:.3f} m/s, lateral {nom['lateral']*100:.4f} cm")
summary("Campaign A: nominal uncertainty (x1)", A)
summary("Campaign B: stress test (x3 errors)", B)
if C:
    summary("Campaign C: x3 errors, Kalman filter retuned", C)
    worse = sum(1 for b in B for c in C if b["seed"] == c["seed"] and b["pass"] and not c["pass"])
    print(f"    missions that passed in B but fail in C: {worse}")

# ---- Figure 1: histograms of contact conditions ---------------------------------
fig, ax = plt.subplots(2, 4, figsize=(15, 7))
items = METRICS + [("dv", "delta-v [m/s]", 1, None)]
for a, (key, label, sc, lim) in zip(ax.flat, items):
    sets = [(A, "C0", "x1 errors"), (B, "C3", "x3 errors"), (C, "C2", "x3, filter retuned")]
    sets = [st for st in sets if st[0]]
    vals = [np.array([r[key] for r in runs if r["contact"]]) * sc for runs, _, _ in sets]
    bins = np.linspace(min(v.min() for v in vals), max(v.max() for v in vals), 30)
    for v, (runs, col, lab) in zip(vals, sets):
        a.hist(v, bins, alpha=0.5, color=col, label=f"{lab} ({len(runs)})")
    if lim is not None:
        for lv in (lim if isinstance(lim, tuple) else (lim,)):
            if bins[0] <= lv <= bins[-1] * 1.5:
                a.axvline(lv, color="k", ls="--", lw=1.5, label="requirement")
    a.set_title(label, fontsize=10); a.grid(alpha=0.3)
ax.flat[0].legend(fontsize=8)
ax.flat[-1].axis("off")
ax.flat[-1].text(0.0, 0.5,
                 f"Pass rate\n  x1 errors: {sum(r['pass'] for r in A)}/{len(A)}\n"
                 f"  x3 errors: {sum(r['pass'] for r in B)}/{len(B)}\n"
                 + (f"  x3, filter retuned: {sum(r['pass'] for r in C)}/{len(C)}\n" if C else "") + "\n"
                 "Dashed line = requirement\n(shown when inside the plotted range)",
                 fontsize=12, va="center")
fig.suptitle("Monte Carlo: contact conditions over all missions")
fig.tight_layout()
fig.savefig(os.path.join(FIG, "step4b_histograms.png"), dpi=150)

# ---- Figure 2: final approaches inside the cone + navigation error -------------
fig, ax = plt.subplots(1, 3, figsize=(16, 4.5))
for a, k, lab in ((ax[0], 7, "x: radial"), (ax[1], 9, "z: cross-track")):
    yy = np.linspace(-12, -1, 50)
    half = np.tan(np.radians(CONE)) * (-1 - yy) * 100
    a.fill_between(yy, -half, half, color="C2", alpha=0.15, label="10 deg cone")
    first = True
    for r in A:
        if r.get("hist"):
            h = np.array(r["hist"])
            sel = h[:, 0] > 3000
            a.plot(h[sel, 8], h[sel, k] * 100, "C0", lw=0.7, alpha=0.6,
                   label="25 random missions (x1)" if first else None)
            first = False
    a.axvline(-1, color="k", lw=2)
    a.set_ylim(-40, 40); a.set_xlabel("y: along-track [m]"); a.set_ylabel(f"{lab} [cm]")
    a.grid(alpha=0.3); a.legend(fontsize=8, loc="lower left")
ax[0].set_title("Final approach, in-plane (docking port)"); ax[1].set_title("Final approach, out-of-plane")
r = next(r for r in A if r.get("est_err"))
e = np.array(r["est_err"]); h = np.array(r["hist"])
n = min(len(e), len(h))
rng_ = np.linalg.norm(h[:n, 1:4], axis=1)
ax[2].semilogy(h[:n, 0] / 60, np.linalg.norm(e[:n, :3], axis=1), "C0", lw=0.8, label="Kalman filter position error")
ax[2].semilogy(h[:n, 0] / 60, np.sqrt(3) * (0.01 + 0.005 * rng_), "k:", label="raw sensor noise (1-sigma, 3D)")
ax[2].set_xlabel("time [min]"); ax[2].set_ylabel("m"); ax[2].grid(alpha=0.3, which="both")
ax[2].set_title("Navigation: filter vs raw measurement"); ax[2].legend(fontsize=8)
fig.tight_layout()
fig.savefig(os.path.join(FIG, "step4b_approaches_navigation.png"), dpi=150)
print("\nsaved figures to", FIG)
