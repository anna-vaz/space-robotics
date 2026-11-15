"""Step 6: final margins table + summary figures for the README.

Reads the result files in results/ (SOLIDWORKS: step3/step4/stepB/step5 *.txt, Ansys: ansys_*.txt, fasteners_5020*.txt)
and writes
  results/margins_summary.md        one table, every load case, governing item, margin, hand check
  figures/step6_design_history.png  v1 -> v5: what each iteration fixed
  figures/step6_correlation.png     SOLIDWORKS vs Ansys vs hand calculation
  figures/step6_margins.png         minimum margin per check (design case and light case)
Run: ../.venv/Scripts/python.exe step6_summary.py
"""
import math, os, re
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
R = lambda f: open(os.path.join(HERE, "results", f), errors="ignore").read()
FIG = os.path.join(HERE, "figures")
FTY = {"6061": 241.0, "7075": 462.0}          # MMPDS A-basis, MPa
FSY, FSU = 1.25, 1.4

def num(pat, txt, g=1):
    m = re.search(pat, txt, re.M)
    return float(m.group(g)) if m else float("nan")

# ------------------------------------------------------------------ SOLIDWORKS results per version (10 g, cantilever)
def sw_static(tag, ax):
    f = {"v1": "step3_cantilever_h4.txt", "v2": "step3_v2_cantilever_h4.txt"}.get(tag, f"step3_{tag}_cantilever_h4_{ax}.txt")
    if tag == "v3" and ax == "X": f = "step3_v3_cantilever_h4.txt"
    t = R(f)
    return (num(rf"\[{ax}\] max displacement ([0-9.]+)", t), num(rf"\[{ax}\] max von Mises ([0-9.]+)", t))
def sw_f1(f):
    t = R(f)
    return num(r"\[cantilever\]\s+1\s+([0-9.]+)", t)
hist = {v: {ax: sw_static(v, ax) for ax in "XYZ"} for v in ("v1", "v2", "v3", "v4", "v5")}
f1 = {"v2": sw_f1("step4_v2_h4.txt"), "v3": sw_f1("step4_v3_h4.txt"), "v4": sw_f1("step4_v4_cantilever_h4.txt"),
      "v5": sw_f1("step4_v5_cant_h4.txt")}
rv_rms = lambda f: num(r"RMS von Mises raw \(err 0\): \S+, \S+, \S+, ([0-9.]+)", R(f))
rv_v4_rms, rv_v5_rms = rv_rms("step5_rv_v4_h6_n40a_Y.txt"), rv_rms("step5_rv_v5_h6_n40a_Y.txt")   # same method + 6 mm mesh

# ------------------------------------------------------------------ Ansys results (v5)
def ans_table(f):
    rows = []
    for m in re.finditer(r"^(.{48}) (6061|7075)\s+(\S+)\s+([0-9.]+)\s+([+-][0-9.]+)\s+([+-][0-9.]+)", R(f), re.M):
        rows.append((m.group(1).strip(), m.group(2), float(m.group(4)), float(m.group(5)), float(m.group(6))))
    return rows
psd = {ax: ans_table(f"ansys_psd_{ax}.txt") for ax in "XYZ"}
psd_l = {ax: ans_table(f"ansys_psd_{ax}_light.txt") for ax in "XYZ"}
stat = {ax: ans_table(f"ansys_static_{ax}.txt") for ax in "XYZ"}
umax = {ax: num(r"displacement \(relative\) ([0-9.]+)", R(f"ansys_psd_{ax}.txt")) for ax in "XYZ"}
mod_b = R("ansys_modal_bothends.txt"); mod_c = R("ansys_modal_cantilever.txt"); mod_l = R("ansys_modal_bothends_light.txt")
def mode_f(txt, i): return num(rf"^\s+{i}\s+([0-9.]+)", txt)
fas = R("fasteners_5020.txt"); fas_l = R("fasteners_5020_light.txt")
fmin = lambda t, k: num(rf"{k} ([+-][0-9.]+)", t)

# ------------------------------------------------------------------ buckling (v5 if available, else v4)
def blf(tag, ax):
    try: t = R(f"stepB_buckling_{tag}_nopanels_{ax}.txt")
    except FileNotFoundError: return float("nan")
    m = re.search(r"raw load factors \(err 0\): (.*)", t)
    v = [abs(float(x)) for x in m.group(1).split(",")[1::2]] if m else []
    return min(v) if v else float("nan")
btag = "v5" if os.path.exists(os.path.join(HERE, "results", "stepB_buckling_v5_nopanels_Y.txt")) else "v4"
B = {ax: blf(btag, ax) for ax in "XYZ"}
BREQ = {"X": 10.6, "Y": 10.6, "Z": 15.7}          # 3-sigma random load x 1.4 / 10 g (hand_calcs.py)

# ------------------------------------------------------------------ hand checks
miles = math.sqrt(math.pi / 2 * mode_f(mod_b, 7) * 10 * 0.16)           # stack lateral mode, Q = 10
worst = lambda rows: min(rows, key=lambda r: r[3])
wy, wy_s = worst(psd["Y"]), worst(stat["Y"])
fe_g = 10 * (wy[2] / 3) / wy_s[2]                                        # 1-sigma g implied by FE

# ------------------------------------------------------------------ margins table
L = ["# Project 7 — margins of safety, design v5 (4.000 kg)", "",
     "Factors of safety: 1.25 yield, 1.4 ultimate (NASA-STD-5001B, analysis only); random vibration design stress = 3σ.",
     "Allowables: MMPDS A-basis 6061-T6 Fty 241 / Ftu 290 MPa, 7075-T6 Fty 462 / Ftu 524 MPa. MS = allowable / (FS × stress) − 1.", "",
     "| Load case | Requirement | Governing item | Result | Margin (yield / ult) | Independent check |",
     "|---|---|---|---|---|---|"]
cf1 = f1["v5"]
L.append(f"| Modal, cantilever (SOLIDWORKS) | f1 > 100 Hz | board-stack local mode | {cf1:.0f} Hz; first global bending {mode_f(mod_c, 1):.0f}–235 Hz | f1 / 100 Hz = {cf1 / 100:.2f} | Ansys {mode_f(mod_c, 1):.0f} Hz Y bending (−8 %); Timoshenko hand calc 255 Hz |")
for ax in "XYZ":
    w = worst(stat[ax]); sw = hist["v5"][ax]
    L.append(f"| 10 g quasi-static {ax} (Ansys) | FS 1.25 / 1.4 | {w[0]} | {w[2]:.1f} MPa | {w[3]:+.2f} / {w[4]:+.2f} | SOLIDWORKS {sw[1]:.1f} MPa; reaction 392.26 N = m·10 g |")
for ax in "XYZ":
    L.append(f"| Buckling at 10 g, {ax} (SOLIDWORKS {btag}, no panels) | BLF ≥ {BREQ[ax]} | first mode | BLF {B[ax]:.1f} | {B[ax] / BREQ[ax] - 1:+.2f} | hand (step 1): rails alone between 1U frames +0.78 (lower bound) |")
for ax in "XYZ":
    w = worst(psd[ax])
    chk = f"Miles {miles:.1f} g vs FE {fe_g:.1f} g (1σ)" if ax == "Y" else ""
    L.append(f"| Random vibration GEVS qual {ax} (Ansys PSD) | FS 1.25 / 1.4 on 3σ | {w[0]} | {w[2]:.1f} MPa 3σ | {w[3]:+.2f} / {w[4]:+.2f} | {chk} |")
L.append(f"| M3 screws, all spacer ends, 3 axes (NASA-STD-5020A) | separation 1.2, yield 1.25, ult 1.4 | 13 mm spacer, bottom, Y | Pt 623 N | sep {fmin(fas, 'separation'):+.2f}, yield {fmin(fas, 'yield'):+.2f}, ult {fmin(fas, 'ultimate'):+.2f} | hand: Pt = F + M/r |")
L.append(f"| Light case 3.565 kg, random vibration Y | as above | 13 mm spacer | {worst(psd_l['Y'])[2]:.1f} MPa 3σ | {worst(psd_l['Y'])[3]:+.2f} / {worst(psd_l['Y'])[4]:+.2f} | screws sep {fmin(fas_l, 'separation'):+.2f}, yield {fmin(fas_l, 'yield'):+.2f}; f1 +7.2 % |")
L += ["", "Every margin is positive. The tightest are the M3 screw preload window (+0.09 yield, +0.10 separation) and the",
      "spacers above the 1U rib in lateral random vibration (+0.59; about +0.49 with the tapped bore). Static 10 g is not a",
      "sizing case (margins > +11). The 4.000 kg design case governs; the light case has higher margins everywhere."]
open(os.path.join(HERE, "results", "margins_summary.md"), "w", encoding="utf8").write("\n".join(L) + "\n")
print("\n".join(L))

# ------------------------------------------------------------------ figures
plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": 0.3})
V = ["v1", "v2", "v3", "v4", "v5"]
fig, ax = plt.subplots(1, 3, figsize=(14, 4.3))
lat = [max(hist[v]["X"][0], hist[v]["Y"][0]) for v in V]
ax[0].bar(V, lat, color=["#c0392b", "#e67e22", "#27ae60", "#27ae60", "#27ae60"])
ax[0].axhline(0.55, color="k", ls="--", lw=1); ax[0].text(4.4, 0.6, "PCB-wall clearance 0.55 mm", ha="right", fontsize=8)
ax[0].set_yscale("log"); ax[0].set_ylabel("max lateral deflection at 10 g (mm)")
ax[0].set_title("Static 10 g (SOLIDWORKS)")
for i, v in enumerate(lat): ax[0].text(i, v * 1.15, f"{v:.2f}", ha="center", fontsize=8)
fv = ["v2", "v3", "v4", "v5"]
ax[1].bar(fv, [f1[v] for v in fv], color=["#c0392b", "#27ae60", "#27ae60", "#27ae60"])
ax[1].axhline(100, color="k", ls="--", lw=1); ax[1].text(3.4, 106, "requirement 100 Hz", ha="right", fontsize=8)
ax[1].set_ylabel("first natural frequency, cantilever (Hz)"); ax[1].set_title("Modal (SOLIDWORKS)")
for i, v in enumerate(fv): ax[1].text(i, f1[v] + 4, f"{f1[v]:.0f}", ha="center", fontsize=8)
s3 = [3 * rv_v4_rms, 3 * rv_v5_rms, wy[2]]
ax[2].bar(["v4\nSOLIDWORKS\n6 mm mesh", "v5\nSOLIDWORKS\n6 mm mesh", "v5\nAnsys\nbeam spacers"], s3,
          color=["#c0392b", "#e67e22", "#27ae60"])
ax[2].text(1, 3 * rv_v5_rms + 40, "joint peak,\nmesh-dependent", ha="center", fontsize=7)
ax[2].set_ylim(0, 640)
ax[2].axhline(FTY["6061"] / FSY, color="k", ls="--", lw=1)
ax[2].text(2.4, FTY["6061"] / FSY + 12, f"allowable {FTY['6061'] / FSY:.0f} MPa (6061, FS 1.25)", ha="right", fontsize=8)
ax[2].set_ylabel("3σ stress, worst spacer, Y (MPa)"); ax[2].set_title("Random vibration GEVS qual")
for i, v in enumerate(s3): ax[2].text(i, v + 8, f"{v:.0f}", ha="center", fontsize=8)
notes = {"v1": "v1\nstack on\nstandoffs", "v2": "v2\n+ bulkheads\nat 1U ribs", "v3": "v3\n+ X-braces\nin pockets",
         "v4": "v4\nbraces 3 mm\n+ panel pads", "v5": "v5\n+ wheel\nbulkhead"}
ax[0].set_xticks(range(5)); ax[0].set_xticklabels([notes[v] for v in V], fontsize=8)
ax[0].set_ylim(0.03, 5)
fig.suptitle("Design iterations v1 → v5: each version fixes the failure found in the previous one", fontsize=11)
fig.tight_layout(); fig.savefig(os.path.join(FIG, "step6_design_history.png"), dpi=130); plt.close(fig)

fig, ax = plt.subplots(1, 2, figsize=(13, 4.3))
cats = ["modal both ends\nlateral Y", "modal both ends\nlateral X", "modal cantilever\nbending Y", "modal cantilever\nbending X"]
sw_m = [319.0, 321.0, 234.8, 261.6]
an_m = [mode_f(mod_b, 7), mode_f(mod_b, 8), mode_f(mod_c, 1), mode_f(mod_c, 4)]
x = range(len(cats))
ax[0].bar([i - 0.2 for i in x], sw_m, 0.4, label="SOLIDWORKS (solid, 400k nodes)")
ax[0].bar([i + 0.2 for i in x], an_m, 0.4, label="Ansys (shell/beam, 62k nodes)")
for i in x: ax[0].text(i, max(sw_m[i], an_m[i]) + 6, f"{(an_m[i] / sw_m[i] - 1) * 100:+.0f} %", ha="center", fontsize=8)
ax[0].set_xticks(list(x)); ax[0].set_xticklabels(cats, fontsize=8); ax[0].set_ylabel("Hz"); ax[0].set_ylim(0, 440)
ax[0].legend(fontsize=8, loc="upper right", ncol=2)
ax[0].set_title("Natural frequencies")
cats = ["10 g X", "10 g Y", "10 g Z", "RV Y 1σ\nload (g)"]
sw_s = [hist["v5"][a][1] for a in "XYZ"] + [float("nan")]
an_s = [worst(stat[a])[2] for a in "XYZ"] + [fe_g]
hand = [float("nan")] * 3 + [miles]
ax[1].bar([i - 0.27 for i in x], sw_s, 0.27, label="SOLIDWORKS max von Mises (MPa)")
ax[1].bar([i for i in x], an_s, 0.27, label="Ansys (MPa / g)")
ax[1].bar([i + 0.27 for i in x], hand, 0.27, label="hand: Miles equation (g)", color="#7f8c8d")
ax[1].set_xticks(list(x)); ax[1].set_xticklabels(cats, fontsize=8); ax[1].legend(fontsize=8, loc="upper left")
ax[1].set_title("Static stress and random-vibration load")
for i in x:
    for dx, val in ((-0.27, sw_s[i]), (0, an_s[i]), (0.27, hand[i])):
        if val == val: ax[1].text(i + dx, val + 0.5, f"{val:.1f}", ha="center", fontsize=7)
fig.suptitle("Cross-check: two independent FE models and a hand calculation", fontsize=11)
fig.tight_layout(); fig.savefig(os.path.join(FIG, "step6_correlation.png"), dpi=130); plt.close(fig)

items = [("Screws: yield (preload window)", fmin(fas, "yield"), fmin(fas_l, "yield")),
         ("Screws: joint separation", fmin(fas, "separation"), fmin(fas_l, "separation")),
         ("Spacers: random vibration Y", worst(psd["Y"])[3], worst(psd_l["Y"])[3]),
         ("Spacers: random vibration X", worst(psd["X"])[3], worst(psd_l["X"])[3]),
         ("Wheel bulkhead: random vibration Z", worst(psd["Z"])[3], worst(psd_l["Z"])[3]),
         ("Screws: ultimate tension + shear", fmin(fas, "ultimate"), fmin(fas_l, "ultimate")),
         (f"Buckling at 10 g (worst axis, {btag})", min(B[a] / BREQ[a] - 1 for a in "XYZ"), float("nan")),
         ("Static 10 g (worst axis)", min(worst(stat[a])[3] for a in "XYZ"), float("nan"))]
fig, ax = plt.subplots(figsize=(9, 4.4))
y = range(len(items))[::-1]
ax.barh([i + 0.18 for i in y], [it[1] for it in items], 0.36, label="design case 4.000 kg", color="#2980b9")
ax.barh([i - 0.18 for i in y], [it[2] for it in items], 0.36, label="light case 3.565 kg", color="#95a5a6")
for i, it in zip(y, items): ax.text(it[1] * 1.08 + 0.03, i + 0.18, f"{it[1]:+.2f}", va="center", fontsize=8)
ax.set_yticks(list(y)); ax.set_yticklabels([it[0] for it in items], fontsize=9)
ax.set_xscale("symlog", linthresh=1); ax.set_xlim(0, 60); ax.axvline(0, color="k")
ax.set_xlabel("margin of safety on yield (0 = just meets the factor of safety)"); ax.legend(fontsize=8, loc="upper right")
ax.set_title("Minimum margins, v5 — all positive; the bolted joints size the structure")
fig.tight_layout(); fig.savefig(os.path.join(FIG, "step6_margins.png"), dpi=130); plt.close(fig)
print("figures written")
