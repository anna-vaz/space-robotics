"""Dimensioned guide figure for CAD part 1 (side frame with integral rails)."""
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, FancyBboxPatch

L, W, T, F, FOOT = 340.5, 100.0, 1.5, 8.5, 7.0
prof = [(0,0),(100,0),(100,8.5),(98.5,8.5),(98.5,1.5),(1.5,1.5),(1.5,8.5),(0,8.5)]
area = 0.5*abs(sum(x1*y2-x2*y1 for (x1,y1),(x2,y2) in zip(prof, prof[1:]+prof[:1])))
foot_add = (F*F - (F*T + T*(F-T))) * FOOT * 4
pockets = [(12,109.5),(117.5,223),(231,328.5)]
pa = sum(83*(b-a) for a,b in pockets) - 12*(1-np.pi/4)*3**2
fil = 2*(1-np.pi/4)*1**2*L
V = area*L + foot_add - pa*T - fil
print(f"profile area {area:.1f} mm2, extrude {area*L:.1f} mm3, feet +{foot_add:.0f}, pockets -{pa*T:.0f}, "
      f"fillets -{fil:.0f} -> V = {V:.0f} mm3, mass 7075 = {V*2.81e-3:.1f} g")

fig, ax = plt.subplots(1, 2, figsize=(15, 5.2), gridspec_kw=dict(width_ratios=[1, 1.6]))
a = ax[0]
a.add_patch(Polygon(prof, closed=True, fc="#9ecae1", ec="k", lw=1.5, label="profile (extrude 340.5)"))
for x0 in (0, 91.5):
    a.add_patch(plt.Rectangle((x0, 0), 8.5, 8.5, fc="none", ec="C3", lw=1.5, ls="--"))
a.plot([], [], "C3--", label="foot 8.5 x 8.5, 7 mm long, both ends")
a.plot(0, 0, "ko", ms=6); a.annotate("origin = outer rail corner", (0, 0), (9, -4.5), fontsize=9,
                                     arrowprops=dict(arrowstyle="->"))
a.text(14, 0.35, "plate 1.5 thick, continues to x = 100", fontsize=9)
a.text(11, 4.5, "foot 8.5 x 8.5 (dashed)\nfills the corner for 7 mm\nat both ends", fontsize=9, color="C3")
a.text(-2.2, 4.0, "8.5", fontsize=9, ha="center")
a.text(0.75, 9.0, "1.5", fontsize=9, ha="center")
a.annotate("R1 on this long edge\n(CDS 2.2.7)", (0, 0), (-4.5, -5.5), fontsize=9,
           arrowprops=dict(arrowstyle="->", color="0.4"))
a.set(xlim=(-5, 28), ylim=(-6, 12), aspect="equal", xlabel="x (mm)", ylabel="y (mm)",
      title="A. Cross-section, left rail corner (Front plane)\nright rail at x = 91.5..100 is its mirror image")
a.legend(loc="upper right", fontsize=8)

b = ax[1]
b.add_patch(plt.Rectangle((0, 0), L, W, fc="#9ecae1", ec="k", lw=1.5))
for z0, z1 in pockets:
    b.add_patch(FancyBboxPatch((z0+3, 8.5+3), z1-z0-6, 83-6, boxstyle="round,pad=3", fc="white", ec="k"))
    b.text((z0+z1)/2, 50, f"pocket\n{z1-z0:.1f} x 83\nR3 corners", ha="center", va="center", fontsize=9)
for zc in (113.5, 227):
    b.text(zc, 103, f"rib 8 mm\nat {zc} (1U)", ha="center", fontsize=8)
for z, txt in [(6, "12"), (334.5, "12")]:
    b.text(z, 103, txt + " end\nband", ha="center", fontsize=8)
b.text(L/2, 2.5, "rail strip 8.5 (no pocket)", ha="center", fontsize=8)
b.text(L/2, 93.5, "rail strip 8.5 (no pocket)", ha="center", fontsize=8)
b.set(xlim=(-10, L+10), ylim=(-8, 115), aspect="equal", xlabel="z (mm)  along the CubeSat",
      ylabel="x (mm)", title="B. Outer face (Top plane, y = 0): through-pockets")
b.grid(alpha=.3)
fig.suptitle(f"CAD part 1: side frame with integral rails, 7075-T6, x2   (expected mass {V*2.81e-3:.1f} g)")
fig.tight_layout(); fig.savefig("cad/part1_side_frame_guide.png", dpi=140)
