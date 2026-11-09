"""Guide figure: X-braces in the side-frame pockets (design v3, step-4 finding: Y bending 72 Hz)."""
import numpy as np, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Polygon
L, W, B, T = 340.5, 100.0, 6.0, 1.5
pockets = [(12, 109.5), (117.5, 223), (231, 328.5)]
fig, a = plt.subplots(figsize=(13, 5))
a.add_patch(plt.Rectangle((0, 0), L, W, fc="#9ecae1", ec="k", lw=1.5))
vol = 0
for z0, z1 in pockets:
    a.add_patch(FancyBboxPatch((z0 + 3, 8.5 + 3), z1 - z0 - 6, 83 - 6, boxstyle="round,pad=3", fc="white", ec="k"))
    a.add_patch(plt.Rectangle((z0, 8.5), z1 - z0, 83, fc="none", ec="C2", ls="--", lw=1))
    for p, q in [((z0, 8.5), (z1, 91.5)), ((z0, 91.5), (z1, 8.5))]:
        p, q = np.array(p), np.array(q); d = (q - p) / np.linalg.norm(q - p); n = np.array([-d[1], d[0]]) * B / 2
        a.add_patch(Polygon([p + n, q + n, q - n, p - n], fc="#3182bd", ec="k", lw=.8, alpha=.9))
        a.plot(*zip(p, q), "C3-", lw=1)
    Ld = np.hypot(z1 - z0, 83); vol += 2 * Ld * B * T
    a.text((z0 + z1) / 2, 98.5, f"pocket {z1 - z0:g} x 83: diagonals {Ld:.0f} mm", ha="center", fontsize=8)
for z in [0, 12, 109.5, 117.5, 223, 231, 328.5, 340.5]:
    ha = "right" if z in (109.5, 223) else "left" if z in (117.5, 231) else "center"
    a.text(z + {"right": -1, "left": 1, "center": 0}[ha], -6, f"{z:g}", ha=ha, fontsize=8)
for y in [8.5, 91.5]:
    a.text(-4, y, f"{y:g}", ha="right", va="center", fontsize=8)
a.text(170, -14, "z (mm), along the length", ha="center", fontsize=9)
title = ("Side frame v3, outer face: X-braces corner-to-corner in each pocket\n"
         "(red = sketch line, green dashed = construction rectangle); thin feature "
         f"{B:g} mm wide (mid-plane), {T:g} mm deep, merged; ~{vol * 2.81e-3:.0f} g per frame")
a.set(xlim=(-14, L + 4), ylim=(-17, 104), aspect="equal", title=title)
a.axis("off"); fig.tight_layout(); fig.savefig("cad/part1_braces_guide.png", dpi=130)
print("ok", round(vol * 2.81e-3, 1), "g/frame")
