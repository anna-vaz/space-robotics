"""Guide figure for CAD part 3 (end plate), in box coordinates and plate coordinates."""
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle

PX, PY, T = 97.0, 95.0, 2.0            # plate, origin at its corner (= box X 1.5, Y 2.5)
foot = [(3.0, 3.0), (94.0, 3.0), (3.0, 92.0), (94.0, 92.0)]
bx0, by0 = 50 - 95.89/2 - 1.5, 50 - 90.17/2 - 2.5        # board corner in plate coords
rod = [(bx0 + 5.08 + i*85.73, by0 + 5.08 + j*80.01) for j in (0, 1) for i in (0, 1)]
V = PX*PY*T - 4*np.pi*1.2**2*T - 4*np.pi*1.7**2*T
print("rod holes:", [(round(x, 3), round(y, 3)) for x, y in rod])
print(f"V = {V:.1f} mm3, mass 6061 = {V*2.7e-3:.2f} g; foot-rod spacing "
      f"{np.hypot(rod[0][0]-3, rod[0][1]-3):.2f} mm")

fig, ax = plt.subplots(figsize=(9, 8.6))
ax.add_patch(Rectangle((-1.5, -2.5), 100, 100, fc="none", ec="0.6", ls=":", lw=1))
ax.text(-1, 98.8, "CubeSat outline 100 x 100", color="0.5", fontsize=8)
ax.add_patch(Rectangle((0, 0), PX, PY, fc="#c6dbef", ec="k", lw=1.5, label="end plate 97 x 95 x 2 (6061)"))
for fx, fy in [(-1.5, -2.5), (91.5-1.5, -2.5), (-1.5, 91.5-2.5), (91.5-1.5, 91.5-2.5)]:
    ax.add_patch(Rectangle((fx, fy), 8.5, 8.5, fc="#fdd0a2", ec="C1", alpha=.6))
ax.plot([], [], "s", color="#fdd0a2", mec="C1", ms=10, label="rail foot underneath (8.5 x 8.5)")
ax.add_patch(Rectangle((bx0, by0), 95.89, 90.17, fc="none", ec="C2", ls="--", lw=1.2,
                       label="PCB outline 95.89 x 90.17 (for reference)"))
for x, y in foot:
    ax.add_patch(Circle((x, y), 1.2, fc="w", ec="C3", lw=1.5))
for x, y in rod:
    ax.add_patch(Circle((x, y), 1.7, fc="w", ec="C0", lw=1.5))
ax.plot([], [], "o", mfc="w", mec="C3", ms=8, label="4x M2 clearance 2.4 -> screw into foot")
ax.plot([], [], "o", mfc="w", mec="C0", ms=9, label="4x M3 clearance 3.4 -> PCB stack rods")
ax.annotate("(3, 3)", (3, 3), (8, 14), fontsize=9, arrowprops=dict(arrowstyle="->", color="C3"))
ax.annotate(f"({rod[0][0]:.3f}, {rod[0][1]:.3f})", rod[0], (14, 22), fontsize=9,
            arrowprops=dict(arrowstyle="->", color="C0"))
ax.annotate("", (rod[0][0], 30), (rod[1][0], 30), arrowprops=dict(arrowstyle="<->", color="C0"))
ax.text(48.5, 31, "85.73", color="C0", ha="center")
ax.annotate("", (30, rod[0][1]), (30, rod[2][1]), arrowprops=dict(arrowstyle="<->", color="C0"))
ax.text(31, 18, "80.01", color="C0")
ax.annotate("", (0, -6), (97, -6), arrowprops=dict(arrowstyle="<->")); ax.text(48.5, -9, "97 (between the side-frame plates)", ha="center")
ax.annotate("", (103, 0), (103, 95), arrowprops=dict(arrowstyle="<->")); ax.text(104.5, 47.5, "95 (between the\nshear panels)", va="center")
ax.plot(0, 0, "ko"); ax.text(1, -4, "origin", fontsize=8)
ax.set(xlim=(-8, 125), ylim=(-12, 104), aspect="equal", xlabel="x (mm)", ylabel="y (mm)",
       title=f"CAD part 3: end plate, x2 identical   (expected mass {V*2.7e-3:.2f} g)")
ax.legend(loc="center", fontsize=8, framealpha=.95); ax.grid(alpha=.25)
fig.tight_layout(); fig.savefig("cad/part3_end_plate_guide.png", dpi=140)
