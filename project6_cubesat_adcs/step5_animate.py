"""Project 6 step 5b: animation of the Athens pass from step 4 (MEKF in the loop).

Left: the CubeSat in the orbit frame (x along-track, y orbit normal, z up), camera face
(+z body) red, boresight red, nadir and line of sight to Athens.
Right: ground view in a local east/north plane around Athens: sub-satellite track and the
point where the boresight hits the ground.
Run after step4_pointing.py:  ../.venv/Scripts/python.exe step5_animate.py
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import attitude as at
import controller as ct
import environment as en
import orbit as ob
import params as pr
import step4_pointing as s4

d = np.load("results_step4.npz")
t = d["pass_mekf_t"]; beta = d["pass_mekf_beta"]; el = d["pass_mekf_el"]; mode = d["pass_mekf_mode"]
zerr = np.degrees(d["pass_mekf_zerr"])
env = en.OrbitEnvironment(s4.ENV_T)

# local east-north-up frame at Athens (ECEF)
up = s4.P_ATHENS / np.linalg.norm(s4.P_ATHENS)
east = np.cross([0, 0, 1.0], up); east /= np.linalg.norm(east)
north = np.cross(up, east)


def to_local(p_ecef):
    """Ground point -> (east, north) km on the plane tangent at Athens (gnomonic-ish)."""
    return np.array([p_ecef @ east, p_ecef @ north]) / 1e3


sub, hit, C_LB, los_L = [], [], [], []
for tk, bk in zip(t, beta):
    e = env.at(tk)
    th = en.gmst(env.jd0 + tk / 86400)
    E = en.R3(th)                                     # ECI -> ECEF
    r = e["r"]
    BN = at.ep2dcm(bk)
    z = BN.T @ np.array([0, 0, 1.0])
    # boresight ray r + s z hits |x| = RE
    b = r @ z; c = r @ r - ob.RE**2
    disc = b * b - c
    hit.append(to_local(E @ (r + (-b - np.sqrt(disc)) * z)) if disc > 0 and -b > 0 else [np.nan, np.nan])
    sub.append(to_local(E @ (ob.RE * r / np.linalg.norm(r))))
    RN = ct.nadir_or_target_frame(r, e["v"])
    C_LB.append(RN @ BN.T)
    p = ct.target_eci(s4.P_ATHENS, env.jd0 + tk / 86400)
    los_L.append(RN @ ((p - r) / np.linalg.norm(p - r)))
sub, hit, C_LB, los_L = map(np.array, (sub, hit, C_LB, los_L))
D = np.diag([1.0, -1.0, -1.0])                        # LVLH (along, -h, nadir) -> display (along, h, up)

half = pr.DIMS / 2 / pr.DIMS.max()
corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
faces = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]

fig = plt.figure(figsize=(11, 5.2))
ax3 = fig.add_subplot(1, 2, 1, projection="3d")
ax2 = fig.add_subplot(1, 2, 2)
fig.subplots_adjust(left=0.02, right=0.97, bottom=0.1, top=0.9, wspace=0.15)
bb = ax2.get_position()
axi = fig.add_axes([bb.x1 - 0.36 * bb.width, bb.y1 - 0.38 * bb.height, 0.34 * bb.width, 0.34 * bb.height])
frames = np.arange(0, len(t), 12)
t_aos = s4.T_AOS


def draw(k):
    ax3.cla()
    C = D @ C_LB[k]
    V = corners @ C.T
    polys = [V[f] for f in faces]
    zface = [i for i, f in enumerate(faces) if all(corners[j][2] > 0 for j in f)][0]
    cols = ["0.8"] * 6; cols[zface] = "C3"
    ax3.add_collection3d(Poly3DCollection(polys, facecolors=cols, edgecolors="k", linewidths=0.6, alpha=0.9))
    zb = C @ np.array([0, 0, 1.0])
    ax3.quiver(0, 0, 0, *(1.3 * zb), color="C3", lw=2, arrow_length_ratio=0.1)
    ax3.quiver(0, 0, 0, 0, 0, -1.3, color="k", lw=1, arrow_length_ratio=0.1)
    ax3.text(0, 0, -1.45, "nadir", fontsize=8, ha="center")
    if mode[k]:
        lv = D @ los_L[k]
        ax3.quiver(0, 0, 0, *(1.3 * lv), color="C2", lw=1.2, ls="--", arrow_length_ratio=0.1)
        ax3.text(*(1.45 * lv), "Athens", color="C2", fontsize=8)
    ax3.set(xlim=(-1, 1), ylim=(-1, 1), zlim=(-1.4, 1), xlabel="along-track", ylabel="orbit normal")
    ax3.set_xticks([]); ax3.set_yticks([]); ax3.set_zticks([])
    ax3.view_init(elev=18, azim=-60)
    ax3.set_title(f"t = {(t[k]-t_aos)/60:+5.1f} min from AOS   mode: {'TARGET' if mode[k] else 'nadir'}\n"
                  f"Athens elevation {el[k]:5.1f} deg, boresight error {zerr[k]:6.2f} deg", fontsize=9)

    ax2.cla()
    ax2.plot(sub[:, 0], sub[:, 1], color="0.7", lw=1, label="sub-satellite track")
    ax2.plot(sub[k, 0], sub[k, 1], "ko", ms=5)
    j0 = max(0, k - 90)
    ax2.plot(hit[j0:k + 1, 0], hit[j0:k + 1, 1], "C3-", lw=1.2, label="boresight on the ground (last 90 s)")
    ax2.plot(hit[k, 0], hit[k, 1], "C3o", ms=6)
    ax2.plot(0, 0, "C2*", ms=14, label="Athens")
    for rr in (250, 500, 1000):
        ax2.add_patch(plt.Circle((0, 0), rr, fill=False, color="C2", lw=0.5, ls=":"))
    miss = np.hypot(*hit[k]) if np.all(np.isfinite(hit[k])) else np.nan
    axi.cla()
    axi.plot(hit[j0:k + 1, 0], hit[j0:k + 1, 1], "C3-", lw=1)
    axi.plot(hit[k, 0], hit[k, 1], "C3o", ms=4)
    axi.plot(0, 0, "C2*", ms=10)
    axi.set(xlim=(-10, 10), ylim=(-10, 10), aspect="equal")
    axi.tick_params(labelsize=6); axi.text(0.5, 0.88, "zoom +-10 km", transform=axi.transAxes, ha="center", fontsize=7); axi.grid(alpha=0.3)
    ax2.set(xlim=(-1800, 1800), ylim=(-1800, 1800), xlabel="east of Athens [km]", ylabel="north of Athens [km]",
            title=f"Ground view (circles 250/500/1000 km); boresight miss {miss:.1f} km", aspect="equal")
    ax2.legend(fontsize=7, loc="lower left"); ax2.grid(alpha=0.3)


anim = FuncAnimation(fig, draw, frames=frames, interval=80)
anim.save("figures/athens_pass.gif", writer=PillowWriter(fps=12), dpi=75)
draw(int(np.argmax(el))); fig.savefig("figures/step5_pass_frame.png", dpi=110)
print("written figures/athens_pass.gif and figures/step5_pass_frame.png")
