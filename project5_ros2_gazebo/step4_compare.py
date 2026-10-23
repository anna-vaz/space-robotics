"""Step 4 check: reach + capture of Project 3's tumbling target in Gazebo.

  1. base attitude in each phase (A: approach, B: spin-synchronised final approach),
     'rns' vs 'minnorm', and the gripper accuracy at capture
  2. momentum of robot + target through the capture (robot momentum from the NumPy model
     with Gazebo's joint rates and finite-differenced base pose; target from its odometry)
  3. after capture the joints are locked -> one rigid body: its spin must equal
     L_cm / I_sys (angular momentum about the system CoM / inertia of the composite),
     the same check as Project 3 step 3

    .venv\\Scripts\\python project5_ros2_gazebo\\step4_compare.py
"""
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "zero_g_manipulator"))
from zero_g_manipulator import reach as R  # noqa: E402
from zero_g_manipulator.free_floating_model import P1B_ARM, PlanarRobot  # noqa: E402

RES, FIG = os.path.join(HERE, "results"), os.path.join(HERE, "figures")
rob = PlanarRobot(**P1B_ARM)
M_T, I_T, A_T = 50.0, 50.0 * 2 / 12, 1.0
D = np.degrees


def load(mode):
    with np.load(os.path.join(RES, f"step4_{mode}.npz")) as z:
        g = {k: z[k] for k in z.files}
    out = {"t_capture": float(g["t_capture"])}
    for key, tk, cols in (("j", "joint_t", ("q", "qd", "grip_err", "phi_err")), ("b", "base_t", ("base",)),
                          ("g", "target_t", ("target",))):
        t, k = np.unique(g[tk], return_index=True)
        out[key + "_t"] = t
        for c in cols:
            out[c] = g[c][k]
    out["base"][:, 2] = np.unwrap(out["base"][:, 2])
    out["target"][:, 2] = np.unwrap(out["target"][:, 2])
    return out


def interp(t, tt, y):
    return np.array([np.interp(t, tt, y[:, i]) for i in range(y.shape[1])])


def state(d, t, h=0.002):
    """Full robot q, qd and target (c, th, v, w) at time t (finite differences of poses)."""
    base = interp(t, d["b_t"], d["base"])
    bdot = (interp(t + h, d["b_t"], d["base"]) - interp(t - h, d["b_t"], d["base"])) / (2 * h)
    q = np.r_[base, interp(t, d["j_t"], d["q"])]
    qd = np.r_[bdot, interp(t, d["j_t"], d["qd"])]
    tg = interp(t, d["g_t"], d["target"][:, :3])
    tgd = (interp(t + h, d["g_t"], d["target"][:, :3]) - interp(t - h, d["g_t"], d["target"][:, :3])) / (2 * h)
    return q, qd, tg[:2], tg[2], tgd[:2], tgd[2]


def momentum(d, t):
    q, qd, c, th, v, w = state(d, t)
    P, L = rob.momentum(q, qd)
    return P + M_T * v, L + M_T * (c[0] * v[1] - c[1] * v[0]) + I_T * w


def composite_prediction(d, t):
    """Spin of the locked robot + target as one rigid body: L_cm / I_sys."""
    q, qd, c, th, v, w = state(d, t)
    pts = rob.mass_points(q) + [(M_T, I_T, c)]
    mtot = sum(m for m, _, _ in pts)
    rcm = sum(m * r for m, _, r in pts) / mtot
    I_sys = sum(I + m * np.sum((r - rcm) ** 2) for m, I, r in pts)
    P, L = momentum(d, t)
    L_cm = L - (rcm[0] * P[1] - rcm[1] * P[0])
    return L_cm / I_sys, I_sys, L_cm


def draw(ax, d, t, alpha):
    q, _, c, th, _, _ = state(d, t)
    r0, joints, _, _, ee = rob._chain(q)
    cs, sn = np.cos(q[2]), np.sin(q[2])
    sq = np.array([[0.5, 0.5], [-0.5, 0.5], [-0.5, -0.5], [0.5, -0.5]])
    Rb = np.array([[cs, -sn], [sn, cs]])
    ax.add_patch(Polygon(r0 + sq @ Rb.T, closed=True, fc="goldenrod", ec="k", alpha=alpha))
    pts = np.array(joints + [ee])
    ax.plot(pts[:, 0], pts[:, 1], "-o", color="C0", lw=3, ms=4, alpha=alpha)
    Rt = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    ax.add_patch(Polygon(c + (A_T * sq) @ Rt.T, closed=True, fc="0.7", ec="k", alpha=alpha))
    h = c + R.HANDLE_R * np.array([np.cos(th), np.sin(th)])
    ax.plot(*h, "s", color="C3", ms=6, alpha=alpha)


def main():
    data = {m: load(m) for m in ("rns", "minnorm")}
    print("1) base attitude and capture accuracy")
    for m, d in data.items():
        th = d["base"][:, 2]
        bt = d["b_t"]
        thA = th[bt <= R.T1]
        iT1, iT2 = np.searchsorted(bt, R.T1), np.searchsorted(bt, d["t_capture"])
        i = np.searchsorted(d["j_t"], d["t_capture"])
        q, qd, c, tht, v, w = state(d, d["t_capture"] - 0.005)
        rel_w = qd[2] + qd[3:].sum() - w
        print(f"   {m:8s} approach (0-{R.T1:.0f} s): max|th0| {D(np.abs(thA).max()):7.3f} deg |"
              f" sync phase: th0 {D(th[iT1]):+7.3f} -> {D(th[iT2]):+7.3f} deg |"
              f" capture at {d['t_capture']:.3f} s: gripper-handle {d['grip_err'][i]*1e3:.2f} mm,"
              f" angle {D(d['phi_err'][i]):+.3f} deg, relative spin {D(rel_w):+.3f} deg/s")

    print("\n2) momentum of robot + target  [L about the origin in N m s, P in N s]")
    for m, d in data.items():
        tc = d["t_capture"]
        rows = [(t,) + tuple(np.r_[momentum(d, t)]) for t in (1.0, 10.0, tc - 0.05, tc + 0.05, tc + 5.0, tc + 11.0)]
        Ls = np.array([r[3] for r in rows])
        print(f"   {m:8s} L: " + ", ".join(f"t={r[0]:5.2f}: {r[3]:+.5f}" for r in rows)
              + f"  | |P| max {max(np.hypot(r[1], r[2]) for r in rows):.1e}")
        print(f"            spin-up L = I_T w_T = {I_T*np.radians(3.0):.5f};  change across capture"
              f" {Ls[3]-Ls[2]:+.1e} ({abs(Ls[3]-Ls[2])/abs(Ls[2]):.1e} relative)")

    print("\n3) after capture: composite spin vs L_cm / I_sys")
    for m, d in data.items():
        tc = d["t_capture"]
        w_pred, I_sys, L_cm = composite_prediction(d, tc + 2.0)
        bt = d["b_t"]
        sel = (bt > tc + 2.0) & (bt < tc + 11.0)
        w_meas = np.polyfit(bt[sel], d["base"][sel, 2], 1)[0]
        gsel = (d["g_t"] > tc + 2.0) & (d["g_t"] < tc + 11.0)
        w_tgt = np.polyfit(d["g_t"][gsel], d["target"][gsel, 2], 1)[0]
        print(f"   {m:8s} I_sys {I_sys:7.2f} kg m^2, L_cm {L_cm:+.5f} -> predicted {D(w_pred):+.5f} deg/s |"
              f" Gazebo base {D(w_meas):+.5f}, target {D(w_tgt):+.5f} deg/s  (target before: 3.00000)")

    # ---- figure
    fig = plt.figure(figsize=(15, 5.2))
    ax0 = fig.add_subplot(1, 3, 1)
    d = data["rns"]
    for t, a in ((0.0, 0.25), (R.T1, 0.5), (d["t_capture"], 1.0)):
        draw(ax0, d, t, a)
    ax0.set(title="RNS capture: t = 0, 20 s (stand-off), 28 s (capture)", xlabel="x [m]", ylabel="y [m]")
    ax0.axis("equal")
    ax0.set_xlim(-0.9, 2.6)
    ax1 = fig.add_subplot(1, 3, 2)
    for m, c in (("minnorm", "C3"), ("rns", "C0")):
        d = data[m]
        ax1.plot(d["b_t"], D(d["base"][:, 2]), c, lw=2,
                 label="plain J* approach" if m == "minnorm" else "RNS approach")
    ax1.axvspan(R.T1, R.T2, color="0.9", label="spin synchronisation")
    ax1.axvline(R.T2, color="k", lw=0.8, ls=":")
    ax1.text(R.T2 + 0.3, ax1.get_ylim()[0] * 0.9 + 0.1, "capture", fontsize=8)
    ax1.set(title=r"Base attitude $\theta_0$", xlabel="t [s]", ylabel="deg")
    ax1.legend(fontsize=8)
    ax2 = fig.add_subplot(1, 3, 3)
    for m, c in (("minnorm", "C3"), ("rns", "C0")):
        d = data[m]
        ts = np.arange(0.5, d["j_t"][-1] - 0.1, 0.1)
        L = np.array([momentum(d, t)[1] for t in ts])
        ax2.plot(ts, L, c, lw=2, label=m)
    ax2.axhline(I_T * np.radians(3.0), color="k", ls="--", lw=1, label=r"$I_T\,\omega_T$ (target spin-up)")
    ax2.set(title="Total angular momentum (robot + target)", xlabel="t [s]", ylabel="N m s")
    ax2.set_ylim(0, 0.6)
    ax2.legend(fontsize=8)
    fig.suptitle("Step 4: reaching and capturing a tumbling target (Project 3) with the reactionless 3-link arm"
                 " (Project 1b) in Gazebo")
    fig.tight_layout()
    out = os.path.join(FIG, "step4_capture.png")
    fig.savefig(out, dpi=130)
    print(f"\nfigure: {out}")


if __name__ == "__main__":
    main()
