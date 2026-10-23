"""Step 2 check: ros2_control trajectory controller in Gazebo vs Project 1 step 3.

Same robot, same move (0,0) -> (90,-90) deg in 10 s, same PD gains.
  a) Reference : the controller's interpolated reference vs P1's quintic.
  b) Closed loop: Gazebo (1 kHz digital PD, 1 ms physics) vs P1 (continuous PD,
                  DOP853) -- both run independently from rest at q = 0.
  c) Replay    : Gazebo's logged torques fed to P1's model -> separates "is the
                  physics the same" from "is the controller the same".
  d) System CoM must stay put.

    .venv\\Scripts\\python project5_ros2_gazebo\\step2_compare.py
"""
import os
import sys

import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "project1_free_floating"))
import kinematics as k      # noqa: E402  (Project 1)
import simulation as sim    # noqa: E402  (Project 1 step 3: quintic + PD)
from step1_compare import fix_stamps, replay  # noqa: E402

RES = os.path.join(HERE, "results")
FIG = os.path.join(HERE, "figures")
TAG = sys.argv[1] if len(sys.argv) > 1 else ""


def load():
    with np.load(os.path.join(RES, f"step2_gazebo{TAG}.npz")) as z:
        g = {key: z[key] for key in z.files}
    # Stamps: in ~0.2 % of cycles gz_ros2_control runs the controller with time t+1 ms while
    # the physics state is still at t (2 ms gap + duplicate stamp). The REFERENCE belongs to
    # the original stamp, the measured STATE to the corrected one -> keep both time bases.
    t, keep = np.unique(fix_stamps(g["t"]), return_index=True)
    g["t_raw"] = g["t"][keep]
    for key in ("q_ref", "qd_ref", "q", "qd", "tau"):
        g[key] = g[key][keep]
    keep = t >= 0
    t = t[keep]
    for key in ("q_ref", "qd_ref", "q", "qd", "tau", "t_raw"):
        g[key] = g[key][keep]
    xy = [np.interp(t, g["odom_t"], g["base_xyz"][:, i]) for i in range(2)]
    th0 = np.interp(t, g["odom_t"], np.unwrap(g["base_rpy"][:, 2]))
    return g, t, np.column_stack([*xy, th0, g["q"]])


def main():
    g, t, q_gz = load()
    D = np.degrees

    # ---- a) reference
    ref_p1 = np.array([sim.desired(ti)[0] for ti in g["t_raw"]])
    n_shift = int(np.sum(np.abs(g["t_raw"] - t) > 1e-9))
    print(f"   ({n_shift} of {len(t)} cycles ran the controller 1 ms ahead of the physics)")
    print("a) controller reference vs P1 quintic (original stamps): max diff "
          f"{D(np.abs(g['q_ref'] - ref_p1).max()):.2e} deg")

    # ---- b) independent closed-loop runs
    t_p1, q_p1, qd_p1, tau_p1 = sim.simulate(n_points=15001)          # 1 ms grid, 0..15 s
    m = t <= t_p1[-1]
    tg, qg = t[m], q_gz[m]
    q_p1i = np.column_stack([np.interp(tg, t_p1, q_p1[:, i]) for i in range(5)])
    tau_p1i = np.column_stack([np.interp(tg, t_p1, tau_p1[:, i]) for i in range(2)])
    err = qg - q_p1i
    print("\nb) closed loop, Gazebo vs P1 (independent runs)")
    print(f"   final base angle   : Gazebo {D(qg[-1, 2]):8.4f} deg | P1 {D(q_p1[-1, 2]):8.4f} deg")
    print(f"   final base position: Gazebo ({qg[-1, 0]*1e3:7.2f}, {qg[-1, 1]*1e3:7.2f}) mm"
          f" | P1 ({q_p1[-1, 0]*1e3:7.2f}, {q_p1[-1, 1]*1e3:7.2f}) mm")
    print(f"   max |error| base angle {D(np.abs(err[:, 2]).max()):.4f} deg, base x,y "
          f"{np.abs(err[:, 0]).max()*1e3:.3f}, {np.abs(err[:, 1]).max()*1e3:.3f} mm")
    print(f"   max |error| joints {D(np.abs(err[:, 3]).max()):.4f}, {D(np.abs(err[:, 4]).max()):.4f} deg")
    print(f"   max |error| torques {np.abs(g['tau'][m] - tau_p1i).max(axis=0).round(4)} N m"
          f" (peak torques {np.abs(tau_p1).max(axis=0).round(2)} N m)")
    trk_gz = D(np.abs(g["q"] - g["q_ref"]).max(axis=0))
    trk_p1 = D(np.abs(q_p1[:, 3:] - np.array([sim.desired(ti)[0] for ti in t_p1])).max(axis=0))
    print(f"   max tracking error: Gazebo {trk_gz.round(3)} deg | P1 {trk_p1.round(3)} deg")

    # ---- c) replay Gazebo's torques in P1's model
    q_rep = replay(t, g["tau"], q_gz[0], np.concatenate([[0, 0, 0], g["qd"][0]]), t)
    e_rep = q_gz - q_rep
    print("\nc) replay of Gazebo's torques in P1's model (physics only)")
    print(f"   max |error| base angle {D(np.abs(e_rep[:, 2]).max()):.4f} deg, joints "
          f"{D(np.abs(e_rep[:, 3]).max()):.4f}, {D(np.abs(e_rep[:, 4]).max()):.4f} deg")

    # ---- d) system CoM
    com = np.array([k.system_com(qi) for qi in q_gz])
    print(f"\nd) system CoM drift in Gazebo: max {np.linalg.norm(com - com[0], axis=1).max()*1e3:.4f} mm")

    # ---- figure
    fig, ax = plt.subplots(2, 2, figsize=(12, 7.5))
    for j, c in ((0, "C0"), (1, "C1")):
        ax[0, 0].plot(t, D(g["q"][:, j]), c, lw=3, alpha=0.5, label=f"q{j+1} Gazebo")
        ax[0, 0].plot(t_p1, D(q_p1[:, 3 + j]), "k--", lw=1, label="P1" if j == 0 else None)
        ax[0, 0].plot(t, D(g["q_ref"][:, j]), ":", color=c, lw=1, label=f"q{j+1} reference")
    ax[0, 0].set(title="Joint angles (joint_trajectory_controller, PD)", xlabel="t [s]", ylabel="deg")
    ax[0, 0].legend(fontsize=8)
    for j, c in ((0, "C0"), (1, "C1")):
        ax[0, 1].plot(t, g["tau"][:, j], c, lw=3, alpha=0.5, label=rf"$\tau_{j+1}$ Gazebo")
        ax[0, 1].plot(t_p1, tau_p1[:, j], "k--", lw=1, label="P1" if j == 0 else None)
    ax[0, 1].set(title="Joint torques", xlabel="t [s]", ylabel="N m")
    ax[0, 1].legend(fontsize=8)
    ax[1, 0].plot(t, D(q_gz[:, 2]), "C2", lw=3, alpha=0.5, label="Gazebo")
    ax[1, 0].plot(t_p1, D(q_p1[:, 2]), "k--", lw=1, label="P1")
    ax[1, 0].set(title=r"Base attitude $\theta_0$ (uncontrolled)", xlabel="t [s]", ylabel="deg")
    ax[1, 0].legend()
    ax[1, 1].plot(tg, D(err[:, 2]) * 1e3, "C2", label=r"$\theta_0$ [mdeg]")
    ax[1, 1].plot(tg, D(err[:, 3]) * 1e3, "C0", label=r"$q_1$ [mdeg]")
    ax[1, 1].plot(tg, D(err[:, 4]) * 1e3, "C1", label=r"$q_2$ [mdeg]")
    ax[1, 1].plot(tg, err[:, 0] * 1e3, "C3--", label=r"$x_0$ [mm]")
    ax[1, 1].plot(tg, err[:, 1] * 1e3, "C4--", label=r"$y_0$ [mm]")
    ax[1, 1].set(title="Difference Gazebo - P1 (independent closed-loop runs)", xlabel="t [s]")
    ax[1, 1].legend(fontsize=8, ncol=2)
    fig.suptitle("Step 2: ros2_control PD trajectory tracking in zero-g Gazebo vs Project 1")
    fig.tight_layout()
    out = os.path.join(FIG, f"step2_gazebo_vs_python{TAG}.png")
    fig.savefig(out, dpi=130)
    print(f"\nfigure: {out}")


if __name__ == "__main__":
    main()
