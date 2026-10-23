"""Step 1 check: is the Gazebo robot the SAME robot as Project 1's Python model?

1. URDF check  : masses, inertias and joint positions in the URDF that Gazebo
                 loaded vs project1_free_floating/params.py.
2. Zero-g check: with no torque the robot must not move at all.
3. Replay      : the torques Gazebo actually received (logged, 1 kHz, zero-order
                 hold) are fed to Project 1's Lagrange model (DOP853, tight tol.);
                 base angle/position and joint angles are compared.
4. Momentum    : the system centre of mass must not move (no external force),
                 computed from Gazebo's data with Project 1's kinematics.

Run (Windows, from the repo root):
    .venv\\Scripts\\python project5_ros2_gazebo\\step1_compare.py
"""
import os
import sys
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "project1_free_floating"))
import dynamics as d        # noqa: E402  (Project 1)
import kinematics as k      # noqa: E402
import params as p          # noqa: E402

RES = os.path.join(HERE, "results")
FIG = os.path.join(HERE, "figures")
TAG = sys.argv[1] if len(sys.argv) > 1 else ""   # e.g. "_dt05" for the 0.5 ms physics-step run
os.makedirs(FIG, exist_ok=True)


# ---------------------------------------------------------------- 1. URDF check
def urdf_check():
    root = ET.parse(os.path.join(RES, "robot.urdf")).getroot()
    links = {l.get("name"): l for l in root.findall("link")}
    joints = {j.get("name"): j for j in root.findall("joint")}  # top level only, not <ros2_control>'s

    def mass(n):
        return float(links[n].find("inertial/mass").get("value"))

    def izz(n):
        return float(links[n].find("inertial/inertia").get("izz"))

    def com_x(n):
        return float(links[n].find("inertial/origin").get("xyz").split()[0])

    def jxyz(n):
        return [float(v) for v in joints[n].find("origin").get("xyz").split()]

    rows = [
        ("base mass m0", mass("base_link"), p.m0),
        ("base Izz  I0", izz("base_link"), p.I0),
        ("link1 mass", mass("link1"), p.m1), ("link1 Izz", izz("link1"), p.I1),
        ("link2 mass", mass("link2"), p.m2), ("link2 Izz", izz("link2"), p.I2),
        ("link1 CoM x", com_x("link1"), p.l1 / 2), ("link2 CoM x", com_x("link2"), p.l2 / 2),
        ("shoulder x b0", jxyz("joint1")[0], p.b0[0]), ("elbow x l1", jxyz("joint2")[0], p.l1),
    ]
    print("1) URDF vs Project 1 params")
    worst = 0.0
    for name, u, ref in rows:
        worst = max(worst, abs(u - ref))
        print(f"   {name:14s} URDF {u:10.5f}   P1 {ref:10.5f}")
    print(f"   max difference: {worst:.1e}")
    return worst


# ---------------------------------------------------------------- load Gazebo log
def load():
    with np.load(os.path.join(RES, f"step1_gazebo{TAG}.npz")) as z:
        g = {key: z[key] for key in z.files}
    t, keep = np.unique(g["joint_t"], return_index=True)   # drop the few repeated stamps
    for key in ("q", "qd", "effort", "cmd"):
        g[key] = g[key][keep]
    # base pose sampled at the joint-state times
    xyz = np.column_stack([np.interp(t, g["odom_t"], g["base_xyz"][:, i]) for i in range(3)])
    yaw = np.interp(t, g["odom_t"], np.unwrap(g["base_rpy"][:, 2]))
    q = np.column_stack([xyz[:, 0], xyz[:, 1], yaw, g["q"]])
    return g, t, q


# ---------------------------------------------------------------- 3. replay in P1
def replay(t_log, cmd, q0, qd0, t_eval):
    """Integrate P1's model with the logged torque held constant between samples."""
    def tau_at(t):
        i = np.searchsorted(t_log, t, side="right") - 1
        return cmd[max(i, 0)]

    def rhs(t, x):
        return np.concatenate([x[5:], d.accelerations(x[:5], x[5:], tau_at(t))])

    sol = solve_ivp(rhs, (t_eval[0], t_eval[-1]), np.concatenate([q0, qd0]), t_eval=t_eval,
                    method="DOP853", rtol=1e-10, atol=1e-12, max_step=1e-3)
    return sol.y[:5].T


def main():
    urdf_err = urdf_check()
    g, t, q_gz = load()
    t_rest, t_pulse = float(g["t_rest"]), float(g["t_pulse"])

    # ---- 2. zero-g rest check
    rest = t < t_rest
    drift = np.abs(q_gz[rest] - q_gz[0]).max(axis=0)
    print("\n2) Rest phase (no torque, first %.1f s): max motion" % t_rest)
    print(f"   base x,y {drift[0]:.1e}, {drift[1]:.1e} m | base angle {np.degrees(drift[2]):.1e} deg"
          f" | joints {np.degrees(drift[3]):.1e}, {np.degrees(drift[4]):.1e} deg")
    plan = max(np.abs(g["base_xyz"][:, 2]).max(), *np.abs(g["base_rpy"][:, :2]).max(axis=0))
    print(f"   out-of-plane motion (|z|, |roll|, |pitch|) over the whole run: {plan:.1e}")

    # ---- 3. replay: start the P1 model from Gazebo's state at the start of the pulse
    i0 = np.searchsorted(t, t_rest) - 1
    te = t[i0:]
    qd0 = np.concatenate([[0.0, 0.0, 0.0], g["qd"][i0]])  # robot is at rest here
    q_py = replay(t, g["cmd"], q_gz[i0], qd0, te)
    err = q_py - q_gz[i0:]
    print("\n3) Gazebo vs Project 1 model (same logged torques)")
    print(f"   final base angle  : Gazebo {np.degrees(q_gz[-1, 2]):8.4f} deg | P1 {np.degrees(q_py[-1, 2]):8.4f} deg")
    print(f"   final joint 1     : Gazebo {np.degrees(q_gz[-1, 3]):8.4f} deg | P1 {np.degrees(q_py[-1, 3]):8.4f} deg")
    print(f"   final joint 2     : Gazebo {np.degrees(q_gz[-1, 4]):8.4f} deg | P1 {np.degrees(q_py[-1, 4]):8.4f} deg")
    print(f"   max |error| base x,y : {np.abs(err[:, 0]).max()*1e3:.3f}, {np.abs(err[:, 1]).max()*1e3:.3f} mm")
    print(f"   max |error| base angle: {np.degrees(np.abs(err[:, 2]).max()):.4f} deg")
    print(f"   max |error| joints    : {np.degrees(np.abs(err[:, 3]).max()):.4f}, "
          f"{np.degrees(np.abs(err[:, 4]).max()):.4f} deg")

    # ---- 4. system centre of mass from Gazebo data
    com = np.array([k.system_com(qi) for qi in q_gz])
    com_drift = np.linalg.norm(com - com[0], axis=1)
    print(f"\n4) System CoM drift in Gazebo (should be 0): max {com_drift.max()*1e3:.4f} mm "
          f"(base itself moved {np.linalg.norm(q_gz[:, :2] - q_gz[0, :2], axis=1).max()*1e3:.1f} mm)")
    print(f"   torque check: logged joint effort vs command, max diff "
          f"{np.nanmax(np.abs(g['effort'] - g['cmd'])):.3f} N m")

    # ---- figure
    fig, ax = plt.subplots(2, 2, figsize=(12, 7.5))
    ax[0, 0].plot(t, g["cmd"][:, 0], label=r"$\tau_1$")
    ax[0, 0].plot(t, g["cmd"][:, 1], label=r"$\tau_2$")
    ax[0, 0].axvspan(0, t_rest, color="0.9", label="rest check")
    ax[0, 0].set(title="Joint torques sent to Gazebo", xlabel="t [s]", ylabel="N m")
    ax[0, 0].legend()
    for j, c in ((3, "C0"), (4, "C1")):
        ax[0, 1].plot(t, np.degrees(q_gz[:, j]), c, lw=2.5, alpha=0.5, label=f"q{j-2} Gazebo")
        ax[0, 1].plot(te, np.degrees(q_py[:, j]), "k--", lw=1, label=f"q{j-2} P1 model" if j == 3 else None)
    ax[0, 1].set(title="Joint angles", xlabel="t [s]", ylabel="deg")
    ax[0, 1].legend()
    ax[1, 0].plot(t, np.degrees(q_gz[:, 2]), "C2", lw=2.5, alpha=0.5, label="Gazebo")
    ax[1, 0].plot(te, np.degrees(q_py[:, 2]), "k--", lw=1, label="P1 model")
    ax[1, 0].set(title=r"Base attitude $\theta_0$ (uncontrolled, reacts to the arm)",
                 xlabel="t [s]", ylabel="deg")
    ax[1, 0].legend()
    ax[1, 1].plot(q_gz[:, 0] * 1e3, q_gz[:, 1] * 1e3, "C3", lw=2.5, alpha=0.5, label="base CoM, Gazebo")
    ax[1, 1].plot(q_py[:, 0] * 1e3, q_py[:, 1] * 1e3, "k--", lw=1, label="base CoM, P1 model")
    ax[1, 1].plot((com[:, 0] - com[0, 0]) * 1e3 + com[0, 0] * 1e3, (com[:, 1]) * 1e3, "C4",
                  lw=3, label="system CoM, Gazebo (stays put)")
    ax[1, 1].set(title="Base translation in the plane", xlabel="x [mm]", ylabel="y [mm]")
    ax[1, 1].axis("equal")
    ax[1, 1].legend(fontsize=8)
    fig.suptitle("Step 1: free-floating robot in zero-g Gazebo vs Project 1 Python model")
    fig.tight_layout()
    out = os.path.join(FIG, f"step1_gazebo_vs_python{TAG}.png")
    fig.savefig(out, dpi=130)
    print(f"\nfigure: {out}  (URDF max diff {urdf_err:.1e})")


if __name__ == "__main__":
    main()
