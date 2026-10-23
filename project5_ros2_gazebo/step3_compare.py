"""Step 3 check: Reaction Null Space controller in Gazebo vs Project 1b.

Robot: P1b 3-link arm (1/1/0.5 m, 10/10/5 kg) on the 100 kg base, start q = (30, 60, -45) deg.
Desired joint rates qd_des = (0.2, -0.3, 0.4) rad/s for 4 s, sent to joint velocity servos:
    commanded : qd = qd_des           -> base rotates
    rns       : qd = P(q) qd_des      -> base should not rotate
The RNS node (ROS 2, 1000/100/20 Hz) computes P(q) from its own NumPy model.
"_ramp" runs scale qd_des with a smooth 0.2 s ramp at start/stop.

Checks
  1. node model (free_floating_model.py) vs P1b SymPy model: M and b
  2. Gazebo vs P1b's continuous zero-momentum integration (P1b step 2)
  3. residual base rotation split into
       ideal controller : the node's discrete logic (hold, 1-step delay) on exact kinematics
       joint-path gap   : Gazebo's joint path vs that ideal path (servo transients)
       physics gap      : Gazebo base angle vs sum b(q_mid) dq along Gazebo's own joint path

    .venv\\Scripts\\python project5_ros2_gazebo\\step3_compare.py
"""
import os
import subprocess
import sys

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "zero_g_manipulator"))
from zero_g_manipulator.free_floating_model import P1B_ARM, PlanarRobot  # noqa: E402

RES, FIG = os.path.join(HERE, "results"), os.path.join(HERE, "figures")
P1B = os.path.join(HERE, "..", "project1b_reaction_null_space")
RNS_CASES = ["rns_1000hz", "rns_100hz", "rns_20hz", "rns_1000hz_ramp", "rns_100hz_ramp", "rns_20hz_ramp"]
Q0 = np.radians([30.0, 60.0, -45.0])
QD_DES = np.array([0.20, -0.30, 0.40])
T, T_RAMP, DT = 4.0, 0.2, 1e-3
rob = PlanarRobot(**P1B_ARM)
D = np.degrees


def ramp(tau, t_move, t_ramp):
    """Same as rns_controller.ramp (copied: that module needs rclpy)."""
    if t_ramp <= 0.0:
        return 1.0
    x = min(tau, t_move - tau) / t_ramp
    return 1.0 if x >= 1.0 else 0.5 - 0.5 * np.cos(np.pi * max(x, 0.0))


def full(qj):
    return np.concatenate([[0.0, 0.0, 0.0], qj])     # b, P do not depend on the base pose


# ---------------------------------------------------------------- 1. model check (P1b in a subprocess)
CHECK = r"""
import sys, numpy as np
sys.path.insert(0, r'%s')
import params as p, rns
from model import build
from zero_g_manipulator.free_floating_model import PlanarRobot, P1B_ARM
rob, mine = build(p), PlanarRobot(**P1B_ARM)
rng = np.random.default_rng(1); eM = eb = 0.0
for _ in range(200):
    q = rng.uniform(-np.pi, np.pi, 6)
    eM = max(eM, np.abs(mine.mass_matrix(q) - rob.mass_matrix(q)).max())
    eb = max(eb, np.abs(mine.reaction_row(q) - rns.reaction_row(rob, q)).max())
print(f"{eM:.1e} {eb:.1e}")
""" % os.path.join(HERE, "zero_g_manipulator")


def model_check():
    out = subprocess.run([sys.executable, "-c", CHECK], cwd=P1B, capture_output=True, text=True, check=True)
    eM, eb = out.stdout.split()
    print(f"1) node model vs P1b SymPy model (200 random q): max |dM| {eM}, max |db| {eb}")


# ---------------------------------------------------------------- references
def p1b_reference(project):
    """P1b step 2: continuous zero-momentum kinematics, th0' = b(q) qd (DOP853)."""
    def f(t, x):
        qd = rob.rns_projector(full(x[1:])) @ QD_DES if project else QD_DES
        return np.concatenate([[rob.reaction_row(full(x[1:])) @ qd], qd])
    t = np.linspace(0, T, 401)
    sol = solve_ivp(f, (0, T), np.concatenate([[0.0], Q0]), t_eval=t, method="DOP853", rtol=1e-11, atol=1e-12)
    return t, sol.y.T


def ideal_controller(rate, t_ramp, delay_steps=1):
    """The node alone: every 1/rate s it sees q (delay_steps old; Gazebo's measured pipeline
    = 1 step) and holds qd = P(q) ramp(t) qd_des; perfect velocity servos; exact kinematics
    (Simpson along each straight 1 ms segment). Returns the base rotation [rad]."""
    hold = int(round(1.0 / rate / DT))
    q, th, hist, v = Q0.copy(), 0.0, [Q0.copy()], np.zeros(3)
    for k in range(int(round(T / DT))):
        if k % hold == 0:
            qs = hist[max(len(hist) - 1 - delay_steps, 0)]
            v = rob.rns_projector(full(qs)) @ (ramp(k * DT, T, t_ramp) * QD_DES)
        bq = [rob.reaction_row(full(q + v * DT * s)) @ v for s in (0.0, 0.5, 1.0)]
        th += DT / 6 * (bq[0] + 4 * bq[1] + bq[2])
        q = q + v * DT
        hist.append(q.copy())
    return th


def load(case):
    with np.load(os.path.join(RES, f"step3_{case}.npz")) as z:
        g = {k: z[k] for k in z.files}
    t, keep = np.unique(g["joint_t"], return_index=True)
    q = g["q"][keep]
    yaw = np.unwrap(g["base_rpy"][:, 2])
    th0 = np.interp(t, g["odom_t"], yaw) - np.interp(0.0, g["odom_t"], yaw)
    m = t >= 0
    return t[m], q[m], th0[m]


def path_integral(q):
    """Base rotation implied by a joint path: sum b(q_mid) . dq (zero momentum)."""
    qm = 0.5 * (q[1:] + q[:-1])
    b = np.array([rob.reaction_row(full(x)) for x in qm])
    return np.concatenate([[0.0], np.cumsum(np.sum(b * np.diff(q, axis=0), axis=1))])


def main():
    model_check()
    refs = {"commanded": p1b_reference(False), "rns": p1b_reference(True)}
    data = {case: load(case) for case in ["commanded_1000hz"] + RNS_CASES}

    print("\n2) Gazebo vs P1b, no ramp (after the 4 s move has finished)")
    for case in ["commanded_1000hz", "rns_1000hz"]:
        t, q, th0 = data[case]
        tr, xr = refs[case.split("_")[0]]
        print(f"   {case:17s} base {D(th0[-1]):+9.4f} deg | P1b {D(xr[-1, 0]):+9.4f} deg |"
              f" joints {D(q[-1]).round(3)} | P1b {D(xr[-1, 1:]).round(3)}")

    print("\n3) RNS residual base rotation [mdeg] = ideal controller + joint-path gap + physics gap")
    for case in RNS_CASES:
        t, q, th0 = data[case]
        rate = float(case.split("_")[1][:-2])
        ideal = ideal_controller(rate, T_RAMP if case.endswith("ramp") else 0.0)
        path = path_integral(q)[-1]
        meas = th0[-1]
        print(f"   {case:16s} Gazebo {D(meas)*1e3:+8.3f} = ideal ctrl {D(ideal)*1e3:+8.3f}"
              f" + joint path {D(path-ideal)*1e3:+7.3f} + physics {D(meas-path)*1e3:+7.3f}")
    t, q, th0 = data["commanded_1000hz"]
    gap = th0[-1] - path_integral(q)[-1]
    print(f"   commanded: Gazebo {D(th0[-1])*1e3:+.1f} mdeg, physics gap {D(gap)*1e3:+.3f} mdeg"
          f" ({abs(gap / th0[-1]):.1e} relative)")

    # ---- figure
    fig, ax = plt.subplots(1, 3, figsize=(16, 4.8))
    for case, c, lab in (("commanded_1000hz", "C3", "commanded rates"),
                         ("rns_1000hz", "C0", "RNS-projected (1 kHz node)")):
        t, q, th0 = data[case]
        tr, xr = refs[case.split("_")[0]]
        ax[0].plot(t, D(th0), c, lw=3, alpha=0.5, label=f"{lab}, Gazebo")
        ax[0].plot(tr, D(xr[:, 0]), "k--", lw=1, label="P1b model" if c == "C3" else None)
    ax[0].set(title=r"Base attitude $\theta_0$ (base not controlled)", xlabel="t [s]", ylabel="deg")
    ax[0].legend(fontsize=8)
    t, q, _ = data["rns_1000hz"]
    tr, xr = refs["rns"]
    for j in range(3):
        ax[1].plot(t, D(q[:, j]), f"C{j}", lw=3, alpha=0.5, label=f"q{j+1} Gazebo")
        ax[1].plot(tr, D(xr[:, 1 + j]), "k--", lw=1, label="P1b" if j == 0 else None)
    ax[1].set(title="Joint angles under the RNS controller", xlabel="t [s]", ylabel="deg")
    ax[1].legend(fontsize=8)
    for case, c, ls in (("rns_1000hz", "C0", "--"), ("rns_100hz", "C1", "--"), ("rns_20hz", "C2", "--"),
                        ("rns_1000hz_ramp", "C0", "-"), ("rns_100hz_ramp", "C1", "-"),
                        ("rns_20hz_ramp", "C2", "-")):
        t, q, th0 = data[case]
        lab = case.split("_")[1].replace("hz", " Hz") + (", 0.2 s ramp" if case.endswith("ramp") else ", step start")
        ax[2].plot(t, D(th0) * 1e3, color=c, ls=ls, lw=1.8, label=lab)
    ax[2].axhline(0, color="k", lw=0.5)
    ax[2].set(title="RNS residual base rotation (Gazebo)", xlabel="t [s]", ylabel="mdeg")
    ax[2].legend(fontsize=7, ncol=2)
    fig.suptitle("Step 3: Reaction Null Space control of the 3-link arm in zero-g Gazebo vs Project 1b"
                 "  (commanded: -14.4 deg, RNS: a few mdeg)")
    fig.tight_layout()
    out = os.path.join(FIG, "step3_rns_gazebo.png")
    fig.savefig(out, dpi=130)
    print(f"\nfigure: {out}")


if __name__ == "__main__":
    main()
