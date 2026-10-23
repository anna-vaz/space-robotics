"""Step 4 planning: search the arm's start configuration for the reactionless reach.

Phase A (reactionless approach) + phase B (final approach with spin synchronisation),
see reach.py. Same idea as Project 1b step 5: the RNS singularity (det A = 0) depends on the
configuration, so the start pose decides whether the end effector can reach the
moving handle without turning the base. Offline, exact zero-momentum kinematics,
midpoint integration (dt = 0.05 s) -- fast enough for a 12^3 grid on 11 processes.

    .venv\\Scripts\\python project5_ros2_gazebo\\step4_plan.py
"""
import itertools
import json
import os
import sys
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "zero_g_manipulator"))
from zero_g_manipulator import reach as R  # noqa: E402
from zero_g_manipulator.free_floating_model import P1B_ARM, PlanarRobot  # noqa: E402

rob = PlanarRobot(**P1B_ARM)
C = np.array([1.9, 1.4])                 # target centre (Project 3)
W = np.radians(3.0)                      # spin 3 deg/s
TH_T0 = np.radians(135.0)                # handle faces the chaser (-135 deg) after 30 s
HALF = 0.55                              # base half-size + margin [m]


def inside_base(q):
    """Is any arm point (joints, link midpoints, end effector) inside the base square?"""
    r0, joints, coms, _, ee = rob._chain(q)
    c, s = np.cos(q[2]), np.sin(q[2])
    pts = joints[1:] + coms + [ee]
    for p in pts:
        d = p - r0
        x, y = c * d[0] + s * d[1], -s * d[0] + c * d[1]
        if abs(x) < HALF and abs(y) < HALF:
            return True
    return False


def simulate(q0_deg, method="rns", dt=0.05, T=R.T2):
    """Both phases on exact zero-momentum kinematics; returns a summary dict (None if the
    start pose puts the arm inside the base)."""
    q = np.r_[0.0, 0.0, 0.0, np.radians(q0_deg)]
    if inside_base(q):
        return None
    pe0 = rob.end_effector(q)
    st = dict(thA=0.0, detA=np.inf, detB=np.inf, qd=0.0, hit=False, phi0=None, thT1=0.0)

    def f(q, t):
        ph, vh, n, dn = R.handle_state(C, TH_T0 + W * t, np.zeros(2), W)
        pr, vr = R.reference(t, pe0, ph, vh, n, dn)
        v = vr + R.KP * (pr - rob.end_effector(q))
        if t < R.T1:
            qdm, d = R.joint_rates(rob, q, v, method)
            st["detA"] = min(st["detA"], abs(d))
        else:
            if st["phi0"] is None:
                st["phi0"] = R.gripper_angle(q)
            phr, dphr = R.angle_reference(t, st["phi0"], TH_T0 + W * t, W)
            qdm, d = R.joint_rates_sync(rob, q, v, dphr + R.K_PHI * R.wrap(phr - R.gripper_angle(q)))
            st["detB"] = min(st["detB"], abs(d))
        return np.r_[rob.base_coupling(q) @ qdm, qdm]

    for k in range(int(round(T / dt))):
        t = k * dt
        k1 = f(q, t)
        k2 = f(q + dt / 2 * k1, t + dt / 2)
        q = q + dt * k2
        if t + dt <= R.T1 + 1e-9:
            st["thA"] = max(st["thA"], abs(q[2]))
            st["thT1"] = q[2]
        st["qd"] = max(st["qd"], np.abs(k1[3:]).max())
        st["hit"] = st["hit"] or inside_base(q)
    ph = R.handle_state(C, TH_T0 + W * T, np.zeros(2), W)[0]
    qd_end = f(q, T - 1e-6)
    rel_w = (rob.reaction_row(q) + 1.0) @ qd_end[3:] - W
    return dict(q0=[int(a) for a in q0_deg], thA_max=float(np.degrees(st["thA"])),
                th_T1=float(np.degrees(st["thT1"])), th_T2=float(np.degrees(q[2])),
                dth_B=float(np.degrees(q[2] - st["thT1"])),
                align_deg=float(np.degrees(R.wrap(R.gripper_angle(q) - (TH_T0 + W * T + np.pi)))),
                rel_w_deg=float(np.degrees(rel_w)), detA_min=float(st["detA"]), detB_min=float(st["detB"]),
                qd_max=float(st["qd"]), hits_base=bool(st["hit"]),
                err_mm=float(np.linalg.norm(rob.end_effector(q) - ph) * 1e3))


def main():
    grid = np.arange(-150, 181, 30)
    cands = list(itertools.product(grid, grid, grid))
    with Pool(11) as pool:
        res = [r for r in pool.map(simulate, cands, chunksize=20) if r is not None]
    ok = [r for r in res if not r["hits_base"] and r["qd_max"] < 0.5 and r["err_mm"] < 1.0
          and r["detA_min"] > 0.03 and r["detB_min"] > 0.05 and abs(r["align_deg"]) < 0.5]
    # total base rotation that the capture costs: approach (should be ~0) + spin synchronisation
    ok.sort(key=lambda r: r["thA_max"] + abs(r["dth_B"]))
    print(f"{len(cands)} start poses, {len(res)} start outside the base, {len(ok)} feasible")
    for r in ok[:10]:
        print(f"  q0 {r['q0']}: approach max|th0| {r['thA_max']:.3f} deg, sync phase dth0 {r['dth_B']:+.2f} deg,"
              f" min|detA| {r['detA_min']:.3f}, min|det sync| {r['detB_min']:.3f},"
              f" max qd {r['qd_max']:.2f}, align {r['align_deg']:+.3f} deg, rel spin {r['rel_w_deg']:+.3f} deg/s")
    with open(os.path.join(HERE, "results", "step4_plan.json"), "w") as fh:
        json.dump(ok, fh, indent=1)


if __name__ == "__main__":
    main()
