"""Torque-level control of the redundant free-floating robot.

Step 3 worked with velocities: it chose joint RATES and assumed the motors could
produce them.  Here the joint motors produce TORQUES and the full dynamics of
model.py move the robot -- base included, nothing assumed.

Joint-space dynamics of a free-floating robot (base unactuated):
    qdd = M^-1 ([0; tau] - c)   ->   qdd_m = Y tau - [M^-1 c]_m,   Y = [M^-1]_mm
so the torque that produces a wanted joint acceleration a is exactly
    tau = H* (a + [M^-1 c]_m),   H* = Y^-1     (exact joint linearization, as in project 3)

Two-loop controller:
  outer (kinematic) loop: qd_des = path_tracking.joint_rates(q, t)   -- the step 3 laws,
      evaluated on the ACTUAL state, so end-effector and base-attitude feedback stay active
  inner loop:  a = d/dt(qd_des) + K_V (qd_des - qd_m)
      d/dt(qd_des) along the motion by a central difference of the outer law
"""
import numpy as np
from scipy.integrate import solve_ivp

import path_tracking as pt

K_V = 20.0       # 1/s, joint-rate loop
H_FD = 1e-5      # s, step of the central difference for d/dt(qd_des)


def joint_space_inertia(rob, q, qd):
    Minv = np.linalg.inv(rob.mass_matrix(q))
    Y = Minv[3:, 3:]
    bias = (Minv @ rob.coriolis(q, qd))[3:]
    return np.linalg.inv(Y), bias


def controller(rob, t, x, ref, method):
    q, qd = x[:6], x[6:]
    qd_des, detA = pt.joint_rates(rob, q, t, ref, method)
    # rate of change of qd_des along the current motion
    qp, qm = q + H_FD * qd, q - H_FD * qd
    acc_ff = (pt.joint_rates(rob, qp, t + H_FD, ref, method)[0]
              - pt.joint_rates(rob, qm, t - H_FD, ref, method)[0]) / (2 * H_FD)
    a = acc_ff + K_V * (qd_des - qd[3:])
    Hs, bias = joint_space_inertia(rob, q, qd)
    return Hs @ (a + bias), qd_des, detA


def simulate(rob, q0, ref, T_end, method, qd0=None, n_points=1001):
    qd0 = np.zeros(6) if qd0 is None else np.asarray(qd0, float)

    def f(t, x):
        tau = controller(rob, t, x, ref, method)[0]
        return np.concatenate([x[6:], rob.accelerations(x[:6], x[6:], tau)])

    sol = solve_ivp(f, (0, T_end), np.concatenate([q0, qd0]), method="DOP853",
                    rtol=1e-10, atol=1e-11, t_eval=np.linspace(0, T_end, n_points))
    if not sol.success:
        raise RuntimeError(sol.message)
    X = sol.y.T
    tau, qd_des, detA = zip(*(controller(rob, t, x, ref, method) for t, x in zip(sol.t, X)))
    return dict(t=sol.t, q=X[:, :6], qd=X[:, 6:], tau=np.array(tau), qd_des=np.array(qd_des),
                detA=np.array(detA), ee=np.array([rob.end_effector(x[:6]) for x in X]),
                ee_des=np.array([ref(t)[0] for t in sol.t]),
                h=np.array([rob.momentum(x[:6], x[6:]) for x in X]),
                T=np.array([rob.kinetic_energy(x[:6], x[6:]) for x in X]))
