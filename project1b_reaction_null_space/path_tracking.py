"""End-effector path tracking for the redundant free-floating robot (zero momentum).

Task: follow x_des(t) with the end effector, using the commanded velocity
    v = v_des + K_p (x_des - x_e)                (resolved-rate with position feedback)

J* (2x3) has a one-dimensional null space.  Its direction is
    n = r1 x r2        (r1, r2 = rows of J*; continuous, no SVD sign flips)
and every solution of J* qd_m = v is  qd_m = J*^+ v + s n.
The base rotation rate of that solution is
    w0 = b J*^+ v + (b . n) s,       and   b . n = det[J*; b] = det A.

Three ways to choose s:
  "minnorm"  s = 0                     smallest joint rates, base rotates freely
  "exact"    s = (w_c - b J^+ v)/det A  base follows w_c exactly (Reaction Null Space);
                                        s -> infinity where det A -> 0
  "priority" s = det A (w_c - b J^+ v) / (det A^2 + lam^2)
                                        end effector first, base second (damped):
                                        identical to "exact" far from the singularity,
                                        bounded joint rates near it, a small temporary
                                        base rotation instead
with the base-attitude correction  w_c = -K_th * th0  (drives th0 back to 0).
"""
import numpy as np
from scipy.integrate import solve_ivp

import rns

KP = 2.0       # 1/s, end-effector position feedback
K_TH = 1.0     # 1/s, base attitude correction (exact / priority)
LAM = 0.02     # m^2, damping of the base-attitude task (det A is ~0.1-0.17 at most)


def quintic(t, T):
    x = np.clip(t / T, 0.0, 1.0)
    s = 10 * x**3 - 15 * x**4 + 6 * x**5
    ds = (30 * x**2 - 60 * x**3 + 30 * x**4) / T if 0 <= t < T else 0.0
    return s, ds


def straight_line(x_start, x_goal, T):
    x_start, x_goal = np.asarray(x_start, float), np.asarray(x_goal, float)

    def ref(t):
        s, ds = quintic(t, T)
        return x_start + s * (x_goal - x_start), ds * (x_goal - x_start)
    return ref


def joint_rates(rob, q, t, ref, method, lam=None):
    """Returns qd_m and det A.  lam=None -> module-level LAM (so it can be swept)."""
    lam = LAM if lam is None else lam
    Js = rns.generalized_jacobian(rob, q)
    b = rns.reaction_row(rob, q)
    x_d, v_d = ref(t)
    v = v_d + KP * (x_d - rob.end_effector(q))
    n = np.cross(Js[0], Js[1])
    detA = b @ n
    qd0 = np.linalg.pinv(Js) @ v
    w_c = -K_TH * q[2]
    if method == "minnorm":
        s = 0.0
    elif method == "exact":
        s = (w_c - b @ qd0) / detA
    elif method == "priority":
        s = detA * (w_c - b @ qd0) / (detA**2 + lam**2)
    else:
        raise ValueError(method)
    return qd0 + s * n, detA


def track(rob, q0, ref, T_end, method, rate_limit=10.0, n_points=401):
    """Integrate the zero-momentum motion. Stops if a joint rate exceeds rate_limit."""
    def f(t, q):
        qd_m, _ = joint_rates(rob, q, t, ref, method)
        return rns.full_rates(rob, q, qd_m)

    def too_fast(t, q):
        return rate_limit - np.abs(joint_rates(rob, q, t, ref, method)[0]).max()
    too_fast.terminal = True

    sol = solve_ivp(f, (0, T_end), np.asarray(q0, float), method="DOP853", rtol=1e-10, atol=1e-12,
                    t_eval=np.linspace(0, T_end, n_points), events=too_fast)
    out = dict(t=sol.t, q=sol.y.T, stopped=sol.status == 1,
               t_stop=sol.t_events[0][0] if sol.status == 1 else None)
    rates, dets = zip(*(joint_rates(rob, q, t, ref, method) for t, q in zip(sol.t, sol.y.T)))
    out["qd_m"], out["detA"] = np.array(rates), np.array(dets)
    out["ee"] = np.array([rob.end_effector(q) for q in sol.y.T])
    out["ee_des"] = np.array([ref(t)[0] for t in sol.t])
    return out
