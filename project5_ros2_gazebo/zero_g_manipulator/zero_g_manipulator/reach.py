"""Reaching and capturing a tumbling target's handle with the free-floating 3-link arm.

Combines Project 3 (reach a moving handle, generalized Jacobian J*) with
Project 1b (Reaction Null Space: move the end effector without turning the base).

Phase A, 0 .. T1  -- reactionless approach to a stand-off point in front of the handle
    g(t)     = p_h(t) + D_STANDOFF n_h(t)      (moves with the handle)
    p_ref(t) = p_e0 + s(t) (g(t) - p_e0)       quintic s: 0 -> 1 over [0, T1]
    v        = v_ref + KP (p_ref - p_e)
    joint rates: every solution of J* qd = v is qd = J*^+ v + s n,  n = r1 x r2
        "minnorm" : s = 0                     -> base rotates freely
        "rns"     : s = detA (w_c - b J*^+ v) / (detA^2 + LAM^2),  w_c = -K_TH th0  (P1b)

Phase B, T1 .. T2 -- final approach + SPIN SYNCHRONISATION (both modes)
    A soft capture needs zero relative motion of gripper and handle, including the
    rotation: the gripper (last link) must point into the handle (angle th_t + pi) and turn
    with the target (rate w_t). With 3 joints the planar task [p_e; phi_e] uses all of them,
    so the base is no longer controlled in this phase (the cost of matching the spin).
        phi_e = th0 + q1 + q2 + q3,   phi_e' = (b + 1) qd   (zero momentum)
        [v; phi'_cmd] = [J*; b + 1] qd        (damped least squares)
"""
import numpy as np

T1, T2 = 20.0, 28.0          # end of the reactionless approach / of the synchronised final approach [s]
D_STANDOFF = 0.10            # pre-grasp stand-off [m] (as in Project 3)
KP = 2.0                     # end-effector position feedback [1/s]
K_PHI = 2.0                  # gripper angle feedback [1/s]
K_TH = 1.0                   # base attitude correction [1/s]
LAM = 0.02                   # damping of the base-attitude task (P1b)
LAM_B = 0.01                 # damping of the 3x3 phase-B solve
HANDLE_R = 0.6               # handle distance from the target centre [m] (Project 3)


def quintic(t, t0, t1):
    x = np.clip((t - t0) / (t1 - t0), 0.0, 1.0)
    s = 10 * x**3 - 15 * x**4 + 6 * x**5
    ds = (30 * x**2 - 60 * x**3 + 30 * x**4) / (t1 - t0) if t0 <= t < t1 else 0.0
    return s, ds


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def handle_state(c, th, v_c, w):
    """Handle position/velocity and outward normal (+ its rate) from the target CoM state."""
    n = np.array([np.cos(th), np.sin(th)])
    dn = w * np.array([-np.sin(th), np.cos(th)])
    return c + HANDLE_R * n, v_c + HANDLE_R * dn, n, dn


def reference(t, p_e0, p_h, v_h, n, dn):
    """Gripper position reference: stand-off approach (A), then close the stand-off (B)."""
    s2, ds2 = quintic(t, T1, T2)
    d, dd = D_STANDOFF * (1 - s2), -D_STANDOFF * ds2
    g = p_h + d * n
    dg = v_h + dd * n + d * dn
    s1, ds1 = quintic(t, 0.0, T1)
    return p_e0 + s1 * (g - p_e0), ds1 * (g - p_e0) + s1 * dg


def gripper_angle(q):
    return q[2] + np.sum(q[3:])


def angle_reference(t, phi_start, th_t, w_t):
    """Phase B: blend the gripper angle from phi_start to th_t + pi (pointing into the handle),
    ending with the target's rate w_t."""
    s2, ds2 = quintic(t, T1, T2)
    e = wrap(th_t + np.pi - phi_start)
    return phi_start + s2 * e, ds2 * e + s2 * w_t


def joint_rates(rob, q, v, method):
    """Phase A: qd_m for end-effector velocity v; returns (qd_m, det A)."""
    Js = rob.generalized_jacobian(q)
    b = rob.reaction_row(q)
    n = np.cross(Js[0], Js[1])
    detA = b @ n
    qd0 = np.linalg.pinv(Js) @ v
    if method == "minnorm":
        return qd0, detA
    w_c = -K_TH * q[2]
    s = detA * (w_c - b @ qd0) / (detA**2 + LAM**2)
    return qd0 + s * n, detA


def joint_rates_sync(rob, q, v, phid):
    """Phase B: qd_m for end-effector velocity v and gripper angular rate phid (DLS);
    returns (qd_m, det [J*; b+1])."""
    J = np.vstack([rob.generalized_jacobian(q), rob.reaction_row(q) + 1.0])
    det = np.linalg.det(J)
    qd = J.T @ np.linalg.solve(J @ J.T + LAM_B**2 * np.eye(3), np.r_[v, phid])
    return qd, det
