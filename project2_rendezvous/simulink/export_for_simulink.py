"""Export the step 3 (Part B) attitude scenario and Python results for the Simulink cross-check.

Writes simulink/crosscheck_data.mat with
  parameters : J, K, P, U_MAX, n, beta0, omega0
  Python run : t, beta, omega, u   (same solver tolerances as step 3)
"""
import os
import sys

import numpy as np
from scipy.integrate import solve_ivp
from scipy.io import savemat

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import attitude as att   # noqa: E402
import orbit as o        # noqa: E402


def reference(t):
    """Same reference as step 3: docking axis along +y_LVLH, b3 along the orbit normal."""
    rt, vt = o.target_state(t)
    LN = o.eci_to_lvlh_dcm(rt, vt)
    r1, r3 = LN[1], LN[2]
    return np.vstack([r1, np.cross(r3, r1), r3]), np.array([0, 0, o.N]), np.zeros(3)


def rhs(t, x):
    beta, omega = x[:4], x[4:]
    RN, w_r, w_r_dot = reference(t)
    u = att.control_torque(beta, omega, RN, w_r, w_r_dot, saturate=True)
    bd, wd = att.rigid_body_rhs(beta, omega, u)
    return np.concatenate([bd, wd])


RN0 = reference(0.0)[0]
beta0 = att.dcm2ep(att.ep2dcm(att.prv2ep([0.3, -1, 0.5], np.radians(150))) @ RN0)
omega0 = np.radians([1.0, -0.5, 0.8])
t = np.linspace(0, 600, 3001)
sol = solve_ivp(rhs, (0, t[-1]), np.concatenate([beta0, omega0]), t_eval=t,
                method="DOP853", rtol=1e-11, atol=1e-12)
beta, omega = sol.y[:4].T, sol.y[4:].T
u = np.array([att.control_torque(b, w, *reference(tk)) for tk, b, w in zip(t, beta, omega)])

savemat(os.path.join(HERE, "crosscheck_data.mat"), dict(
    J=att.J, K=att.K_GAIN, P=att.P_GAIN, U_MAX=att.U_MAX, n=o.N,
    beta0=beta0, omega0=omega0, t=t, beta_py=beta, omega_py=omega, u_py=u))
print(f"exported {len(t)} samples; beta0 = {beta0.round(6)}, K = {att.K_GAIN:.4f}, P = {att.P_GAIN:.4f}")
