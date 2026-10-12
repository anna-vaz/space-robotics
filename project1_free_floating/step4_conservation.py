"""Step 4: verify the conservation laws of the free-floating robot.

With no external forces or torques on the system:
  * total linear momentum   P = sum(m_i v_i)                 is constant
  * total angular momentum  L = sum(m_i r_i x v_i + I_i w_i) is constant
  * so the system centre of mass moves in a straight line at constant speed
    (and stays put if P = 0)
Kinetic energy is NOT constant (the joint motors do work), but it must obey
the work-energy theorem:  T(t) - T(0) = integral of (tau . joint rates) dt.

We test two cases:
  A. start at rest (as in step 3)            -> P = 0, L = 0, CoM fixed
  B. start with the base drifting + spinning -> P, L nonzero but constant
"""
import os
import numpy as np
import sympy as sp
import matplotlib.pyplot as plt
from scipy.integrate import cumulative_trapezoid

import dynamics as d
import simulation as s
from kinematics import system_com
from params import m0, m1, m2, I0, I1, I2, M_TOTAL

HERE = os.path.dirname(os.path.abspath(__file__))

# ---- Momentum functions, built from the symbolic positions in dynamics.py ----
# velocity of each CoM = (d position / d q) * qd   (exact, no finite differences)
bodies = [(m0, I0, d.r0, d.th0), (m1, I1, d.c1, d.phi1), (m2, I2, d.c2, d.phi2)]
P_sym = sp.zeros(2, 1)
L_sym = 0
for m, I, r, ang in bodies:
    v = r.jacobian(d.q) * d.qd
    w = (sp.Matrix([ang]).jacobian(d.q) * d.qd)[0]
    P_sym += m * v
    L_sym += m * (r[0] * v[1] - r[1] * v[0]) + I * w      # 2D cross product (z component)
_P = sp.lambdify([d.q, d.qd], P_sym, "numpy")
_L = sp.lambdify([d.q, d.qd], L_sym, "numpy")


def check(name, q0=None, qd0=None):
    t, q, qd, tau = s.simulate(q0=q0, qd0=qd0)
    P = np.array([np.ravel(_P(qi, qdi)) for qi, qdi in zip(q, qd)])
    L = np.array([float(_L(qi, qdi)) for qi, qdi in zip(q, qd)])
    com = np.array([system_com(qi) for qi in q])
    T = np.array([d.kinetic_energy(qi, qdi) for qi, qdi in zip(q, qd)])

    # where the CoM SHOULD be if it moves at constant velocity P / M_total
    com_pred = com[0] + np.outer(t, P[0] / M_TOTAL)
    work = cumulative_trapezoid(np.sum(tau * qd[:, 3:], axis=1), t, initial=0.0)

    dP = np.abs(P - P[0]).max()
    dL = np.abs(L - L[0]).max()
    dcom = np.linalg.norm(com - com_pred, axis=1).max()
    dE = np.abs((T - T[0]) - work).max()
    print(f"--- Case {name} ---")
    print(f"  P(0) = {P[0].round(6)} kg m/s,   L(0) = {L[0]:.6f} kg m^2/s")
    print(f"  max |P(t) - P(0)|                  = {dP:.1e}")
    print(f"  max |L(t) - L(0)|                  = {dL:.1e}")
    print(f"  max CoM deviation from straight line = {dcom:.1e} m")
    print(f"  max |dT - motor work|              = {dE:.1e} J  (T changes by up to {np.abs(T - T[0]).max():.4f} J)")
    return dict(t=t, P=P, L=L, com=com, com_pred=com_pred, T=T, work=work)


if __name__ == "__main__":
    A = check("A: start at rest")
    B = check("B: base drifting + spinning",
              qd0=[0.02, -0.01, np.radians(2.0), 0.0, 0.0])

    # ---- Figure: conservation errors over time for both cases ----
    fig, ax = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
    for res, lab, c in ((A, "A (at rest)", "C0"), (B, "B (drifting)", "C1")):
        t = res["t"]
        ax[0, 0].semilogy(t, np.linalg.norm(res["P"] - res["P"][0], axis=1) + 1e-18, c, label=lab)
        ax[0, 1].semilogy(t, np.abs(res["L"] - res["L"][0]) + 1e-18, c, label=lab)
        ax[1, 0].semilogy(t, np.linalg.norm(res["com"] - res["com_pred"], axis=1) + 1e-18, c, label=lab)
        ax[1, 1].semilogy(t, np.abs(res["T"] - res["T"][0] - res["work"]) + 1e-18, c, label=lab)
    titles = ["|P(t) - P(0)|  [kg m/s]", "|L(t) - L(0)|  [kg m^2/s]",
              "CoM deviation from straight line [m]", "|dT - motor work|  [J]"]
    for a, ttl in zip(ax.flat, titles):
        a.set_title(ttl); a.grid(alpha=0.3, which="both"); a.legend(fontsize=8)
    for a in ax[1]:
        a.set_xlabel("time [s]")
    fig.suptitle("Conservation checks (all errors should be tiny)")
    fig.tight_layout()
    out = os.path.join(HERE, "figures", "step4_conservation.png")
    fig.savefig(out, dpi=150)
    print("saved", out)
