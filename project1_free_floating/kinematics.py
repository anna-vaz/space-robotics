"""Forward kinematics of the planar free-floating robot.

State (generalized coordinates):
    q = [x0, y0, th0, q1, q2]
      x0, y0 : position of the base centre of mass in the inertial frame
      th0    : base attitude angle
      q1, q2 : arm joint angles (relative angles)
"""
import numpy as np
from params import b0, l1, l2, m0, m1, m2, M_TOTAL


def rot(theta):
    """2D rotation matrix (base frame -> inertial frame)."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def unit(angle):
    """Unit vector pointing along a given absolute angle."""
    return np.array([np.cos(angle), np.sin(angle)])


def positions(q):
    """Return the important points of the robot in the inertial frame."""
    x0, y0, th0, q1, q2 = q
    r0 = np.array([x0, y0])               # base centre of mass
    phi1 = th0 + q1                       # absolute angle of link 1
    phi2 = phi1 + q2                      # absolute angle of link 2

    j1 = r0 + rot(th0) @ np.array(b0)     # shoulder joint
    c1 = j1 + (l1 / 2) * unit(phi1)       # link 1 centre of mass
    j2 = j1 + l1 * unit(phi1)             # elbow joint
    c2 = j2 + (l2 / 2) * unit(phi2)       # link 2 centre of mass
    ee = j2 + l2 * unit(phi2)             # end effector

    return dict(r0=r0, j1=j1, c1=c1, j2=j2, c2=c2, ee=ee, phi1=phi1, phi2=phi2)


def system_com(q):
    """Centre of mass of the whole robot (base + both links)."""
    p = positions(q)
    return (m0 * p["r0"] + m1 * p["c1"] + m2 * p["c2"]) / M_TOTAL
