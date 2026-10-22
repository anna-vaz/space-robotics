"""Plain-NumPy forward kinematics of the n-link planar free-floating robot.

q = [x0, y0, th0, q1, ..., qn]   (base CoM position, base attitude, relative joint angles)

This file is deliberately independent of the SymPy model in model.py: the checks in
step1_model.py compare the two, so a typo in one of them would show up as a mismatch.
"""
import numpy as np


def rot(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def unit(angle):
    return np.array([np.cos(angle), np.sin(angle)])


def positions(q, lengths, b0):
    """Joints, link centres, end effector and absolute link angles (inertial frame)."""
    q = np.asarray(q, float)
    r0 = q[:2]
    phi = q[2] + np.cumsum(q[3:])            # absolute angle of each link
    joints = [r0 + rot(q[2]) @ np.asarray(b0)]
    centres = []
    for L, ph in zip(lengths, phi):
        centres.append(joints[-1] + 0.5 * L * unit(ph))
        joints.append(joints[-1] + L * unit(ph))
    return dict(r0=r0, joints=np.array(joints), centres=np.array(centres),
                ee=joints[-1], phi=phi)


def system_com(q, lengths, masses, m0, b0):
    p = positions(q, lengths, b0)
    return (m0 * p["r0"] + masses @ p["centres"]) / (m0 + masses.sum())


def ik_with_wrist_angle(ee, phi3, lengths, b0):
    """3-link inverse kinematics, base at the origin with th0 = 0.

    The arm is redundant, so fix the absolute angle phi3 of the last link; the wrist
    point is then known and the first two links are a standard 2-link problem
    (elbow branch q2 > 0).  Returns q = [0, 0, 0, q1, q2, q3] or None if unreachable.
    """
    L1, L2, L3 = lengths
    w = np.asarray(ee, float) - L3 * unit(phi3) - np.asarray(b0)
    c2 = (w @ w - L1**2 - L2**2) / (2 * L1 * L2)
    if abs(c2) > 0.999:
        return None
    q2 = np.arccos(c2)
    q1 = np.arctan2(w[1], w[0]) - np.arctan2(L2 * np.sin(q2), L1 + L2 * np.cos(q2))
    q3 = (phi3 - q1 - q2 + np.pi) % (2 * np.pi) - np.pi
    return np.array([0.0, 0.0, 0.0, q1, q2, q3])
