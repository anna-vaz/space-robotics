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
