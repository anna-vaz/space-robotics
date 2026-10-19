"""Tumbling target (a piece of debris): a square plate with a grasp handle.

Before capture the target moves freely: constant velocity of its centre of mass
and constant spin (planar motion, no external forces).
The chaser's previous rendezvous has left the two spacecraft almost at rest
relative to each other, with the target a couple of metres from the arm.
"""
import numpy as np

M_T = 50.0                         # target mass [kg]
A_T = 1.0                          # side of the square plate [m]
I_T = M_T * (A_T**2 + A_T**2) / 12  # inertia about its centre [kg m^2]
HANDLE_R = 0.6                     # handle distance from the target centre [m]

C0 = np.array([1.9, 1.4])          # target centre at t = 0 [m]
V_T = np.array([0.0, 0.0])         # target centre velocity [m/s]
W_T = np.radians(3.0)              # spin rate [rad/s]  (3 deg/s, 2 min per turn)
T_FACE = 30.0                      # the handle faces the chaser (points at -135 deg) at t = 30 s
TH0 = np.radians(-135.0) - W_T * T_FACE   # handle angle at t = 0  (= 135 deg for 3 deg/s)


def set_target(mass, inertia_factor=1.0):
    """Change the (unknown in reality) mass and inertia of the debris."""
    global M_T, I_T
    M_T = mass
    I_T = inertia_factor * mass * (A_T**2 + A_T**2) / 12


def set_spin(deg_per_s):
    """Change the tumble rate, keeping the handle facing the chaser at t = T_FACE."""
    global W_T, TH0
    W_T = np.radians(deg_per_s)
    TH0 = np.radians(-135.0) - W_T * T_FACE


def state(t):
    """Target centre, angle, velocity and angular rate at time t (free motion)."""
    return C0 + V_T * t, TH0 + W_T * t, V_T, W_T


def handle(t):
    """Handle position, velocity, acceleration and outward unit normal at time t."""
    c, th, v, w = state(t)
    n = np.array([np.cos(th), np.sin(th)])            # outward direction (centre -> handle)
    p = c + HANDLE_R * n
    dn = w * np.array([-np.sin(th), np.cos(th)])      # derivative of n
    vel = v + HANDLE_R * dn
    acc = -HANDLE_R * w**2 * n                         # centripetal
    return p, vel, acc, n, dn


def corners(t):
    c, th, _, _ = state(t)
    h = A_T / 2
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    return (R @ np.array([[h, h], [-h, h], [-h, -h], [h, -h]]).T).T + c
