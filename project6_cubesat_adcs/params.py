"""3U CubeSat parameters (body frame: x, y = short axes, z = long 3U axis).

Numbers are representative of flight 3U CubeSats (e.g. ~4 kg, CoM within the
2 cm box the CubeSat Design Specification allows, residual dipole of a few
mA m^2 after careful harness design).
"""
import numpy as np

# ---- Mass properties -----------------------------------------------------------
MASS = 4.0                                   # [kg]
DIMS = np.array([0.10, 0.10, 0.34])          # outer box [m] (x, y, z)
# Uniform box would give Ixx = Iyy = m/12 (0.1^2 + 0.34^2) = 0.0419, Izz = 0.0067;
# a real stack is slightly asymmetric, so add small products of inertia.
J = np.array([[0.0410, 0.0004, -0.0002],
              [0.0004, 0.0400,  0.0003],
              [-0.0002, 0.0003, 0.0070]])    # [kg m^2] about the CoM, body axes
J_INV = np.linalg.inv(J)

# CoM position relative to the geometric centre of the box (body axes) [m]
R_CM = np.array([0.005, -0.003, 0.010])

# ---- Surfaces (for drag and solar radiation pressure) ---------------------------
# six flat faces of the box: outward normal, area, centre (relative to the CoM)
_normals = np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]], float)
FACE_NORMALS = _normals
FACE_AREAS = np.array([DIMS[1] * DIMS[2]] * 2 + [DIMS[0] * DIMS[2]] * 2 + [DIMS[0] * DIMS[1]] * 2)
FACE_CENTRES = _normals * DIMS / 2 - R_CM      # face centre seen from the CoM

CD = 2.2          # drag coefficient (free-molecular flow, flat plates)
CR = 1.3          # radiation-pressure coefficient (1 = black body, 2 = perfect mirror)

# ---- Magnetic cleanliness -------------------------------------------------------
M_RESIDUAL = np.array([0.002, -0.001, 0.005])  # residual dipole [A m^2], |m| = 5.5 mA m^2

# ---- Actuators (used from step 2 on) --------------------------------------------
MTQ_MAX = 0.2      # magnetorquer dipole limit per axis [A m^2]
RW_HMAX = 1.5e-3   # reaction-wheel momentum capacity per axis [N m s]
RW_TMAX = 0.5e-3   # reaction-wheel torque limit per axis [N m]
