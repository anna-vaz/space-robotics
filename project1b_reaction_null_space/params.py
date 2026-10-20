"""Physical parameters of the REDUNDANT planar free-floating robot.

Same satellite base as project 1, but the arm now has THREE links instead of two.
Why a third joint?  A planar end-effector position has 2 coordinates.  With 2 joints
every joint motion is used up by the end-effector task, so the base reaction is
fixed by the path.  With 3 joints there is one spare degree of freedom that can be
spent on keeping the base still -- the idea behind the Reaction Null Space.

SI units throughout (m, kg, rad).
"""
import numpy as np

# ---- Base (satellite body): uniform square plate, identical to project 1 ----
m0 = 100.0
a0 = 1.0
I0 = m0 * (a0**2 + a0**2) / 12
b0 = (a0 / 2, 0.0)              # shoulder joint position in the base frame

# ---- Links: uniform slender rods (link 3 = short wrist link) ----
LENGTHS = np.array([1.0, 1.0, 0.5])
MASSES = np.array([10.0, 10.0, 5.0])
INERTIAS = MASSES * LENGTHS**2 / 12   # about each link's own centre

N_LINKS = len(LENGTHS)
M_TOTAL = m0 + MASSES.sum()
