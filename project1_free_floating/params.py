"""Physical parameters of the planar free-floating space robot.

Model: a square satellite base with a 2-link arm mounted on its right face.
Everything moves in the x-y plane; there is no gravity and no external force.
SI units throughout (m, kg, rad).
"""

# ---- Base (satellite body): a uniform square plate ----
m0 = 100.0                     # base mass [kg]
a0 = 1.0                       # base side length [m]
I0 = m0 * (a0**2 + a0**2) / 12 # moment of inertia about its centre [kg m^2]

# Where the shoulder joint sits, expressed in the BASE frame
# (measured from the base centre of mass): middle of the right face.
b0 = (a0 / 2, 0.0)

# ---- Links: uniform slender rods ----
l1, l2 = 1.0, 1.0              # link lengths [m]
m1, m2 = 10.0, 10.0            # link masses [kg]
I1 = m1 * l1**2 / 12           # inertia about each link's own centre [kg m^2]
I2 = m2 * l2**2 / 12

M_TOTAL = m0 + m1 + m2
