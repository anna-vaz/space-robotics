"""Project 7 inputs: CubeSat geometry, requirements, load environment, materials.

Every number is either taken from a standard (source given next to it; the PDFs
are in D:/Projects/papers/standards) or marked ASSUMPTION. An assumption is a
placeholder that a real mission takes from its launch provider / deployer ICD.

Units: SI (m, kg, s, Pa) unless the name says otherwise (_mm, _g, _MPa).
"""
import numpy as np

G0 = 9.80665  # m/s^2, standard gravity

# --------------------------------------------------------------------------
# Geometry -- CubeSat Design Specification (CDS) Rev 14.1, Cal Poly 2022
# --------------------------------------------------------------------------
CUBE_X = CUBE_Y = 0.100    # m, 100.0 mm envelope (CDS App. B drawing)
CUBE_Z = 0.3405            # m, 3U rail length 340.5 mm (CDS App. B)
RAIL_W = 0.0085            # m, CDS 2.2.5: rails >= 8.5 mm wide on each face
RAIL_T = 0.0015            # m, ASSUMPTION: rail = equal-leg angle, 1.5 mm wall
RAIL_END_MIN = 0.0065      # m, CDS 2.2.8: rail end contact >= 6.5 x 6.5 mm
M_MAX_3U = 6.00            # kg, CDS Table 1 (typical max for 3U)
CG_LIMIT = (0.02, 0.02, 0.07)  # m, CDS Table 2, +/- from geometric centre (3U)

# Design point from Project 6 (CubeSat ADCS): 4 kg, J ~ diag(0.041, 0.040, 0.007)
M_DESIGN = 4.0                       # kg
J_P6 = np.array([0.041, 0.040, 0.007])  # kg m^2 (Project 6 params)

WALL_T = 0.0010            # m, ASSUMPTION: 1.0 mm 6061 side panels (stiffness upper bound)
END_PLATE_T = 0.0020       # m, ASSUMPTION: 2.0 mm 6061 end plates

# PCB stack: PC/104-style boards (90.17 x 95.89 mm), 1.6 mm FR-4,
# held by M3 spacer rods through 4 corner holes.
PCB_A = 0.09017            # m
PCB_B = 0.09589            # m
PCB_T = 0.0016             # m
PCB_HOLE_INSET = 0.00508   # m, ASSUMPTION: holes 0.2 in from both edges (symmetric
                           # approximation of the PC/104 hole pattern)

# --------------------------------------------------------------------------
# Requirements
# --------------------------------------------------------------------------
F1_MIN = 100.0             # Hz, ASSUMPTION: first natural frequency > 100 Hz
                           # (typical deployer/launcher value; check the ICD)
QS_LIMIT_G = 10.0          # g, ASSUMPTION: quasi-static limit load, each axis
                           # separately (stand-in for the launch provider value)

# Factors of safety on limit loads, GSFC-STD-7000B (GEVS) Table 2.2-3, metallic:
FS = {
    "static": {"yield": 1.25, "ultimate": 1.4},
    "random": {"yield": 1.6, "ultimate": 1.8},   # applied to the 3-sigma peak
    "analysis_only": {"yield": 2.0, "ultimate": 2.6},  # note 3: no test
}
# NASA-STD-5001B Table 1 (protoflight): ultimate 1.4, yield 1.25, qual test 1.2

# --------------------------------------------------------------------------
# Random vibration -- GEVS Table 2.4-3, components (ELV) <= 22.7 kg
# Breakpoints (Hz, g^2/Hz); between them the ASD follows +6 / -6 dB/oct.
# Overall: qualification 14.1 grms, acceptance 10.0 grms (= qual - 3 dB).
# --------------------------------------------------------------------------
GEVS_QUAL = [(20.0, 0.026), (50.0, 0.16), (800.0, 0.16), (2000.0, 0.026)]
GEVS_ACC = [(20.0, 0.013), (50.0, 0.08), (800.0, 0.08), (2000.0, 0.013)]
GEVS_QUAL_GRMS = 14.1
GEVS_ACC_GRMS = 10.0
Q_ASSUMED = 10.0           # ASSUMPTION: amplification at resonance (zeta = 5 %)

# --------------------------------------------------------------------------
# Materials. Aluminium values = SOLIDWORKS 2025 library ("solidworks
# materials.sldmat"), so hand calcs and FEA use the same numbers.
# FR-4 is not in the library: ASSUMPTION, typical in-plane laminate values.
# --------------------------------------------------------------------------
MATERIALS = {
    "Al6061-T6": dict(E=69.0e9, nu=0.33, G=26.0e9, rho=2700.0, Sy=275e6, Su=310e6,
                      sw_name="6061-T6 (SS)"),
    "Al7075-T6": dict(E=72.0e9, nu=0.33, G=26.9e9, rho=2810.0, Sy=505e6, Su=570e6,
                      sw_name="7075-T6 (SN)"),
    "FR4": dict(E=20.0e9, nu=0.13, rho=1850.0, sw_name=None),
}
RAIL_MAT = "Al7075-T6"
WALL_MAT = "Al6061-T6"
