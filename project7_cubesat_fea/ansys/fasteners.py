"""NASA-STD-5020A style check of the M3 screws at every spacer end, loads from the Ansys PSD runs (3-sigma).

Joint (standard PC/104-style CubeSat stack): 5 mm round 6061-T6 spacer, M3 A2-70 stainless screw through the board
into the spacer thread, the spacer end face (Ø5 mm) clamping the board.
Loads per spacer end, per axis, from the BEAM188 end forces (1-sigma, ansys/beams_<axis>.csv) x 3:
  axial F, shear V = RSS(Vy, Vz), moment M = RSS(My, Mz)
  bolt tension from the moment: the end face rocks about its edge, lever = face radius r = 2.5 mm
     Pt = |F| + M / r      (conservative: no credit for the board's bending stiffness)
Preload: torque T, nut factor K, uncertainty Gamma (no preload measurement, unlubricated): NASA-STD-5020A 4.5
  Pp_max = (1 + G) T / (K d),  Pp_min = (1 - G) T / (K d) - relaxation (5 %)
Checks (FS: yield 1.25, ultimate 1.4, separation 1.2 (NASA-STD-5020A Table 1, no test verification of the joint
gapping at 1.4 would be required if separation were safety-critical; it is not: the stack is redundant x4)):
  separation    : Pp_min >= FS_sep * Pt                       MS_sep  = Pp_min / (1.2 Pt) - 1
  tension yield : Pp_max + n*phi * FS_y * Pt <= Pty           (phi from bolt / spacer stiffness, n = 1 conservative)
  ultimate      : (Rt)^2 + (Rs)^2 <= 1 with Rt = 1.4 Pt / Ptu, Rs = 1.4 V / Psu (threads in the shear plane), preload
                  excluded (ductile A2-70, NASA-STD-5020A 6.1)
  slip          : mu * Pp_min vs V (mu = 0.2), informative: if it slips, shear is carried by the screw (checked above)
  thread strip  : M3 internal thread in 6061-T6 spacer, engagement Le, shear area per FED-STD-H28 approximation
"""
import csv, json, math, os

HERE = os.path.dirname(os.path.abspath(__file__))
d, p = 3.0, 0.5
As = math.pi / 4 * (d - 0.9382 * p) ** 2               # tensile stress area, 5.03 mm2
d3 = d - 1.2268 * p                                     # minor diameter
Ftu, Fty = 700.0, 450.0                                 # A2-70 (ISO 3506), MPa
Ptu, Pty = Ftu * As, Fty * As
Psu = 0.62 * Ftu * math.pi / 4 * d3 ** 2                # threads in the shear plane
T, K, G, RELAX = 0.80, 0.20, 0.35, 0.05                 # N m, nut factor, preload uncertainty, relaxation
Pnom = T / (K * d * 1e-3)
Pp_max, Pp_min = (1 + G) * Pnom, (1 - G) * Pnom * (1 - RELAX)
r_face = 2.5
# joint stiffness: bolt (steel, As over grip) vs clamped spacer annulus (Ø5 / Ø3.2 clearance, 6061)
kb, kc = 193e3 * As, 68.9e3 * math.pi / 4 * (5.0 ** 2 - 3.2 ** 2)
phi, n = kb / (kb + kc), 1.0
Fsu_al, Le = 0.6 * 290.0, 6.0                           # 6061-T6 shear ultimate ~ 0.6 Ftu; 2d engagement
A_strip = math.pi * d3 * Le * 0.75                      # internal-thread shear area (0.75 effective)
P_strip = Fsu_al * A_strip

mp = json.load(open(os.path.join(HERE, "model_map.json")))
rows, worst = [], {}
for A in "XYZ":
    B = {int(float(r["elem"])): {k: float(v) for k, v in r.items()} for r in csv.DictReader(open(os.path.join(HERE, f"beams_{A}.csv")))}
    for tag, els in mp["rod_elems"].items():
        for e, end in ((els[0], "bottom"), (els[-1], "top")):
            s = "I" if end == "bottom" else "J"; r = B[e]
            F, V, M = 3 * abs(r["FX" + s]), 3 * math.hypot(r["SY" + s], r["SZ" + s]), 3 * math.hypot(r["MY" + s], r["MZ" + s]) * 1e3
            Pt = F + M / r_face
            ms_sep = Pp_min / (1.2 * Pt) - 1
            ms_y = (Pty - Pp_max) / (n * phi * 1.25 * Pt) - 1
            Rt, Rs = 1.4 * Pt / Ptu, 1.4 * V / Psu
            ms_u = 1 / math.hypot(Rt, Rs) - 1
            ms_strip = P_strip / (1.4 * Pt) - 1
            rows.append((A, tag, end, F, V, M, Pt, ms_sep, ms_y, ms_u, ms_strip, 0.2 * Pp_min >= V))
key = lambda r: min(r[7], r[8], r[9], r[10])
rows.sort(key=key)
L = ["NASA-STD-5020A check, M3 A2-70 screws at the spacer ends, GEVS qualification random vibration (3-sigma loads, Ansys)",
     f"screw: As {As:.2f} mm2, Ptu {Ptu:.0f} N, Pty {Pty:.0f} N, Psu {Psu:.0f} N | torque {T} N m, K {K}, Gamma {G}: "
     f"Pp_min {Pp_min:.0f} N, Pp_max {Pp_max:.0f} N ({Pp_max / As:.0f} MPa = {Pp_max / Pty * 100:.0f} % of yield) | phi {phi:.2f}, n {n}",
     f"thread strip in 6061 spacer (Le {Le} mm): {P_strip:.0f} N",
     f"{'axis':4s} {'joint':42s} {'end':6s} {'F N':>6s} {'V N':>6s} {'M Nmm':>7s} {'Pt N':>6s} {'MS sep':>7s} {'MS y':>6s} {'MS ult':>7s} {'MS strip':>8s} no-slip"]
for r in rows[:16]:
    L.append(f"{r[0]:4s} {r[1][:42]:42s} {r[2]:6s} {r[3]:6.1f} {r[4]:6.1f} {r[5]:7.0f} {r[6]:6.0f} {r[7]:+7.2f} {r[8]:+6.2f} {r[9]:+7.2f} {r[10]:+8.2f} {'yes' if r[11] else 'NO'}")
L.append(f"... {len(rows)} joint-axis cases in total; minimum margins: separation {min(r[7] for r in rows):+.2f}, "
         f"yield {min(r[8] for r in rows):+.2f}, ultimate {min(r[9] for r in rows):+.2f}, thread strip {min(r[10] for r in rows):+.2f}; "
         f"slip in {sum(not r[11] for r in rows)} cases")
txt = "\n".join(L)
open(os.path.join(HERE, "..", "results", "fasteners_5020.txt"), "w").write(txt + "\n")
print(txt)
