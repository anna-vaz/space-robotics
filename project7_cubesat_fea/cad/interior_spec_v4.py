"""Interior of the 3U CubeSat: geometry, positions and mass assignment.

Writes
  cad/interior_spec.txt       parts + placements, read by sw/Interior.cs
  <CAD dir>/CubeSat3U.sldmat  custom SOLIDWORKS material library (smeared FR-4,
                              mass dummies) whose densities make every item hit
                              its mass target
  results/interior_budget.json

Box coordinates (mm): X normal to the side-frame plates, Y normal to the shear
panels, Z along the rails (0..340.5). Cavity: X 1.5..98.5, Y 2.5..97.5,
Z 9..331.5 (between the end plates).

Mass rule (agreed with the user): real structural parts keep their real material
(structure, standoffs, RW plate, camera bracket). Every equipment item gets its
step-1 budget mass times one common scale factor s, chosen so that the total is
the 4.000 kg design mass of Project 6. The nominal case (s = 1, ~3.35 kg) is kept
for a later "light" load case.
"""
import json
import os
import numpy as np

CAD = r"D:\cad free projects\CubeSat 3U"
HERE = os.path.dirname(os.path.abspath(__file__))
M_TARGET = 4.000            # kg
RHO_AL = 2700.0

# ---------------------------------------------------------------- geometry
PCB_A, PCB_B, PCB_T = 95.89, 90.17, 1.6
PCB_X0, PCB_Y0 = 50 - PCB_A / 2, 50 - PCB_B / 2          # 2.055, 4.915
INSET = 5.08
CORNER_HOLES = [(INSET, INSET), (PCB_A - INSET, INSET), (INSET, PCB_B - INSET), (PCB_A - INSET, PCB_B - INSET)]
CENTRE_HOLE = [(PCB_A / 2, PCB_B / 2)]
R_M3 = 1.7
SO_RO, SO_RI = 2.5, 0.0     # standoff: solid 5 mm rod (tube + M3 screw; a 1.6 mm bore left a 0.1 mm
                            # ring against the 1.7 mm board holes that the mesher could not handle)
VERSION = "v4"              # v2: lateral bulkheads at the 1U ribs (step-3 finding)
                            # v3: X-braces in the side-frame pockets + solar panels 332.5 long (step-4 finding)
                            # v4: braces 3 mm thick (1.5 outward) + 1.5 mm panel pads; X panels move out 1.5 mm (buckling)

# stack levels (bottom face Z of each plate/board), see the comment in the README
# v2: bulkhead 1 at the first 1U rib (side-frame rib z 109.5..117.5), the camera bracket becomes
# bulkhead 2 at the second rib (z 223..231). Both span the full 97 x 95 cavity and are bonded/bolted
# to the side frames and shear panels, so the stack is held laterally at 4 stations, not 2.
Z = dict(rw_plate=55.0, battery=73.0, mtq=98.0, bulkhead=112.0, eps=128.0, obc=143.0, adcs=158.0,
         radio=173.0, payload=188.0, cam_bracket=225.5)
BH_A, BH_B = 97.0, 95.0                 # bulkhead = cavity cross-section
BH_HOLES = [(5.635, 7.495), (91.365, 7.495), (5.635, 87.505), (91.365, 87.505)]   # = end-plate stack holes
Z_END_BOT, Z_END_TOP = 9.0, 331.5      # inner faces of the end plates
T_AL = 3.0                              # RW plate and camera bracket thickness

I3 = (1, 0, 0, 0, 1, 0, 0, 0, 1)
R_CELL = (0, 0, 1, 0, 1, 0, -1, 0, 0)          # local z -> world X
R_SOLAR_X = (0, 0, 1, 1, 0, 0, 0, 1, 0)        # local x->Y, y->Z, z->X
R_SOLAR_Y = (1, 0, 0, 0, 0, 1, 0, 1, 0)        # local x->X, y->Z, z->Y

parts = {}       # file -> dict(kind, params, material, lib, volume_mm3, group, budget_kg)
comps = []       # (file, R, t, label)


def plate(file, a, b, t, holes=(), boss=None, **kw):
    v = a * b * t - sum(np.pi * r**2 * t for _, _, r in holes)
    if boss:
        v += np.pi * boss[2]**2 * boss[3]
    parts[file] = dict(kind="PLATE", a=a, b=b, t=t, holes=list(holes), boss=boss, volume=v, **kw)


def tube(file, ro, ri, L, **kw):
    parts[file] = dict(kind="TUBE", ro=ro, ri=ri, L=L, volume=np.pi * (ro**2 - ri**2) * L, **kw)


def cyl(file, r, L, **kw):
    parts[file] = dict(kind="TUBE", ro=r, ri=0.0, L=L, volume=np.pi * r**2 * L, **kw)


hole_list = lambda pts, r: [(x, y, r) for x, y in pts]

# --- circuit boards (smeared FR-4): name, budget kg (from step 1)
boards = [("mtq", 0.10), ("battery", 0.066), ("eps", 0.09), ("obc", 0.07), ("adcs", 0.06),
          ("radio", 0.08), ("payload", 0.10)]
for name, m in boards:
    holes = CORNER_HOLES + (CENTRE_HOLE if name == "battery" else [])
    plate(f"pcb_{name}.SLDPRT", PCB_A, PCB_B, PCB_T, hole_list(holes, R_M3),
          group="equipment", budget=m, mat=f"FR4 smeared {name} {VERSION}", E=20e9, nu=0.13)
    comps.append((f"pcb_{name}.SLDPRT", I3, (PCB_X0, PCB_Y0, Z[name]), f"PCB {name}"))

# --- aluminium plates in the stack (real material)
plate("rw_plate.SLDPRT", PCB_A, PCB_B, T_AL, hole_list(CORNER_HOLES + CENTRE_HOLE, R_M3),
      group="real", mat="6061-T6 (SS)")
comps.append(("rw_plate.SLDPRT", I3, (PCB_X0, PCB_Y0, Z["rw_plate"]), "RW plate"))
plate("bulkhead.SLDPRT", BH_A, BH_B, T_AL, hole_list(BH_HOLES, R_M3) + [(BH_A / 2, BH_B / 2, 25.0)],
      group="real", mat="6061-T6 (SS)")          # 50 mm centre cut-out for the harness
comps.append(("bulkhead.SLDPRT", I3, (1.5, 2.5, Z["bulkhead"]), "bulkhead 1 (1U rib)"))
plate("cam_bracket_v2.SLDPRT", BH_A, BH_B, T_AL, hole_list(BH_HOLES, R_M3),
      group="real", mat="6061-T6 (SS)")
comps.append(("cam_bracket_v2.SLDPRT", I3, (1.5, 2.5, Z["cam_bracket"]), "camera bracket = bulkhead 2 (2U rib)"))

# --- mass dummies
W = 36.0
plate("rw_wheel.SLDPRT", W, W, 30.0, group="equipment", budget=0.13, mat="Dummy reaction wheel " + VERSION, E=70e9, nu=0.33)
for cx, cy in [(29, 31), (71, 31), (50, 72)]:
    comps.append(("rw_wheel.SLDPRT", I3, (cx - W / 2, cy - W / 2, Z["rw_plate"] - 30.0), "reaction wheel"))
cyl("battery_cell.SLDPRT", 9.2, 65.0, group="equipment", budget=0.046, mat="Dummy battery cell " + VERSION, E=70e9, nu=0.33)
for cy in (18.0, 36.5, 63.5, 82.0):
    comps.append(("battery_cell.SLDPRT", R_CELL, (17.5, cy, Z["battery"] + PCB_T + 9.2), "battery cell"))
plate("camera_v2.SLDPRT", 78.0, 78.0, 40.0, boss=(39.0, 39.0, 20.0, 60.0),   # 100 mm tall to fit above rib 2
      group="equipment", budget=1.00, mat="Dummy camera " + VERSION, E=70e9, nu=0.33)
comps.append(("camera_v2.SLDPRT", I3, (11.0, 11.0, Z["cam_bracket"] + T_AL), "camera + lens"))
plate("antenna.SLDPRT", 82.0, 82.0, 6.0, group="equipment", budget=0.08, mat="Dummy antenna " + VERSION, E=70e9, nu=0.33)
comps.append(("antenna.SLDPRT", I3, (9.0, 9.0, 333.5), "antenna deployer"))
plate("solar_panel_v3.SLDPRT", 83.0, 332.5, 1.5, group="equipment", budget=0.12,   # ends land on the 12 mm end bands
      mat="Solar panel smeared " + VERSION, E=20e9, nu=0.13)
comps += [("solar_panel_v3.SLDPRT", R_SOLAR_X, (-3.0, 8.5, 4.0), "solar panel -X"),
          ("solar_panel_v3.SLDPRT", R_SOLAR_X, (101.5, 8.5, 4.0), "solar panel +X"),
          ("solar_panel_v3.SLDPRT", R_SOLAR_Y, (8.5, 0.0, 4.0), "solar panel -Y"),
          ("solar_panel_v3.SLDPRT", R_SOLAR_Y, (8.5, 98.5, 4.0), "solar panel +Y")]

# --- standoffs (6061 tubes) between levels
corner_xy = [(PCB_X0 + x, PCB_Y0 + y) for x, y in CORNER_HOLES]
gaps = [(Z_END_BOT, Z["rw_plate"]), (Z["rw_plate"] + T_AL, Z["battery"]),
        (Z["battery"] + PCB_T, Z["mtq"]), (Z["mtq"] + PCB_T, Z["bulkhead"]),
        (Z["bulkhead"] + T_AL, Z["eps"])]
for lo, hi in [("eps", "obc"), ("obc", "adcs"), ("adcs", "radio"), ("radio", "payload")]:
    gaps.append((Z[lo] + PCB_T, Z[hi]))
gaps += [(Z["payload"] + PCB_T, Z["cam_bracket"]), (Z["cam_bracket"] + T_AL, Z_END_TOP)]
for z0, z1 in gaps:
    f = f"rod_{VERSION}_{z1 - z0:g}".replace(".", "p") + ".SLDPRT"
    tube(f, SO_RO, SO_RI, round(z1 - z0, 4), group="real", mat="6061-T6 (SS)")
    for x, y in corner_xy:
        comps.append((f, I3, (x, y, z0), f"standoff {z1 - z0:g} mm"))
comps.append((f"rod_{VERSION}_15.SLDPRT", I3, (50.0, 50.0, Z["rw_plate"] + T_AL), "battery centre support"))

# ---------------------------------------------------------------- masses
rho = lambda p: RHO_AL if p["mat"] == "6061-T6 (SS)" else None
count = {}
for f, *_ in comps:
    count[f] = count.get(f, 0) + 1
# structure (already built, real): frames, panels, end plates incl. countersinks + aperture
csk = 4 * (np.pi * 1.66 / 3 * (3.36**2 + 3.36 * 1.7 + 1.7**2) - np.pi * 1.7**2 * 1.66)  # mm^3
m_structure = (2 * 110.841 +   # side frame v4: braces 3 mm + pads (v3 80.556, v1 61.838)
    0 + 2 * 85.39 + 2 * 49.467) / 1e3 - 2 * csk * RHO_AL * 1e-9 \
    - np.pi * 21.0**2 * 2.0 * RHO_AL * 1e-9
m_real = m_structure + sum(p["volume"] * 1e-9 * RHO_AL * count[f] for f, p in parts.items() if p["group"] == "real")
m_equip_nominal = sum(p["budget"] * count[f] for f, p in parts.items() if p["group"] == "equipment")
HARNESS = 0.11   # kg, harness + fasteners: in the step-1 budget, not modelled as geometry
s = (M_TARGET - m_real) / m_equip_nominal
for f, p in parts.items():
    if p["group"] == "equipment":
        p["target"] = p["budget"] * s
        p["rho"] = p["target"] / (p["volume"] * 1e-9)
    else:
        p["target"] = p["volume"] * 1e-9 * RHO_AL
        p["rho"] = RHO_AL

print(f"structure (real) {m_structure*1e3:.1f} g, other real parts {(m_real-m_structure)*1e3:.1f} g")
print(f"equipment nominal {m_equip_nominal*1e3:.1f} g + harness {HARNESS*1e3:.0f} g (not modelled)")
print(f"nominal total (light case) = {(m_real+m_equip_nominal+HARNESS)*1e3:.1f} g")
print(f"scale s = {s:.4f} -> design total {M_TARGET*1e3:.1f} g")
for f, p in sorted(parts.items()):
    print(f"  {f:26s} x{count[f]:2d}  V {p['volume']:10.1f} mm3  m {p['target']*1e3:8.2f} g  rho {p['rho']:8.0f}  {p['mat']}")

# ---------------------------------------------------------------- outputs
def fmt(v):
    return f"{v:.6f}".rstrip("0").rstrip(".")

lines = []
for f, p in parts.items():
    lib = "SOLIDWORKS Materials" if p["mat"] == "6061-T6 (SS)" else "CubeSat3U"  # library name; its folder is registered in SOLIDWORKS (sw/MatFix.cs)
    if p["kind"] == "PLATE":
        holes = ";".join(f"{fmt(x)}:{fmt(y)}:{fmt(r)}" for x, y, r in p["holes"]) or "-"
        boss = ":".join(fmt(v) for v in p["boss"]) if p["boss"] else "-"
        geo = f"{fmt(p['a'])}|{fmt(p['b'])}|{fmt(p['t'])}|{holes}|{boss}"
    else:
        geo = f"{fmt(p['ro'])}|{fmt(p['ri'])}|{fmt(p['L'])}"
    lines.append(f"PART|{f}|{p['kind']}|{geo}|{p['mat']}|{lib}|{p['target']*1e3:.4f}")
for f, R, t, label in comps:
    lines.append(f"COMP|{f}|{','.join(str(v) for v in R)}|{','.join(fmt(v) for v in t)}|{label}")
with open(os.path.join(HERE, "cad", f"interior_spec.txt"), "w") as fh:
    fh.write("\n".join(lines) + "\n")

mats = []
for f, p in parts.items():
    if p["mat"] == "6061-T6 (SS)":
        continue
    G = p["E"] / (2 * (1 + p["nu"]))
    strength = 3.1e8 if p["E"] < 50e9 else 5.0e8
    mats.append(f'''    <material name="{p['mat']}" matid="{900 + len(mats)}">
      <physicalproperties>
        <EX displayname="Elastic modulus" value="{p['E']:.1f}" />
        <NUXY displayname="Poisson's ratio" value="{p['nu']:.4f}" />
        <GXY displayname="Shear modulus" value="{G:.1f}" />
        <DENS displayname="Mass density" value="{p['rho']:.4f}" />
        <SIGXT displayname="Tensile strength" value="{strength:.1f}" />
        <SIGYLD displayname="Yield strength" value="{strength:.1f}" />
      </physicalproperties>
    </material>''')
xml = ('<?xml version="1.0" encoding="utf-16"?>\n'
       '<mstns:materials xmlns:mstns="http://www.solidworks.com/sldmaterials" '
       'xmlns:msdata="urn:schemas-microsoft-com:xml-msdata" '
       'xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" version="2008.03">\n'
       '  <classification name="CubeSat 3U">\n' + "\n".join(mats) + '\n  </classification>\n</mstns:materials>\n')
with open(os.path.join(CAD, "CubeSat3U.sldmat"), "w", encoding="utf-16") as fh:
    fh.write(xml)
with open(os.path.join(HERE, "results", "interior_budget.json"), "w") as fh:
    json.dump(dict(scale=s, m_structure=m_structure, m_real=m_real, m_equip_nominal=m_equip_nominal,
                   harness=HARNESS, m_nominal_total=m_real + m_equip_nominal + HARNESS,
                   parts={f: dict(count=count[f], target_g=p["target"] * 1e3, rho=p["rho"], mat=p["mat"])
                          for f, p in parts.items()}), fh, indent=1)
print(f"{len(parts)} parts, {len(comps)} components, {len(mats)} custom materials")
