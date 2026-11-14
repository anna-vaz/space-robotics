"""Industry-style idealised FE model of CubeSat_3U_v5 for Ansys MAPDL (Ansys cross-check of the SOLIDWORKS FEA).

Same design as the SOLIDWORKS v5 assembly (positions, sizes, masses read from cad/interior_spec.txt), different
idealisation:
  shells (SHELL181) at mid-surfaces : side-frame plates + rail flanges, shear panels, end plates, bulkheads,
                                       PCBs, solar panels
  beams (BEAM188)                    : X-braces (6 x 3 mm rect), stack spacers (5 mm round, real length)
  point masses (MASS21 + inertia)    : reaction wheels, battery cells, camera, antenna
  rigid links (CERIG)                : the same "bonded" assumption as the SOLIDWORKS model: spacer spiders
                                       (r 3.6 mm), plate edges to the frame, panels to pads/braces, flanges to the
                                       shear panels, point masses to their mounting footprint
Each part's density is scaled so its mass equals the CAD part mass (non-structural mass correction), so the
total (4.000 kg) and CoM match the CAD. Coordinates mm here, metres in APDL; X normal to the side frames,
Y normal to the shear panels, Z along the rails (0 .. 340.5).
Writes ansys/model.inp (PREP7) and ansys/model_map.json.  Env: AN_H = max element edge (mm, default 3).
"""
import json, math, os
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
SPEC = os.path.join(HERE, "..", "cad", "interior_spec.txt")
H = float(os.environ.get("AN_H", "3.0"))
L = 340.5
RHO_AL = 2.7e-3                                  # g/mm3
CAD_G = {"side_frame": 110.841, "shear_panel": 85.39, "end_plate": 49.256,
         "end_plate_top": 49.256 - math.pi * 21 ** 2 * 2.0 * RHO_AL}
AL7075, AL6061 = (71.7e9, 0.33, 2810.0), (68.9e9, 0.33, 2700.0)

# ------------------------------------------------------------------ spec
parts, comps = {}, []
for line in open(SPEC, encoding="utf8"):
    f = line.strip().split("|")
    if f[0] == "PART": parts[f[1]] = f
    elif f[0] == "COMP":
        comps.append((f[1], [float(v) for v in f[2].split(",")], [float(v) for v in f[3].split(",")], f[4]))
gmass = {k: float(v[-1]) for k, v in parts.items()}
# light case (AN_LIGHT=1): equipment back to its step-1 budget + harness smeared over it (interior_budget.json);
# real structural parts (6061-T6 (SS)) unchanged.  4.000 kg -> m_real + m_equip_nominal + harness = 3.565 kg
LIGHT = os.environ.get("AN_LIGHT") == "1"
if LIGHT:
    bud = json.load(open(os.path.join(HERE, "..", "results", "interior_budget.json")))
    k_light = (bud["m_equip_nominal"] + bud["harness"]) / (bud["m_equip_nominal"] * bud["scale"])
    for k, v in parts.items():
        if v[-3] != "6061-T6 (SS)": gmass[k] *= k_light
# PART|file|PLATE|a|b|t|holes|boss|mat|lib|g   or   PART|file|TUBE|R|Ri|length|mat|lib|g

# ------------------------------------------------------------------ grids
HARD = set()                                       # features that must be exact (structural edges, rod axes)
def grid(feat, lo, hi):
    raw = sorted({round(v, 4) for v in feat if lo - 1e-9 <= v <= hi + 1e-9} | {round(lo, 4), round(hi, 4)})
    pts = []
    for v in raw:                                  # merge features closer than 0.3 mm (no sliver elements)
        if pts and v - pts[-1] < 0.3:
            if v in HARD and pts[-1] not in HARD: pts[-1] = v
            continue
        pts.append(v)
    out = [pts[0]]
    for a, b in zip(pts, pts[1:]):
        n = max(1, math.ceil((b - a) / H - 1e-9))
        out += [round(a + (b - a) * i / n, 4) for i in range(1, n + 1)]
    return out

rods = sorted([(p[0], p[1], p[2], float(parts[fn][5]), fn) for fn, r, p, d in comps if fn.startswith("rod_")],
              key=lambda t: t[2])                                   # x, y, z_bottom, length
fx = [0.75, 1.5, 8.5, 91.5, 98.5, 99.25, 50.0]
fy = [0.75, 2.0, 2.5, 8.5, 91.5, 97.5, 98.0, 99.25, 50.0]
for x, y, *_ in rods: fx += [x - 2.5, x, x + 2.5]; fy += [y - 2.5, y, y + 2.5]
HARD.update(round(v, 4) for v in fx + fy)
plates = []                                                         # name, file, x0, x1, y0, y1, zb, t
for fn, r, p, d in comps:
    pr = parts[fn]
    if fn.startswith("pcb_") or "bulkhead" in fn or "bracket" in fn:
        plates.append((d, fn, p[0], p[0] + float(pr[3]), p[1], p[1] + float(pr[4]), p[2], float(pr[5])))
        fx += [p[0], p[0] + float(pr[3])]; fy += [p[1], p[1] + float(pr[4])]
    if fn.startswith(("rw_wheel", "camera", "antenna")):
        fx += [p[0], p[0] + float(pr[3])]; fy += [p[1], p[1] + float(pr[4])]
    if fn.startswith("battery_cell"):
        fy += [p[1] - 2.5, p[1], p[1] + 2.5]; fx += [p[0], p[0] + float(pr[5])]
plates += [("end plate -Z", "end_plate.SLDPRT", 1.5, 98.5, 2.5, 97.5, 7.0, 2.0),
           ("end plate +Z", "end_plate_top.SLDPRT", 1.5, 98.5, 2.5, 97.5, 331.5, 2.0)]
fz = [0, 4, 7, 12, 109.5, 117.5, 223, 231, 328.5, 333.5, 336.5, L] + [pl[6] + pl[7] / 2 for pl in plates]
GX, GY, GZ = grid(fx, 0.75, 99.25), grid(fy, 0.75, 99.25), grid(fz, 0.0, L)
sub = lambda G, lo, hi: grid([g for g in G if lo <= g <= hi], lo, hi)

# ------------------------------------------------------------------ model containers
nodes, gridnode = {}, {}
def node(x, y, z):                                  # merged grid node
    k = (round(x, 3), round(y, 3), round(z, 3))
    if k not in gridnode:
        gridnode[k] = len(nodes) + 1; nodes[gridnode[k]] = k
    return gridnode[k]
def free(x, y, z):                                  # never merged
    i = len(nodes) + 1; nodes[i] = (x, y, z); return i
look = lambda x, y, z: gridnode.get((round(x, 3), round(y, 3), round(z, 3)))

mats, matkey = [], {}
def mat(E, nu, rho, label):
    k = (E, nu, round(rho, 6), label)
    if k not in matkey: mats.append(k); matkey[k] = len(mats)
    return matkey[k]
secs, seckey = [], {}
def sec(*s):
    if s not in seckey: secs.append(s); seckey[s] = len(secs)
    return seckey[s]
shells, beams, masses, cer = [], [], [], []
slaved, masters = set(), set()
def tie(m, s):
    """rigid link master -> slave; a slave is constrained once and never used as a master (CE rules)"""
    if s == m or s in slaved or s in masters or m in slaved: return False
    slaved.add(s); masters.add(m); cer.append((m, s)); return True
geo = defaultdict(float)                             # kg per part at nominal density

def shell_grid(part, u, v, coord, thick, matid):
    used = set()
    for i in range(len(u) - 1):
        for j in range(len(v) - 1):
            t = thick((u[i] + u[i + 1]) / 2, (v[j] + v[j + 1]) / 2)
            if not t: continue
            q = [node(*coord(u[a], v[b])) for a, b in ((i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1))]
            shells.append([sec("SHELL", t), matid, *q, part]); used.update(q)
            geo[part] += (u[i + 1] - u[i]) * (v[j + 1] - v[j]) * t * 1e-9 * mats[matid - 1][2]
    return used

# ------------------------------------------------------------------ side frames
POCK = [(12, 109.5), (117.5, 223), (231, 328.5)]
PADZ = [(4, 12), (109.5, 117.5), (223, 231), (328.5, 336.5)]
def plate_t(y, z):
    inner = 8.5 < y < 91.5
    if inner and any(a < z < b for a, b in POCK): return None
    if inner and any(a < z < b for a, b in PADZ): return 3.0        # 1.5 plate + 1.5 panel pad
    return 1.5
m7075 = mat(*AL7075, "side frame 7075-T6")
brace_nodes = {1: [], 2: []}
for fr in (1, 2):
    T = (lambda x, y, z: (100 - x, 100 - y, z)) if fr == 2 else (lambda x, y, z: (x, y, z))
    part = f"side_frame-{fr}"
    shell_grid(part, GY, GZ, lambda y, z: T(0.75, y, z), plate_t, m7075)
    for yf in (0.75, 99.25):
        shell_grid(part, sub(GX, 0.75, 8.5), GZ, lambda x, z, yf=yf: T(x, yf, z), lambda x, z: 1.5, m7075)
    sb = sec("RECT", 6.0, 3.0)
    for z0, z1 in POCK:
        c = T(0.75, 50.0, (z0 + z1) / 2); cn = free(*c); brace_nodes[fr].append(cn)
        for ya, za in ((8.5, z0), (91.5, z1), (8.5, z1), (91.5, z0)):
            a = T(0.75, ya, za); prev = node(*a)
            n = max(3, math.ceil(math.dist(a, c) / H))
            for k in range(1, n + 1):
                p = tuple(a[i] + (c[i] - a[i]) * k / n for i in range(3))
                cur = cn if k == n else free(*p)
                if k < n: brace_nodes[fr].append(cur)
                beams.append([sb, m7075, prev, cur, free(p[0] + (5 if fr == 1 else -5), p[1], p[2]), part, "brace"])
                prev = cur
            geo[part] += math.dist(a, c) * 18 * 1e-9 * 2810
def rail_end(k):
    x, y, z = k
    return (abs(z) < 1e-6 or abs(z - L) < 1e-6) and (x <= 8.5001 or x >= 91.4999) and (y <= 8.5001 or y >= 91.4999)

# ------------------------------------------------------------------ shear panels + flange bolting
msp = mat(*AL6061, "shear panel 6061-T6")
GXS, GZS = sub(GX, 1.5, 98.5), sub(GZ, 7, 333.5)
for k, ys in ((1, 2.0), (2, 98.0)):
    shell_grid(f"shear_panel-{k}", GXS, GZS, lambda x, z, ys=ys: (x, ys, z), lambda x, z: 1.0, msp)
for (x, y, z), n in list(gridnode.items()):
    if (abs(y - 0.75) < 1e-6 or abs(y - 99.25) < 1e-6) and (1.5 <= x <= 8.5 or 91.5 <= x <= 98.5) and 7 <= z <= 333.5:
        m = look(x, 2.0 if y < 50 else 98.0, z)
        if m: tie(m, n)

# ------------------------------------------------------------------ plates in XY (end plates, bulkheads, PCBs)
pnodes, levels = {}, []
for name, fn, x0, x1, y0, y1, zb, t in plates:
    zm = zb + t / 2; levels.append((name, zb, zb + t, zm))
    if fn.startswith("pcb_"):
        mm = mat(20e9, 0.13, gmass[fn] * 1e-3 / ((x1 - x0) * (y1 - y0) * t * 1e-9), "FR4 smeared " + fn[4:-7])
    else:
        mm = mat(*AL6061, name)
    hole = {"bulkhead.SLDPRT": 25.0, "end_plate_top.SLDPRT": 21.0}.get(fn)
    thick = (lambda x, y, hole=hole, t=t: None if hole and math.hypot(x - 50, y - 50) < hole else t)
    pnodes[name] = shell_grid(name, sub(GX, x0, x1), sub(GY, y0, y1), lambda x, y, zm=zm: (x, y, zm), thick, mm)
    if fn.startswith("pcb_"): continue
    for n in sorted(pnodes[name]):                    # bonded edges
        x, y, z = nodes[n]
        if abs(x - 1.5) < 1e-6 or abs(x - 98.5) < 1e-6:
            m = look(0.75 if x < 50 else 99.25, y, z)
            if m and tie(m, n): continue
        if abs(y - 2.5) < 1e-6 or abs(y - 97.5) < 1e-6:
            m = look(x, 2.0 if y < 50 else 98.0, z)
            if m: tie(m, n)

# ------------------------------------------------------------------ solar panels (bonded, like SOLIDWORKS v5)
spf = [fn for fn in parts if fn.startswith("solar_panel")][0]
msol = mat(20e9, 0.13, gmass[spf] * 1e-3 / (83 * 332.5 * 1.5e-9), "solar panel smeared")
GZP = sub(GZ, 4, 336.5)
solar = {}
for k, xs, xf in ((1, -2.25, 0.75), (2, 102.25, 99.25)):
    solar[k] = shell_grid(f"solar_panel-X{k}", sub(GY, 8.5, 91.5), GZP, lambda y, z, xs=xs: (xs, y, z), lambda y, z: 1.5, msol)
    for n in sorted(solar[k]):
        x, y, z = nodes[n]
        on_pad = 8.5 <= y <= 91.5 and any(a <= z <= b for a, b in PADZ)
        if on_pad:
            m = look(xf, y, z)
            if m: tie(m, n)
for k, ys in ((1, 0.75), (2, 99.25)):
    used = shell_grid(f"solar_panel-Y{k}", sub(GX, 8.5, 91.5), GZP, lambda x, z, ys=ys: (x, ys, z), lambda x, z: 1.5, msol)
    for n in sorted(used):
        x, y, z = nodes[n]
        m = look(x, 2.0 if ys < 50 else 98.0, z)
        if m: tie(m, n)
for fr in (1, 2):                                      # braces bonded to the X solar panel (outer face contact)
    free_solar = [s for s in solar[fr] if s not in slaved]
    for n in brace_nodes[fr]:
        x, y, z = nodes[n]
        s = min(free_solar, key=lambda s: (nodes[s][1] - y) ** 2 + (nodes[s][2] - z) ** 2)
        if (nodes[s][1] - y) ** 2 + (nodes[s][2] - z) ** 2 < 2.5 ** 2: tie(s, n)

# ------------------------------------------------------------------ spacer rods + spiders
def level(z, top):
    for name, zb, zt, zm in levels:
        if abs((zt if top else zb) - z) < 1e-3: return name, zm
    raise ValueError(f"no plate face at z = {z}")
spider = {}
def attach(plate, zm, x, y, end_node):
    """first rod end at a location becomes the spider master (rigid to the plate nodes within 3.6 mm);
    further rod ends there are tied to it."""
    key = (plate, round(x, 3), round(y, 3))
    if key in spider:
        if not tie(spider[key], end_node): raise ValueError(f"rod tie failed at {key}")
        return
    spider[key] = end_node
    got = [n for n in pnodes[plate] if math.hypot(nodes[n][0] - x, nodes[n][1] - y) <= 3.6]
    nt = sum(tie(end_node, n) for n in got)
    if nt < 4: raise ValueError(f"spider {key}: {nt} nodes")
mrod = mat(*AL6061, "spacer 6061-T6")
srod = sec("CIRC", 2.5)
rod_elems = defaultdict(list)
for x, y, zb, ln, fn in rods:
    tag = f"rod {ln:g} @ ({x:g}, {y:g}) z {zb:g}-{zb + ln:g}"
    n = max(4, math.ceil(ln / H))
    ends = [free(x, y, zb)]
    for k in range(1, n + 1):
        cur = free(x, y, zb + ln * k / n)
        beams.append([srod, mrod, ends[-1], cur, free(x + 5, y, zb + ln * k / n), f"rod_{ln:g}", tag])
        rod_elems[tag].append(len(beams)); ends.append(cur)
    geo[f"rod_{ln:g}"] += math.pi * 6.25 * ln * 1e-9 * 2700
    lb, zmb = level(zb, top=True); lt, zmt = level(zb + ln, top=False)
    attach(lb, zmb, x, y, ends[0]); attach(lt, zmt, x, y, ends[-1])

# ------------------------------------------------------------------ point masses on their footprints
def fp(plate, pred): return [n for n in sorted(pnodes[plate]) if pred(nodes[n][0], nodes[n][1])]
def box_I(m, a, b, c): return (m * (b * b + c * c) / 12, m * (a * a + c * c) / 12, m * (a * a + b * b) / 12)
for fn, r, p, d in comps:
    pr = parts[fn]; m = gmass[fn] * 1e-3
    if fn.startswith("rw_wheel"):
        a, b, c = (float(v) for v in pr[3:6])
        cg, plate = (p[0] + a / 2, p[1] + b / 2, p[2] + c / 2), "RW bulkhead"
        foot = fp(plate, lambda x, y: p[0] <= x <= p[0] + a and p[1] <= y <= p[1] + b)
        I = box_I(m, a * 1e-3, b * 1e-3, c * 1e-3)
    elif fn.startswith("battery_cell"):
        R, ln = float(pr[3]) * 1e-3, float(pr[5]) * 1e-3          # cell axis along +X from p
        cg, plate = (p[0] + ln * 500, p[1], p[2]), "PCB battery"
        foot = fp(plate, lambda x, y: p[0] <= x <= p[0] + ln * 1e3 and abs(y - p[1]) <= 2.51)
        I = (m * R * R / 2, m * (3 * R * R + ln * ln) / 12, m * (3 * R * R + ln * ln) / 12)
    elif fn.startswith("camera"):
        vb, vl = 78 * 78 * 40, math.pi * 20 ** 2 * 60
        cg, plate = (p[0] + 39, p[1] + 39, p[2] + (vb * 20 + vl * 70) / (vb + vl)), "camera bracket = bulkhead 2 (2U rib)"
        foot = fp(plate, lambda x, y: p[0] <= x <= p[0] + 78 and p[1] <= y <= p[1] + 78)
        I = box_I(m, 0.078, 0.078, 0.10)
    elif fn.startswith("antenna"):
        cg, plate = (p[0] + 41, p[1] + 41, p[2] + 3), "end plate +Z"
        foot = fp(plate, lambda x, y: p[0] <= x <= p[0] + 82 and p[1] <= y <= p[1] + 82)
        I = box_I(m, 0.082, 0.082, 0.006)
    else:
        continue
    mn = free(*cg); masses.append((mn, m, *I, d))
    nt = sum(tie(mn, n) for n in foot)
    if nt < 4: raise ValueError(f"{d}: {nt} footprint nodes")

# ------------------------------------------------------------------ CAD mass correction
target = {f"side_frame-{k}": CAD_G["side_frame"] for k in (1, 2)}
target.update({f"shear_panel-{k}": CAD_G["shear_panel"] for k in (1, 2)})
target.update({"end plate -Z": CAD_G["end_plate"], "end plate +Z": CAD_G["end_plate_top"]})
for name, fn, *_ in plates:
    if not fn.startswith(("pcb_", "end_plate")): target[name] = gmass[fn]
for x, y, zb, ln, fn in rods: target[f"rod_{ln:g}"] = target.get(f"rod_{ln:g}", 0) + gmass[fn]
target.update({f"solar_panel-{k}": gmass[spf] for k in ("X1", "X2", "Y1", "Y2")})
scale = {p: target[p] * 1e-3 / geo[p] for p in target}
pm = {}
def pmat(part, m0):
    if (part, m0) not in pm:
        E, nu, rho, lab = mats[m0 - 1]; s = scale.get(part, 1.0)
        pm[(part, m0)] = mat(E, nu, rho * s, f"{lab} [{part} x{s:.3f}]")
    return pm[(part, m0)]
for e in shells: e[1] = pmat(e[6], e[1])
for e in beams: e[1] = pmat(e[5], e[1])

# ------------------------------------------------------------------ independent mass / CoM check
M, Mc = 0.0, [0.0, 0.0, 0.0]
partM = defaultdict(float)
def add(m, c, part):
    global M
    M += m; partM[part] += m
    for i in range(3): Mc[i] += m * c[i]
for s, m, a, b, c, d, part in shells:
    P = [nodes[q] for q in (a, b, c, d)]
    u = [P[2][i] - P[0][i] for i in range(3)]; v = [P[3][i] - P[1][i] for i in range(3)]
    A = 0.5 * math.sqrt((u[1] * v[2] - u[2] * v[1]) ** 2 + (u[2] * v[0] - u[0] * v[2]) ** 2 + (u[0] * v[1] - u[1] * v[0]) ** 2)
    add(A * secs[s - 1][1] * 1e-9 * mats[m - 1][2], [sum(p[i] for p in P) / 4 for i in range(3)], part)
for s, m, i, j, o, part, tag in beams:
    A = secs[s - 1][1] * secs[s - 1][2] if secs[s - 1][0] == "RECT" else math.pi * secs[s - 1][1] ** 2
    add(A * math.dist(nodes[i], nodes[j]) * 1e-9 * mats[m - 1][2], [(nodes[i][q] + nodes[j][q]) / 2 for q in range(3)], part)
for n, m, *_r, d in masses: add(m, nodes[n], d)
cog = [c / M for c in Mc]

# ------------------------------------------------------------------ write APDL
with open(os.path.join(HERE, "model.inp"), "w") as f:
    w = f.write
    w("! generated by build_model.py from cad/interior_spec.txt -- do not edit\n/prep7\n")
    w("et,1,181\net,2,188\nkeyopt,2,3,2\net,3,21\n")
    for i, (E, nu, rho, lab) in enumerate(mats, 1):
        w(f"! mat {i}: {lab}\nmp,ex,{i},{E:.6e}\nmp,prxy,{i},{nu}\nmp,dens,{i},{rho:.6f}\n")
    for i, s in enumerate(secs, 1):
        if s[0] == "SHELL": w(f"sectype,{i},shell\nsecdata,{s[1] * 1e-3:.6e}\n")
        elif s[0] == "RECT": w(f"sectype,{i},beam,rect\nsecdata,{s[1] * 1e-3:.6e},{s[2] * 1e-3:.6e}\n")
        else: w(f"sectype,{i},beam,csolid\nsecdata,{s[1] * 1e-3:.6e},8\n")
    for n, (x, y, z) in nodes.items(): w(f"n,{n},{x * 1e-3:.8e},{y * 1e-3:.8e},{z * 1e-3:.8e}\n")
    eid, prev = 0, None
    w("type,1\n")
    for s, m, a, b, c, d, part in shells:
        eid += 1
        if (s, m) != prev: w(f"mat,{m}\nsecnum,{s}\n"); prev = (s, m)
        w(f"en,{eid},{a},{b},{c},{d}\n")
    first_beam = eid + 1; prev = None
    w("type,2\n")
    for s, m, i, j, o, part, tag in beams:
        eid += 1
        if (s, m) != prev: w(f"mat,{m}\nsecnum,{s}\n"); prev = (s, m)
        w(f"en,{eid},{i},{j},{o}\n")
    w("type,3\n")
    for n, m, Ix, Iy, Iz, d in masses:
        eid += 1
        w(f"r,{eid},{m:.6e},{m:.6e},{m:.6e},{Ix:.6e},{Iy:.6e},{Iz:.6e}\nreal,{eid}\nen,{eid},{n}\n")
    for m, s in cer: w(f"cerig,{m},{s},all\n")
    w("nsel,none\n")
    for k, n in gridnode.items():
        if rail_end(k): w(f"nsel,a,node,,{n}\n")
    w("cm,RAILENDS,node\nnsel,r,loc,z,-1e-6,1e-6\ncm,RAILEND0,node\nallsel\nfinish\n")

json.dump(dict(nodes=len(nodes), shells=len(shells), beams=len(beams), masses=len(masses), cerig=len(cer),
               mass_kg=M, cog_mm=cog, h_mm=H, part_mass_g={k: round(v * 1e3, 3) for k, v in sorted(partM.items())},
               scale={k: round(v, 4) for k, v in scale.items()}, first_beam=first_beam,
               rod_elems={k: [first_beam - 1 + e for e in v] for k, v in rod_elems.items()},
               brace_elems=[first_beam + i for i, b in enumerate(beams) if b[6] == "brace"]),
          open(os.path.join(HERE, "model_map.json"), "w"), indent=1)
print(f"nodes {len(nodes)}, shells {len(shells)}, beams {len(beams)}, point masses {len(masses)}, rigid links {len(cer)}")
print(f"light case, equipment x{k_light:.4f}" if LIGHT else "design case")
print(f"mass {M * 1e3:.2f} g (CAD 4000.00, light 3565.46), CoM ({cog[0]:.2f}, {cog[1]:.2f}, {cog[2]:.2f}) mm (CAD 50.00, 49.38, 173.79)")
print("density scale (model geometry -> CAD mass):", {k: round(v, 3) for k, v in scale.items()})
print("rail-end nodes:", sum(rail_end(k) for k in gridnode))
