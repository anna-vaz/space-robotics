"""Post-process an Ansys PSD run (psd_<axis>.out + beams_<axis>.csv) into a margin-of-safety table.

Random vibration margins (GEVS / NASA-STD-5001B practice):
  design stress = 3-sigma = 3 x 1-sigma RMS;  MS_y = Fty / (FS_y * sigma_3) - 1, FS_y = 1.25
                                               MS_u = Ftu / (FS_u * sigma_3) - 1, FS_u = 1.4
Beams (spacers, braces): sigma = |N|/A + sqrt(My^2 + Mz^2)/Z from the 1-sigma end forces (each PSD component is an
independent 1-sigma value; the root-sum-square of the two bending moments is exact for a round section bent in one
plane and is the usual engineering combination). Shells: MAPDL 1-sigma von Mises per part.
Allowables: MMPDS-type A-basis sheet/plate values (lower than the SOLIDWORKS library values):
  6061-T6  Ftu 290  Fty 241 MPa      7075-T6  Ftu 524  Fty 462 MPa
"""
import csv, json, math, os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
ALLOW = {"6061": (241e6, 290e6), "7075": (462e6, 524e6)}
FSY, FSU = 1.25, 1.4

def margins(s3, alloy):
    fty, ftu = ALLOW[alloy]
    return fty / (FSY * s3) - 1, ftu / (FSU * s3) - 1

def post(axis, kind="psd"):
    """kind psd: 1-sigma results x 3; kind static: 10 g limit stresses x 1"""
    k3 = 3 if kind == "psd" else 1
    out = open(os.path.join(HERE, f"{kind}_{axis}.out"), errors="ignore").read()
    mp = json.load(open(os.path.join(HERE, "model_map.json")))
    labels = {int(m.group(1)): m.group(2) for m in re.finditer(r"^! mat (\d+): (.*)$", open(os.path.join(HERE, "model.inp")).read(), re.M)}
    rows = {int(float(r["elem"])): {k: float(v) for k, v in r.items()} for r in csv.DictReader(open(os.path.join(HERE, f"beams_{axis}.csv" if kind == "psd" else f"beams_static_{axis}.csv")))}
    res = []
    # spacers: round 5 mm 6061-T6
    A, Z = math.pi * 2.5e-3 ** 2, math.pi * (2.5e-3) ** 3 / 4
    for tag, els in mp["rod_elems"].items():
        best = None
        for e in els:
            r = rows.get(e)
            if not r: continue
            for end in "IJ":
                s = abs(r["FX" + end]) / A + math.hypot(r["MY" + end], r["MZ" + end]) / Z
                if not best or s > best[0]: best = (s, abs(r["FX" + end]), math.hypot(r["MY" + end], r["MZ" + end]))
        s1, N, M = best
        res.append(("spacer " + tag.split(" @")[0][4:] + " mm" + tag[tag.index(" @"):], "6061", s1, N, M))
    # braces: rect 6 x 3 mm 7075-T6 (bending about both axes: |My|/Zy + |Mz|/Zz, conservative)
    Ab = 6e-3 * 3e-3; Zy, Zz = 6e-3 * 3e-3 ** 2 / 6, 3e-3 * 6e-3 ** 2 / 6
    bmax = None
    for e in mp["brace_elems"]:
        r = rows.get(e)
        if not r: continue
        for end in "IJ":
            s = abs(r["FX" + end]) / Ab + abs(r["MY" + end]) / Zy + abs(r["MZ" + end]) / Zz
            if not bmax or s > bmax[0]: bmax = (s, abs(r["FX" + end]), e)
    res.append(("X-braces (worst element)", "7075", bmax[0], bmax[1], 0.0))
    # shells
    for m in re.finditer(r"^\s*SHELLMAT\s+(\d+)\s+([0-9.Ee+-]+)\s+([0-9.Ee+-]+)", out, re.M):
        lab = labels.get(int(m.group(1)), "?")
        alloy = "7075" if "7075" in lab else "6061" if ("6061" in lab or "plate" in lab or "bulkhead" in lab) else None
        if alloy is None: continue                                   # FR4 boards / solar panels: no metal allowable
        part = lab[lab.find("[") + 1: lab.find(" x")] if "[" in lab else lab
        res.append((part, alloy, float(m.group(2)) * 1e6, 0.0, 0.0))
    umax = re.search(r"^\s*RES UMAX_MM\s+([0-9.Ee+-]+)", out, re.M)
    react = re.search(r"^\s*RES REACT\s+(\S+)\s+(\S+)\s+(\S+)", out, re.M)
    if kind == "psd":
        lines = [f"Ansys PSD, GEVS qualification 14.1 grms, axis {axis}, 5 % damping, both rail ends fixed",
                 f"1-sigma max displacement (relative) {float(umax.group(1)):.4f} mm" if umax else "",
                 f"{'item':48s} {'alloy':5s} {'1-sig MPa':>9s} {'3-sig MPa':>9s} {'MS yield':>9s} {'MS ult':>8s}"]
    else:
        lines = [f"Ansys static 10 g, axis {axis}, cantilever (rail ends at Z = 0 fixed)",
                 "reaction sum (N): " + " ".join(f"{float(v):.2f}" for v in react.groups()) + "  (expected 392.27 N)" if react else "",
                 f"max displacement {float(umax.group(1)):.4f} mm" if umax else "",
                 f"{'item':48s} {'alloy':5s} {'':>9s} {'limit MPa':>9s} {'MS yield':>9s} {'MS ult':>8s}"]
    for name, alloy, s1, N, M in sorted(res, key=lambda t: -t[2] / ALLOW[t[1]][0]):
        my, mu = margins(k3 * s1, alloy)
        lines.append(f"{name[:48]:48s} {alloy:5s} {(s1 / 1e6 if k3 == 3 else float('nan')):9.2f} {k3 * s1 / 1e6:9.1f} {my:+9.2f} {mu:+8.2f}")
    txt = "\n".join(l for l in lines if l)
    open(os.path.join(HERE, "..", "results", f"ansys_{kind}_{axis}.txt"), "w").write(txt + "\n")
    print(txt)

if __name__ == "__main__":
    post(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "psd")
