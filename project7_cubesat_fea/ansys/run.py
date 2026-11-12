"""Run MAPDL batch jobs for the CubeSat Ansys model and parse the results.

  python run.py modal  BC=1|0 NMODE=60   -> results/ansys_modal_<bc>.txt (frequencies, effective masses, mass check)
"""
import os, re, subprocess, sys, json, time

HERE = os.path.dirname(os.path.abspath(__file__))
MAPDL = r"C:\Program Files\ANSYS Inc\v222\ansys\bin\winx64\MAPDL.exe"
RES = os.path.join(HERE, "..", "results")

def run(inp, out, params, jobname, np_=4):
    """MAPDL batch with parameters prepended (written to a driver file)."""
    drv = os.path.join(HERE, f"_drv_{jobname}.inp")
    with open(drv, "w") as f:
        for k, v in params.items(): f.write(f"{k} = {v}\n")
        f.write(f"/input,{inp}\n")
    t0 = time.time()
    p = subprocess.run([MAPDL, "-b", "-np", str(np_), "-j", jobname, "-i", drv, "-o", out],
                       cwd=HERE, capture_output=True, text=True)
    txt = open(os.path.join(HERE, out), errors="ignore").read() if not params.get("_reuse") else open(os.path.join(HERE, out), errors="ignore").read()
    nerr = re.findall(r"NUMBER OF ERROR\s+MESSAGES ENCOUNTERED=\s*(\d+)", txt)
    return txt, time.time() - t0, int(nerr[-1]) if nerr else -1

def modal(bc, nmode):
    tag = {1: "bothends", 0: "cantilever"}[bc]
    if os.environ.get("PARSE_ONLY"):
        txt, dt, nerr = open(os.path.join(HERE, f"modal_{tag}.out"), errors="ignore").read(), 0, 0
    else:
        txt, dt, nerr = run("modal.inp", f"modal_{tag}.out", {"BC": bc, "NMODE": nmode}, f"modal_{tag}")
    freqs = [float(m.group(2)) for m in re.finditer(r"MODEFREQ\s+(\d+)\s+([0-9.Ee+-]+)", txt) if not m.group(0).startswith("/com")]
    freqs = [float(v) for v in re.findall(r"^\s*MODEFREQ\s+\d+\s+([0-9.Ee+-]+)", txt, re.M)]
    # participation-factor tables (one per excitation direction), effective mass ratio column
    eff = {}                                          # direction -> {mode: (effective mass kg, ratio to total)}
    for d in "XYZ":
        m = re.search(r"\*{5} PARTICIPATION FACTOR CALCULATION \*{5}\s+" + d + r"\s+DIRECTION(.*?)\n\s+sum", txt, re.S)
        eff[d] = {}
        for row in (m.group(1).splitlines() if m else []):
            f = row.split()
            if len(f) == 8 and f[0].isdigit(): eff[d][int(f[0])] = (float(f[5]), float(f[7]))
    mass = re.search(r"TOTAL MASS\s*=\s*([0-9.E+-]+)", txt)
    lines = [f"Ansys MAPDL modal, idealised shell/beam/mass model of CubeSat_3U_v5, BC = {tag}, solve {dt:.0f} s, errors {nerr}",
             f"model mass (MAPDL) {float(mass.group(1)) * 1e3 if mass else float('nan'):.2f} g",
             " mode   f (Hz)   eff. mass ratio X     Y      Z"]
    for i, f in enumerate(freqs, 1):
        rx, ry, rz = (eff[d].get(i, (0, 0))[1] for d in "XYZ")
        lines.append(f" {i:4d} {f:9.1f}        {rx:6.3f} {ry:6.3f} {rz:6.3f}")
    lines.append("  sum " + " " * 15 + " ".join(f"{sum(v[1] for v in eff[d].values()):6.3f}" for d in "XYZ"))
    s = "\n".join(lines)
    open(os.path.join(RES, f"ansys_modal_{tag}.txt"), "w").write(s + "\n")
    print(s)

if __name__ == "__main__":
    a = dict(x.split("=") for x in sys.argv[2:])
    if sys.argv[1] == "modal": modal(int(a.get("BC", 1)), int(a.get("NMODE", 60)))
