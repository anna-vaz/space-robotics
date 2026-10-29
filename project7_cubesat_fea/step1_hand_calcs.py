"""Step 1 -- hand calculations for the 3U CubeSat structure (no FEA yet).

What it does
1. Mass budget: items with mass and position -> total, centre of mass vs the
   CDS box, inertia vs the Project 6 ADCS value.
2. Quasi-static 10 g: rail axial stress, rail buckling, root bending of the
   bus as a cantilever, rail-end bearing. Margins of safety with GEVS factors.
3. Natural frequencies: the bus as a Timoshenko beam (bending + axial), and
   each PCB as a plate on 4 corner supports (Rayleigh-Ritz).
4. Random vibration: GEVS qualification spectrum (grms check) and Miles'
   equation for every mode found in 3.

All numbers come from specs.py; the solvers in hand_calcs.py are checked
against textbook values at the top of the run.

Run:  ../.venv/Scripts/python.exe step1_hand_calcs.py
"""
import json
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import specs as S
from hand_calcs import (angle_section, box_bending_I, timoshenko_beam_modes,
                        plate_modes_point_supported, plate_lambda, gevs_asd, grms,
                        miles_grms)

OUT = os.path.join(os.path.dirname(__file__), "results")
FIG = os.path.join(os.path.dirname(__file__), "figures")
os.makedirs(OUT, exist_ok=True)
os.makedirs(FIG, exist_ok=True)
AL7, AL6, FR4 = (S.MATERIALS[k] for k in ("Al7075-T6", "Al6061-T6", "FR4"))
HZ = S.CUBE_Z / 2
summary = {}


def ms(allow, stress, fs):
    """Margin of safety MS = allowable / (FS * applied) - 1  (must be >= 0)."""
    return allow / (fs * stress) - 1


# ==========================================================================
# 0. Solver checks against textbook values
# ==========================================================================
print("== 0. solver checks")
lam_free = plate_lambda(plate_modes_point_supported(1, 1, 1, 1, 0.3, 1.0, [])[0],
                        1, 1 / (12 * 0.91), 1.0)
corners = [(sx * .5, sy * .5) for sx in (-1, 1) for sy in (-1, 1)]
lam_corner = plate_lambda(plate_modes_point_supported(1, 1, 1, 1, 0.3, 1.0, corners)[0],
                          1, 1 / (12 * 0.91), 1.0)
print(f"  square plate, free:            lambda1 = {lam_free:.3f}   (Leissa 13.468)")
print(f"  square plate, corner supports: lambda1 = {lam_corner:.3f}   (Leissa 7.12)")
# Step 0 SOLIDWORKS benchmark bar (20 x 20 x 200 mm 6061, fixed one end)
a, Lb = 0.02, 0.2
kap = 10 * (1 + AL6["nu"]) / (12 + 11 * AL6["nu"])   # Cowper, rectangle
fb = timoshenko_beam_modes(Lb, AL6["E"] * a**4 / 12, AL6["rho"] * a * a,
                           kGA=kap * AL6["G"] * a * a, rhoI=AL6["rho"] * a**4 / 12,
                           n_el=80, n_modes=2)
print(f"  step-0 bar, Timoshenko beam:   {fb[0]:.1f} / {fb[1]:.0f} Hz   "
      f"(SOLIDWORKS 2 mm mesh: 407.3 / 2441 Hz)")
print(f"  GEVS qual / acceptance:        {grms(S.GEVS_QUAL):.2f} / {grms(S.GEVS_ACC):.2f} grms"
      f"   (GEVS 14.1 / 10.0)")
summary["checks"] = dict(lam_free=lam_free, lam_corner=lam_corner, bar_timo_hz=fb.tolist(),
                         bar_sw_hz=[407.32, 2440.67], grms_qual=grms(S.GEVS_QUAL),
                         grms_acc=grms(S.GEVS_ACC))

# ==========================================================================
# 1. Mass budget
# ==========================================================================
sec = box_bending_I(S.CUBE_X, S.RAIL_W, S.RAIL_T, S.WALL_T)
rail = sec["rail"]
rc = S.CUBE_X / 2 - rail["cx"]                       # rail centroid offset
bw = S.CUBE_X - 2 * S.RAIL_W
pcb_area = S.PCB_A * S.PCB_B
# (name, group, mass kg, centre (x, y, z), box size (dx, dy, dz))  -- masses of
# bought-in items are ASSUMPTIONS (typical COTS values), structure is computed.
items = [
    ("rails (4x 7075 angle)", "structure", 4 * rail["A"] * S.CUBE_Z * AL7["rho"], (0, 0, 0),
     (2 * rc, 2 * rc, S.CUBE_Z)),
    ("side walls (4x 1 mm 6061)", "structure", S.WALL_T * bw * S.CUBE_Z * AL6["rho"] * 4, (0, 0, 0),
     (S.CUBE_X, S.CUBE_Y, S.CUBE_Z)),
    ("end plates (2x 2 mm 6061)", "structure", 2 * S.END_PLATE_T * S.CUBE_X**2 * AL6["rho"], (0, 0, 0),
     (S.CUBE_X, S.CUBE_Y, S.CUBE_Z)),
    ("solar panels (4x body-mounted)", "power", 4 * 0.12, (0, 0, 0), (S.CUBE_X, S.CUBE_Y, 0.30)),
    ("reaction wheels + bracket", "ADCS", 3 * 0.13 + 0.06, (0, 0, -0.120), (0.08, 0.08, 0.06)),
    ("magnetorquers (2 rods + coil)", "ADCS", 0.10, (0, 0, -0.080), (0.08, 0.08, 0.02)),
    ("battery board", "power", 0.25, (0, 0, -0.065), (0.09, 0.096, 0.02)),
    ("EPS board", "power", 0.09, (0, 0, -0.045), (0.09, 0.096, 0.01)),
    ("OBC board", "avionics", 0.07, (0, 0, -0.020), (0.09, 0.096, 0.01)),
    ("ADCS sensor board", "ADCS", 0.06, (0, 0, 0.000), (0.09, 0.096, 0.01)),
    ("UHF radio board", "avionics", 0.08, (0, 0, 0.020), (0.09, 0.096, 0.01)),
    ("payload electronics board", "payload", 0.10, (0, 0, 0.040), (0.09, 0.096, 0.01)),
    ("payload camera + optics", "payload", 1.00, (0, 0, 0.105), (0.085, 0.085, 0.12)),
    ("antenna deployer", "comms", 0.08, (0, 0, 0.162), (0.098, 0.098, 0.008)),
    ("stack rods, spacers, harness, fasteners", "structure", 0.20, (0, 0, -0.01), (0.08, 0.08, 0.30)),
]
# hollow items (walls, panels, rails, plates) -> thin-shell inertia; others solid box
def item_inertia(name, m, d):
    dx, dy, dz = d
    if name.startswith(("side walls", "solar panels")):      # square tube of 4 plates
        s = dx
        Ixx = Iyy = m * s**2 / 6 + m * dz**2 / 12
        Izz = m * s**2 / 3
        return np.array([Ixx, Iyy, Izz])
    if name.startswith("rails"):                             # 4 lines at (+-r, +-r)
        r = dx / 2
        return np.array([m * (r**2 + dz**2 / 12), m * (r**2 + dz**2 / 12), m * 2 * r**2])
    if name.startswith("end plates"):                        # 2 plates at z = +-L/2
        return np.array([m * (dy**2 / 12 + (dz / 2)**2), m * (dx**2 / 12 + (dz / 2)**2),
                         m * (dx**2 + dy**2) / 12])
    return m / 12 * np.array([dy**2 + dz**2, dx**2 + dz**2, dx**2 + dy**2])

M_items = sum(it[2] for it in items)
r_cm = sum(it[2] * np.array(it[3]) for it in items) / M_items
J = np.zeros(3)
for name, _, m, c, d in items:
    rr = np.array(c) - r_cm
    J += item_inertia(name, m, d) + m * np.array([rr[1]**2 + rr[2]**2, rr[0]**2 + rr[2]**2,
                                                   rr[0]**2 + rr[1]**2])
print("\n== 1. mass budget")
for name, grp, m, c, _ in items:
    print(f"  {name:42s} {m*1e3:7.1f} g   z = {c[2]*1e3:+6.0f} mm")
print(f"  {'TOTAL (modelled)':42s} {M_items*1e3:7.1f} g   -> margin to 4.0 kg design: "
      f"{(S.M_DESIGN-M_items)*1e3:.0f} g, to CDS 6.00 kg: {(S.M_MAX_3U-M_items)*1e3:.0f} g")
m_struct = sum(it[2] for it in items if it[1] == "structure")
print(f"  structure fraction: {m_struct*1e3:.0f} g = {100*m_struct/M_items:.0f} % of total")
print(f"  centre of mass z = {r_cm[2]*1e3:+.1f} mm  (CDS limit +-{S.CG_LIMIT[2]*1e3:.0f} mm) -> "
      f"{'OK' if abs(r_cm[2]) <= S.CG_LIMIT[2] else 'FAIL'}")
print(f"  inertia about CoM: {J.round(4)} kg m^2   (Project 6 used {S.J_P6})")
summary["mass"] = dict(items=[(n, g, m) for n, g, m, _, _ in items], total=M_items,
                       structure=m_struct, cm=r_cm.tolist(), J=J.tolist())

# ==========================================================================
# 2. Quasi-static 10 g limit load
# ==========================================================================
print(f"\n== 2. quasi-static {S.QS_LIMIT_G:.0f} g limit load (each axis separately)")
print(f"  rail angle {S.RAIL_W*1e3:.1f} x {S.RAIL_T*1e3:.1f} mm: A = {rail['A']*1e6:.2f} mm^2, "
      f"I_min = {rail['I_min']*1e12:.0f} mm^4")
print(f"  section: I_rails = {sec['I_rails']*1e12:.0f} mm^4, I_walls = {sec['I_walls']*1e12:.0f} mm^4")
qs = {}
for mass in (S.M_DESIGN, S.M_MAX_3U):
    W = mass * S.QS_LIMIT_G * S.G0                 # N, limit load
    # (a) axial: whole stack reacted by the 4 rails at the -Z end (walls ignored)
    F_rail = W / 4
    s_ax = F_rail / rail["A"]
    # (b) rail buckling: one rail, pinned-pinned over the full length (worst case,
    #     no intermediate frames), Euler with the weak principal axis
    P_cr = np.pi**2 * AL7["E"] * rail["I_min"] / S.CUBE_Z**2
    # (c) lateral: bus as a cantilever from the -Z end, uniform load, rails only
    M_root = W * S.CUBE_Z / 2
    s_b = M_root * (S.CUBE_X / 2) / sec["I_rails"]
    # (d) bearing on the rail-end contact pad (CDS 2.2.8: >= 6.5 x 6.5 mm)
    s_br = F_rail / S.RAIL_END_MIN**2
    r = dict(W=W, F_rail=F_rail, s_axial=s_ax, P_cr=P_cr, s_bend=s_b, s_bearing=s_br,
             MS_axial_y=ms(AL7["Sy"], s_ax, S.FS["static"]["yield"]),
             MS_buckling=P_cr / (1.4 * F_rail) - 1,
             MS_bend_y=ms(AL7["Sy"], s_b, S.FS["static"]["yield"]),
             MS_bend_u=ms(AL7["Su"], s_b, S.FS["static"]["ultimate"]),
             MS_bend_y_noTest=ms(AL7["Sy"], s_b, S.FS["analysis_only"]["yield"]))
    qs[mass] = r
    print(f"  m = {mass:.1f} kg: W = {W:.0f} N | rail axial {s_ax/1e6:.1f} MPa (MS {r['MS_axial_y']:.0f})"
          f" | buckling P_cr {P_cr:.0f} N vs {F_rail:.0f} N (MS {r['MS_buckling']:.1f})"
          f" | root bending {s_b/1e6:.1f} MPa (MS_y {r['MS_bend_y']:.1f}, MS_u {r['MS_bend_u']:.1f},"
          f" no-test MS_y {r['MS_bend_y_noTest']:.1f}) | end bearing {s_br/1e6:.1f} MPa")
print(f"  NOTE: the hollow rail end is only {rail['A']*1e6:.0f} mm^2 < 6.5 x 6.5 = "
      f"{S.RAIL_END_MIN**2*1e6:.0f} mm^2 required by CDS 2.2.8 -> rails need solid end feet.")
P_cr_1U = np.pi**2 * AL7["E"] * rail["I_min"] / (S.CUBE_Z / 3)**2
print(f"  NOTE: buckling over the full length is the weakest check; frames every 1U cut the"
      f" free length to 1/3 -> P_cr = {P_cr_1U:.0f} N (x9).")
summary["P_cr_full"] = qs[S.M_DESIGN]["P_cr"]; summary["P_cr_1U"] = P_cr_1U
summary["quasi_static"] = {str(k): v for k, v in qs.items()}

# ==========================================================================
# 3. Natural frequencies
# ==========================================================================
print("\n== 3. natural frequencies")
M_bus = S.M_DESIGN
mpl = M_bus / S.CUBE_Z                               # smeared, kg/m
rhoI = mpl * S.CUBE_X**2 / 12                        # rotary inertia of smeared mass
EI_full = AL7["E"] * sec["I_rails"] + AL6["E"] * sec["I_walls"]
EI_rails = AL7["E"] * sec["I_rails"]
kGA = AL6["G"] * sec["A_shear"]
EA = AL7["E"] * sec["A_rails"] + AL6["E"] * sec["A_walls"]
f_eb = timoshenko_beam_modes(S.CUBE_Z, EI_full, mpl, rhoI=rhoI, n_modes=2)
f_tim = timoshenko_beam_modes(S.CUBE_Z, EI_full, mpl, kGA=kGA, rhoI=rhoI, n_modes=2)
f_rails = timoshenko_beam_modes(S.CUBE_Z, EI_rails, mpl, rhoI=rhoI, n_modes=2)
f_ax = np.sqrt(EA / mpl) / (4 * S.CUBE_Z)
print(f"  bus as cantilever from -Z (test-fixture case), {M_bus:.1f} kg smeared:")
print(f"    bending, walls + rails, Euler-Bernoulli : {f_eb[0]:6.0f} Hz")
print(f"    bending, walls + rails, Timoshenko      : {f_tim[0]:6.0f} Hz  (shear in 1 mm walls)")
print(f"    bending, rails only (no walls), E-B     : {f_rails[0]:6.0f} Hz  (lower bound on bending)")
print(f"    axial                                   : {f_ax:6.0f} Hz")
print("  (inside the deployer the rails are guided along their length, so global bending is")
print("   suppressed and the internal boards set the first mode -> see below)")

D = FR4["E"] * S.PCB_T**3 / (12 * (1 - FR4["nu"]**2))
xs, ys = S.PCB_A / 2 - S.PCB_HOLE_INSET, S.PCB_B / 2 - S.PCB_HOLE_INSET
holes = [(sx * xs, sy * ys) for sx in (-1, 1) for sy in (-1, 1)]
bare = FR4["rho"] * S.PCB_T * pcb_area

def pcb_f1(m_board, E=FR4["E"]):
    Dm = E * S.PCB_T**3 / (12 * (1 - FR4["nu"]**2))
    return plate_modes_point_supported(S.PCB_A, S.PCB_B, S.PCB_T, E, FR4["nu"],
                                       m_board / pcb_area, holes, n_modes=1)[0]

boards = [(n, m) for n, g, m, _, _ in items if n.endswith("board")]
print(f"  PCBs, {S.PCB_A*1e3:.2f} x {S.PCB_B*1e3:.2f} x {S.PCB_T*1e3:.1f} mm FR-4, 4 corner supports,"
      f" bare board {bare*1e3:.0f} g, mass smeared:")
pcb = {}
for n, m in boards:
    f1 = pcb_f1(m)
    lo, hi = pcb_f1(m, 0.7 * FR4["E"]), pcb_f1(m, 1.3 * FR4["E"])
    pcb[n] = dict(m=m, f1=f1, f1_Elo=lo, f1_Ehi=hi)
    print(f"    {n:28s} {m*1e3:5.0f} g : f1 = {f1:5.0f} Hz  (E -30 %/+30 %: {lo:.0f}-{hi:.0f})"
          f"  {'OK' if f1 >= S.F1_MIN else '< 100 Hz'}")
m_grid = np.linspace(bare, 0.35, 60)
f_grid = np.array([pcb_f1(m) for m in m_grid])
m_100 = np.interp(-S.F1_MIN, -f_grid, m_grid)
print(f"    max board mass for f1 >= {S.F1_MIN:.0f} Hz with corner supports only: {m_100*1e3:.0f} g")
summary["frequencies"] = dict(bus_EB=f_eb[0], bus_timo=f_tim[0], bus_rails_only=f_rails[0],
                              bus_axial=f_ax, pcb=pcb, pcb_mass_limit_100Hz=m_100)

# ==========================================================================
# 4. Random vibration preview (Miles)
# ==========================================================================
print(f"\n== 4. random vibration, GEVS qualification ({grms(S.GEVS_QUAL):.1f} grms input), "
      f"Miles with Q = {S.Q_ASSUMED:.0f}")
modes = [("bus bending (Timoshenko)", f_tim[0]), ("bus axial", f_ax)] + \
        [(n, d["f1"]) for n, d in pcb.items()]
rv = {}
for n, fn in modes:
    g = miles_grms(fn, S.Q_ASSUMED, gevs_asd(fn, S.GEVS_QUAL)[0])
    z3 = 3 * g * S.G0 / (2 * np.pi * fn)**2
    rv[n] = dict(fn=fn, grms=g, g3sigma=3 * g, disp3sigma_mm=z3 * 1e3)
    print(f"  {n:28s} fn = {fn:5.0f} Hz -> {g:5.1f} grms, 3-sigma {3*g:5.0f} g, "
          f"3-sigma displacement {z3*1e3:.2f} mm")
print(f"  -> 3-sigma random loads are {min(v['g3sigma'] for v in rv.values()):.0f}-"
      f"{max(v['g3sigma'] for v in rv.values()):.0f} g, far above the {S.QS_LIMIT_G:.0f} g quasi-static case:"
      f" random vibration sizes this structure.")
F_ax3 = S.M_DESIGN * rv["bus axial"]["g3sigma"] * S.G0 / 4
fs_r = S.FS["random"]["ultimate"]
print(f"  rail buckling under the 3-sigma axial load ({F_ax3:.0f} N per rail, rails only, x{fs_r} ult.):"
      f" full length MS = {summary['P_cr_full']/(fs_r*F_ax3)-1:+.2f}, frames every 1U MS ="
      f" {P_cr_1U/(fs_r*F_ax3)-1:+.2f}")
summary["random_axial_rail"] = dict(F=F_ax3, MS_full=summary['P_cr_full']/(fs_r*F_ax3)-1,
                                    MS_1U=P_cr_1U/(fs_r*F_ax3)-1)
summary["random"] = rv
with open(os.path.join(OUT, "step1_summary.json"), "w") as fh:
    json.dump(summary, fh, indent=1, default=float)

# ==========================================================================
# Figures
# ==========================================================================
# Figure 1: GEVS spectrum + Miles response
f = np.logspace(np.log10(20), np.log10(2000), 400)
fig, ax = plt.subplots(1, 2, figsize=(13, 4.2))
ax[0].loglog(f, gevs_asd(f, S.GEVS_QUAL), "C3", lw=2, label=f"qualification ({grms(S.GEVS_QUAL):.1f} grms)")
ax[0].loglog(f, gevs_asd(f, S.GEVS_ACC), "C0", lw=2, label=f"acceptance ({grms(S.GEVS_ACC):.1f} grms)")
ax[0].set(xlabel="frequency (Hz)", ylabel="ASD (g$^2$/Hz)", title="GEVS random vibration (Table 2.4-3)")
ax[0].grid(True, which="both", alpha=.3); ax[0].legend()
g3 = 3 * miles_grms(f, S.Q_ASSUMED, gevs_asd(f, S.GEVS_QUAL))
ax[1].loglog(f, g3, "k", lw=1.5, label=f"Miles, Q = {S.Q_ASSUMED:.0f}, 3$\\sigma$")
ax[1].axhline(S.QS_LIMIT_G, color="C2", ls="--", label=f"quasi-static limit {S.QS_LIMIT_G:.0f} g")
ax[1].axvline(S.F1_MIN, color="0.5", ls=":", label=f"f1 requirement {S.F1_MIN:.0f} Hz")
for i, (n, v) in enumerate(rv.items()):
    ax[1].plot(v["fn"], v["g3sigma"], "o", ms=7, label=f"{n} ({v['fn']:.0f} Hz)")
ax[1].set(xlabel="natural frequency (Hz)", ylabel="3$\\sigma$ response (g)",
          title="Equivalent load on a single mode (qualification)")
ax[1].set_ylim(5, 250)
ax[1].grid(True, which="both", alpha=.3); ax[1].legend(fontsize=7, loc="upper left", bbox_to_anchor=(1.02, 1.0))
fig.tight_layout(); fig.savefig(os.path.join(FIG, "step1_random_vibration.png"), dpi=150)

# Figure 2: PCB first mode + mass limit
m_show = 0.10
f1, shape = plate_modes_point_supported(S.PCB_A, S.PCB_B, S.PCB_T, FR4["E"], FR4["nu"],
                                        m_show / pcb_area, holes, n_modes=1, return_shape=True)
X, Y = np.meshgrid(np.linspace(-S.PCB_A / 2, S.PCB_A / 2, 81), np.linspace(-S.PCB_B / 2, S.PCB_B / 2, 81))
Wm = shape(X, Y); Wm /= np.abs(Wm).max()
fig, ax = plt.subplots(1, 2, figsize=(11, 4.4))
cs = ax[0].contourf(X * 1e3, Y * 1e3, Wm, 21, cmap="viridis")
ax[0].plot([h[0] * 1e3 for h in holes], [h[1] * 1e3 for h in holes], "wo", mec="k", ms=8, label="supports")
ax[0].set_aspect("equal"); fig.colorbar(cs, ax=ax[0], label="normalised deflection")
ax[0].set(xlabel="x (mm)", ylabel="y (mm)", title=f"PCB first mode, {m_show*1e3:.0f} g board: {f1[0]:.0f} Hz")
ax[0].legend(loc="upper center", fontsize=8)
ax[1].plot(m_grid * 1e3, f_grid, "k", lw=2, label="corner supports, E = 20 GPa")
ax[1].fill_between(m_grid * 1e3, [pcb_f1(m, 0.7 * FR4["E"]) for m in m_grid],
                   [pcb_f1(m, 1.3 * FR4["E"]) for m in m_grid], color="0.85", label="E $\\pm$30 %")
ax[1].axhline(S.F1_MIN, color="C3", ls="--", label=f"requirement {S.F1_MIN:.0f} Hz")
for n, d in pcb.items():
    ax[1].plot(d["m"] * 1e3, d["f1"], "o", ms=7, label=n)
ax[1].set(xlabel="board mass incl. components (g)", ylabel="first natural frequency (Hz)",
          title="PCB first mode vs board mass (1.6 mm FR-4)", ylim=(0, None))
ax[1].grid(alpha=.3); ax[1].legend(fontsize=7)
fig.tight_layout(); fig.savefig(os.path.join(FIG, "step1_pcb_modes.png"), dpi=150)
print("\nfigures: figures/step1_random_vibration.png, figures/step1_pcb_modes.png")
