"""Project 6 step 1: orbit + space environment + disturbance torques for a 3U CubeSat.

Checks (all printed):
  1. SSO design: inclination, period; J2 nodal drift from the propagated orbit vs
     the analytic rate vs the Sun's mean motion (0.9856 deg/day).
  2. Sun model vs JPL Horizons apparent RA/Dec (4 dates, values fetched 2026-10-27).
  3. Eclipse fraction from the propagated orbit vs the analytic formula.
  4. Magnetic field: our tilted-dipole formula vs ppigrf truncated at degree 1 (frame +
     sign check), full IGRF vs dipole along the orbit, spline-interpolation error.
  5. Disturbance torques: hand bounds / exact formulas.
  6. Attitude propagation over 2 orbits (slow tumble): torque-free H and T conserved;
     with disturbances H_N(t) - H_N(0) = integral of the inertial torque.
Run:  ../.venv/Scripts/python.exe step1_environment.py
"""
import time
from datetime import timedelta
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import ppigrf
from scipy.integrate import solve_ivp

import orbit as ob
import environment as en
import params as pr
import attitude as at

np.set_printoptions(precision=4, suppress=False)
t0_wall = time.time()

# ------------------------------------------------------------------ 1. SSO design
i_sso = ob.sso_inclination(ob.A)
print("== 1. Sun-synchronous orbit")
print(f"  altitude 500 km, a = {ob.A/1e3:.3f} km, period = {ob.PERIOD/60:.2f} min")
print(f"  SSO inclination = {np.degrees(i_sso):.3f} deg")

env = en.OrbitEnvironment(2 * ob.PERIOD, dt=10.0)
days = 5
sol5 = ob.propagate(env.s0, days * 86400)
tt = np.arange(0, days * 86400, 60.0)
X = sol5.sol(tt).T
raan = np.unwrap(ob.raan_of(X[:, :3], X[:, 3:]))
rate_fit = np.degrees(np.polyfit(tt, raan, 1)[0]) * 86400
rate_an = np.degrees(ob.raan_rate(ob.A, i_sso)) * 86400
rate_sun = 360 / (ob.YEAR / 86400)
print(f"  RAAN drift: propagated {rate_fit:.4f} deg/day, analytic {rate_an:.4f}, "
      f"Sun mean motion {rate_sun:.4f}")
print(f"    (propagated vs analytic {100*(rate_fit/rate_an-1):+.2f} %: we start from osculating"
      " = mean elements; J2 short-period terms shift the mean a and i slightly)")
r_n = np.linalg.norm(X[:, :3], axis=1)
print(f"  radius range over 5 days: {(r_n.min()-ob.RE)/1e3:.1f} .. {(r_n.max()-ob.RE)/1e3:.1f} km altitude")

# ------------------------------------------------------------------ 2. Sun vs Horizons
print("\n== 2. Sun direction vs JPL Horizons (apparent RA/Dec, geocentric)")
horizons = [(2461345.5, 216.197373638, -14.360617213, 0.99268413815281),
            (2461437.5, 314.340659116, -17.226287895, 0.98527426974241),
            (2461529.5, 40.873101019, 15.838110358, 1.00815275486797),
            (2461621.5, 133.887556030, 17.350486250, 1.01470132960260)]
for jd, ra, dec, dist in horizons:
    s = en.sun_eci(jd)
    u = s / np.linalg.norm(s)
    uh = np.array([np.cos(np.radians(dec)) * np.cos(np.radians(ra)),
                   np.cos(np.radians(dec)) * np.sin(np.radians(ra)), np.sin(np.radians(dec))])
    ang = np.degrees(np.arccos(np.clip(u @ uh, -1, 1)))
    print(f"  JD {jd}: angle to Horizons {ang*3600:6.1f} arcsec, distance "
          f"{np.linalg.norm(s)/en.AU:.5f} vs {dist:.5f} AU")

# ------------------------------------------------------------------ 3. Eclipse
s_hat0 = env.sun[0] / np.linalg.norm(env.sun[0])
h_hat = np.cross(env.r[0], env.v[0]); h_hat /= np.linalg.norm(h_hat)
beta = np.arcsin(h_hat @ s_hat0)
t1 = np.arange(0, ob.PERIOD, 0.5)
r1 = env.sol.sol(t1).T[:, :3]
s1 = en.sun_eci(env.jd0 + t1 / 86400)
s1 /= np.linalg.norm(s1, axis=1)[:, None]
sh1 = en.in_shadow(r1, s1)
print("\n== 3. Eclipse")
print(f"  beta angle at epoch = {np.degrees(beta):.2f} deg")
print(f"  shadow fraction: propagated {sh1.mean():.4f} ({sh1.mean()*ob.PERIOD/60:.1f} min), "
      f"analytic {en.eclipse_fraction(ob.A, beta):.4f} ({en.eclipse_fraction(ob.A, beta)*ob.PERIOD/60:.1f} min)")

# ------------------------------------------------------------------ 4. Magnetic field
print("\n== 4. Magnetic field")
B_dip = en.dipole_eci(env.r, env.jd, env.field_date)
B_d1 = en.igrf_eci(env.r, env.jd, env.field_date, max_degree=1)
print(f"  own dipole formula vs ppigrf degree 1: max diff {np.abs(B_dip-B_d1).max():.2e} T")
Bn = np.linalg.norm(env.B, axis=1)
print(f"  |B| along orbit (IGRF-14, degree 13): {Bn.min()*1e6:.1f} .. {Bn.max()*1e6:.1f} uT")
g10, g11, h11 = en.dipole_coefficients(env.field_date)
B0 = np.sqrt(g10**2 + g11**2 + h11**2) * 1e-9 * (en.R_IGRF / ob.A)**3
print(f"  dipole hand values at 500 km: equator {B0*1e6:.1f} uT, pole {2*B0*1e6:.1f} uT")
dB = np.linalg.norm(env.B - B_dip, axis=1)
print(f"  |IGRF - dipole|: rms {np.sqrt(np.mean(dB**2))*1e6:.2f} uT, max {dB.max()*1e6:.2f} uT "
      f"(max direction error {np.degrees(np.arccos(np.min(np.sum(env.B*B_dip,1)/Bn/np.linalg.norm(B_dip,axis=1)))):.1f} deg)")
tm = env.t[:-3] + 5.0                                       # mid-points of the 10 s grid
B_direct = en.igrf_eci(env.sol.sol(tm).T[:, :3], env.jd0 + tm / 86400, env.field_date)
print(f"  spline (10 s grid) vs direct IGRF at mid-points: max {np.abs(env._B(tm)-B_direct).max()*1e9:.2e} nT")

# ------------------------------------------------------------------ 5. Torque formulas
print("\n== 5. Disturbance torque checks")
rng = np.random.default_rng(1)
e0 = env.at(1000.0)
rho = en.density(np.linalg.norm(e0["r"]) - ob.RE)
v_rel = e0["v"] - np.cross([0, 0, ob.OMEGA_E], e0["r"])
err_drag = err_srp = 0.0
for _ in range(200):
    q = rng.normal(size=4); q /= np.linalg.norm(q)
    BN = at.ep2dcm(q)
    vb = BN @ v_rel
    vh = vb / np.linalg.norm(vb)
    A_proj = 0.5 * np.sum(pr.FACE_AREAS * np.abs(pr.FACE_NORMALS @ vh))   # silhouette area of a box
    F = -0.5 * rho * pr.CD * (vb @ vb) * A_proj * vh
    err_drag = max(err_drag, np.linalg.norm(en.drag_torque(vb, rho) - np.cross(-pr.R_CM, F)))
    sb = rng.normal(size=3); sb /= np.linalg.norm(sb)
    A_s = 0.5 * np.sum(pr.FACE_AREAS * np.abs(pr.FACE_NORMALS @ sb))
    Fs = -en.P_SUN * pr.CR * A_s * sb
    err_srp = max(err_srp, np.linalg.norm(en.srp_torque(sb, en.AU, False) - np.cross(-pr.R_CM, Fs)))
print("  convex symmetric box: face-by-face torque == (-R_CM) x (total force)  [CoP at the centre]")
print(f"    drag max diff {err_drag:.1e} N m, SRP max diff {err_srp:.1e} N m (200 random attitudes)")
eig = np.linalg.eigvalsh(pr.J)
r_min = np.linalg.norm(env.r, axis=1).min()                  # J2 makes the "circular" orbit dip ~8 km
gg_bound = 1.5 * ob.MU / r_min**3 * (eig[-1] - eig[0])
mag_bound = np.linalg.norm(pr.M_RESIDUAL) * Bn.max()
A_max = np.sqrt(pr.FACE_AREAS[0]**2 + pr.FACE_AREAS[2]**2 + pr.FACE_AREAS[4]**2)   # max silhouette of a box
rho_max = en.density(r_min - ob.RE)
v2_max = np.max(np.sum((env.v - np.cross([0, 0, ob.OMEGA_E], env.r))**2, axis=1))
drag_bound = 0.5 * rho_max * pr.CD * v2_max * A_max * np.linalg.norm(pr.R_CM)
srp_bound = en.P_SUN * pr.CR * A_max * np.linalg.norm(pr.R_CM) * (1 / 0.9926)**2
print(f"  hand upper bounds: gravity gradient 3mu/(2r^3)(Imax-Imin) = {gg_bound:.2e} N m")
print(f"                     residual dipole |m||B|max            = {mag_bound:.2e} N m")
print(f"                     drag  q CD A_max |R_CM| (rho={rho_max:.2e}) = {drag_bound:.2e} N m")
print(f"                     SRP   P CR A_max |R_CM|               = {srp_bound:.2e} N m")

# ------------------------------------------------------------------ 6. Attitude propagation
print("\n== 6. Attitude propagation, 2 orbits, initial tumble")
beta0 = at.prv2ep([1, 2, 3], np.radians(40))
w0 = np.radians([2.0, -1.0, 1.5])                           # [rad/s] slow tumble


def rhs(t, x, with_dist):
    beta, w = x[:4], x[4:7]
    if with_dist:
        BN = at.ep2dcm(beta)
        tq = sum(en.disturbance_torques(BN, env.at(t)).values())
    else:
        BN, tq = None, np.zeros(3)
    bd, wd = at.rigid_body_rhs(beta, w, tq, pr.J, pr.J_INV)
    tqN = BN.T @ tq if with_dist else np.zeros(3)
    return np.concatenate([bd, wd, tqN])


T_end = 2 * ob.PERIOD
t_out = np.arange(0, T_end, 10.0)
x0 = np.concatenate([beta0, w0, np.zeros(3)])
res = {}
for flag in (False, True):
    tw = time.time()
    res[flag] = solve_ivp(rhs, (0, t_out[-1]), x0, args=(flag,), method="DOP853",
                          rtol=1e-12, atol=1e-12, t_eval=t_out)
    print(f"  {'with' if flag else 'without'} disturbances: {res[flag].nfev} evals, {time.time()-tw:.1f} s")


def momentum_energy(Y):
    H, T = [], []
    for x in Y.T:
        BN = at.ep2dcm(x[:4] / np.linalg.norm(x[:4]))
        Hb = pr.J @ x[4:7]
        H.append(BN.T @ Hb)
        T.append(0.5 * x[4:7] @ Hb)
    return np.array(H), np.array(T)


H0, T0 = momentum_energy(res[False].y)
print(f"  torque-free: |H_N - H_N(0)| max {np.abs(H0-H0[0]).max():.1e} N m s "
      f"(|H| = {np.linalg.norm(H0[0]):.2e}), T rel. drift {np.abs(T0/T0[0]-1).max():.1e}, "
      f"|beta|-1 max {np.abs(np.linalg.norm(res[False].y[:4],axis=0)-1).max():.1e}")
H1, _ = momentum_energy(res[True].y)
intq = res[True].y[7:10].T
print(f"  disturbed:   H_N(t)-H_N(0) vs integral of torque: max diff {np.abs(H1-H1[0]-intq).max():.1e} N m s")
print(f"               momentum change over 2 orbits |dH| = {np.linalg.norm(H1[-1]-H1[0])*1e3:.4f} mN m s "
      f"(max along the way {np.linalg.norm(H1-H1[0],axis=1).max()*1e3:.4f}); wheel capacity {pr.RW_HMAX*1e3} mN m s")

# torques along the disturbed trajectory
names = ["gg", "mag", "drag", "srp"]
TQ = {k: [] for k in names}
for tk, x in zip(t_out, res[True].y.T):
    d = en.disturbance_torques(at.ep2dcm(x[:4]), env.at(tk))
    for k in names:
        TQ[k].append(np.linalg.norm(d[k]))
TQ = {k: np.array(v) for k, v in TQ.items()}
bounds = dict(gg=gg_bound, mag=mag_bound, drag=drag_bound, srp=srp_bound)
print("  torque magnitudes along the tumble (mean / max / hand bound) [N m]:")
for k in names:
    print(f"    {k:5s} {TQ[k].mean():.2e} / {TQ[k].max():.2e} / {bounds[k]:.2e}")

# ------------------------------------------------------------------ Figures
fig_dir = "figures"
import os
os.makedirs(fig_dir, exist_ok=True)

# Fig 1: |B| map at 500 km + ground track
lat = np.linspace(-89, 89, 90); lon = np.linspace(-180, 180, 181)
LON, LAT = np.meshgrid(lon, lat)
Br, Bt, Bp = ppigrf.igrf_gc(np.full(LAT.size, ob.A / 1e3), 90 - LAT.ravel(), LON.ravel(),
                            env.field_date.replace(tzinfo=None))
Bmap = np.sqrt(Br**2 + Bt**2 + Bp**2).reshape(LAT.shape) / 1e3
tg = np.arange(0, 3 * ob.PERIOD, 20.0)
rg = sol5.sol(tg).T[:, :3]
th = en.gmst(env.jd0 + tg / 86400)
xe = np.cos(th) * rg[:, 0] + np.sin(th) * rg[:, 1]; ye = -np.sin(th) * rg[:, 0] + np.cos(th) * rg[:, 1]
glat = np.degrees(np.arcsin(rg[:, 2] / np.linalg.norm(rg, axis=1))); glon = np.degrees(np.arctan2(ye, xe))
sg = en.sun_eci(env.jd0 + tg / 86400); sg /= np.linalg.norm(sg, axis=1)[:, None]
shg = en.in_shadow(rg, sg)
fig, ax = plt.subplots(figsize=(10, 5))
cs = ax.contourf(LON, LAT, Bmap, levels=np.arange(16, 56, 2), cmap="viridis", extend="both")
ax.contour(LON, LAT, Bmap, levels=np.arange(16, 56, 4), colors="k", linewidths=0.4)
plt.colorbar(cs, ax=ax, label="|B| at 500 km [uT]")
brk = np.where(np.abs(np.diff(glon)) > 180)[0] + 1
for seg_lat, seg_lon, seg_sh in zip(np.split(glat, brk), np.split(glon, brk), np.split(shg, brk)):
    ax.plot(seg_lon, seg_lat, "w-", lw=1.5)
    ax.plot(np.where(seg_sh, seg_lon, np.nan), np.where(seg_sh, seg_lat, np.nan), "-", color="0.15", lw=2.5)
ax.plot([], [], "w-", label="ground track, sunlit"); ax.plot([], [], "-", color="0.15", lw=2.5, label="in eclipse")
ax.text(-45, -30, "South Atlantic\nAnomaly", color="w", ha="center", fontsize=9)
ax.set(xlabel="longitude [deg]", ylabel="geocentric latitude [deg]",
       title=f"IGRF-14 field strength at 500 km and 3 orbits of the SSO ({env.field_date:%Y-%m-%d})")
ax.legend(loc="lower left", fontsize=8)
fig.tight_layout(); fig.savefig(f"{fig_dir}/step1_field_map.png", dpi=130)

# Fig 2: field along 2 orbits
tmin = env.t / 60
fig, axs = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
for k, c in enumerate("rgb"):
    axs[0].plot(tmin, env.B[:, k] * 1e6, c, label=f"B_{'xyz'[k]} IGRF")
    axs[0].plot(tmin, B_dip[:, k] * 1e6, c + "--", lw=0.9, label=f"B_{'xyz'[k]} dipole")
axs[0].set_ylabel("B in ECI [uT]"); axs[0].set_ylim(-55, 62); axs[0].legend(ncol=6, fontsize=8, loc="upper center")
axs[1].plot(tmin, Bn * 1e6, "k", label="|B| IGRF (degree 13)")
axs[1].plot(tmin, np.linalg.norm(B_dip, axis=1) * 1e6, "k--", label="|B| tilted dipole")
axs[1].set(ylabel="|B| [uT]", xlabel="time [min]"); axs[1].legend(fontsize=8)
for a in axs:
    on = np.where(np.diff(env.shadow.astype(int)) == 1)[0]; off = np.where(np.diff(env.shadow.astype(int)) == -1)[0]
    if env.shadow[0]: on = np.r_[-1, on]
    for a0, a1 in zip(on, off):
        a.axvspan(tmin[a0 + 1], tmin[a1], color="0.85", zorder=0)
    a.grid(alpha=0.3)
axs[0].set_title("Magnetic field along two orbits (grey = eclipse)")
fig.tight_layout(); fig.savefig(f"{fig_dir}/step1_field_orbit.png", dpi=130)

# Fig 3: torques
fig, ax = plt.subplots(figsize=(10, 4.5))
lab = dict(gg="gravity gradient", mag="residual dipole (5.5 mA m^2)", drag="aerodynamic drag", srp="solar pressure")
cols = dict(gg="C0", mag="C3", drag="C2", srp="C1")
for k in names:
    ax.semilogy(t_out / 60, np.where(TQ[k] > 0, TQ[k], np.nan), color=cols[k], lw=1, label=lab[k])
    ax.axhline(bounds[k], color=cols[k], ls="--", lw=0.8)
ax.set(xlabel="time [min]", ylabel="|torque| [N m]", ylim=(1e-11, 1e-6),
       title="Disturbance torques on a slowly tumbling 3U CubeSat at 500 km (dashed = hand bounds;\n SRP is zero in eclipse)")
ax.legend(fontsize=8, loc="lower right"); ax.grid(alpha=0.3, which="both")
fig.tight_layout(); fig.savefig(f"{fig_dir}/step1_torques.png", dpi=130)
print(f"\nfigures written to {fig_dir}/ ; total {time.time()-t0_wall:.0f} s")
