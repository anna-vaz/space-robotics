"""Project 3, step 5a: Monte Carlo over the unknown properties of the debris.

Random per mission (uniform):
  target mass        30 - 80 kg
  inertia factor     0.7 - 1.3  x  uniform-plate value   (mass distribution unknown)
  tumble rate        1 - 8 deg/s                          (spans the wheel limit)
  grasp instant      30 - 34 s                            (gripper closing time)
  gripper stiffness  0.5 - 2  x  1000 N/m
The controllers keep their NOMINAL tuning (50 kg target, 214 kg m^2 stack).

Success = gripper holds (peak force < 10 N, peak torque < 10 N m, stretch < 10 mm)
          AND the stack is stopped (|rate| < 0.01 deg/s) and the base is back
          within 1 deg of its original attitude, 900 s after the grasp.
Results are appended to results_step5_mc.jsonl as they finish.
"""
import json
import os
import time
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
N_RUNS = 200
LIMITS = dict(force=10.0, torque=10.0, stretch=0.010, rate_deg=0.01, att_deg=1.0)


def job(seed):
    import capture as cp
    import contact as ct
    import gen_jacobian as gj
    import target as tg
    rng = np.random.default_rng(seed)
    p = dict(mass=rng.uniform(30, 80), inertia_factor=rng.uniform(0.7, 1.3),
             spin=rng.uniform(1, 8), t_grasp=rng.uniform(30, 34), k_grip=1e3 * rng.uniform(0.5, 2))
    r = cp.run(p["spin"], t_detumble=900.0, n_out=901, mass=p["mass"],
               inertia_factor=p["inertia_factor"], t_grasp=p["t_grasp"], k_grip=p["k_grip"])
    X, grip = r["X"], r["grip"]
    FG = [grip.wrench(x[:5], x[5:10], x[10:16]) for x in X]
    stretch = max(np.linalg.norm(ct.target_handle(x[10:16])[0] - gj.end_effector(x[:5])) for x in X)
    L = cp.total_L(X[0])
    _, I_sys, _ = ct.system_inertia(X[-1, :5], X[-1, 10:16])
    out = dict(seed=seed, **p, I_t=tg.I_T, L=L, L_ratio=L / cp.H_MAX, I_sys=I_sys,
               peak_force=max(np.linalg.norm(f[0]) for f in FG), peak_torque=max(abs(f[1]) for f in FG),
               stretch=stretch, final_rate_deg=float(np.degrees(X[-1, 7])),
               final_att_deg=float(np.degrees(X[-1, 2])), att_at_grasp_deg=float(np.degrees(X[0, 2])),
               L_drift=float(np.ptp([cp.total_L(x) for x in X[::50]])), solver_ok=r["ok"])
    out["grasp_ok"] = bool(out["peak_force"] < LIMITS["force"] and out["peak_torque"] < LIMITS["torque"]
                           and out["stretch"] < LIMITS["stretch"])
    out["detumble_ok"] = bool(abs(out["final_rate_deg"]) < LIMITS["rate_deg"]
                              and abs(out["final_att_deg"]) < LIMITS["att_deg"])
    out["pass"] = out["grasp_ok"] and out["detumble_ok"] and out["solver_ok"]
    return {k: (v.item() if isinstance(v, np.generic) else v) for k, v in out.items()}


if __name__ == "__main__":
    t0 = time.time()
    path = os.path.join(HERE, "results_step5_mc.jsonl")
    with Pool(max(1, os.cpu_count() - 1)) as pool, open(path, "w") as f:
        for k, res in enumerate(pool.imap_unordered(job, range(N_RUNS)), 1):
            f.write(json.dumps(res) + "\n"); f.flush()
            if k % 20 == 0:
                print(f"{k}/{N_RUNS} done, {time.time() - t0:.0f} s", flush=True)
    print(f"done in {time.time() - t0:.0f} s")
