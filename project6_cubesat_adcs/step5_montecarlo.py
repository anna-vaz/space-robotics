"""Project 6 step 5a: Monte Carlo pointing budget (nadir mode, full closed loop).

Each run disperses the TRUTH (the flight software keeps the nominal values):
  inertia        principal moments x U(0.9, 1.1), products of inertia +- 0.5e-3 kg m^2
  CoM offset     N(0, 1 cm) per axis around the geometric centre
  residual dipole random direction, |m| ~ U(0, 20) mA m^2  (nominal 5.5)
  gyro bias      N(0, 0.3 deg/s) per axis
  initial state  uniform random attitude, |omega| ~ U(0, 0.5) deg/s, random direction
  epoch          start time uniform over the first day (orbit position, Sun, field)
  noise          independent seeds
Then 1 orbit of nadir pointing from that state (acquisition + tracking).
Results stream to results_step5_mc.jsonl (one line per run) so a crash keeps the runs done.
Run:  ../.venv/Scripts/python.exe step5_montecarlo.py [n_runs]      (48 runs ~ 8 min on 12 cores)
"""
import json
import os
import sys
import time
from multiprocessing import Pool
import numpy as np

import attitude as at
import closed_loop as cl
import environment as en
import orbit as ob
import params as pr

OUT = "results_step5_mc.jsonl"
ENV_T = 86400 + 1.2 * ob.PERIOD
_env = None


def get_env():
    global _env
    if _env is None:
        env = en.OrbitEnvironment(ENV_T)
        _env = (env, cl.onboard_field(env))
    return _env


def draw(seed):
    rng = np.random.default_rng(1000 + seed)
    Jp = np.diag(np.diag(pr.J) * rng.uniform(0.9, 1.1, 3))
    off = rng.uniform(-0.5e-3, 0.5e-3, 3)
    J = Jp + np.array([[0, off[0], off[1]], [off[0], 0, off[2]], [off[1], off[2], 0]]) + (pr.J - np.diag(np.diag(pr.J)))
    r_cm = 0.01 * rng.standard_normal(3)
    m_dir = rng.standard_normal(3); m_dir /= np.linalg.norm(m_dir)
    m_res = rng.uniform(0, 0.020) * m_dir
    b0 = np.radians(0.3) * rng.standard_normal(3)
    q = rng.standard_normal(4); q /= np.linalg.norm(q)
    w_dir = rng.standard_normal(3); w_dir /= np.linalg.norm(w_dir)
    w0 = np.radians(rng.uniform(0, 0.5)) * w_dir
    t0 = float(rng.uniform(0, 86400))
    return dict(J=J, r_cm=r_cm, m_res=m_res, b0=b0, beta0=q if q[0] >= 0 else -q, w0=w0, t0=t0)


def acquisition_time(pe, thr=1.0, hold=300):
    """First time [s] after which the error stays below thr for at least `hold` s."""
    below = pe < thr
    run_len = 0
    for k, b in enumerate(below):
        run_len = run_len + 1 if b else 0
        if run_len >= hold:
            return float(k - hold + 1)
    return float("inf")


def run(seed):
    env, B8 = get_env()
    d = draw(seed)
    truth = dict(J=d["J"], m_res=d["m_res"], centres=pr.FACE_NORMALS * pr.DIMS / 2 - d["r_cm"])
    tw = time.time()
    r = cl.simulate(env, B8, round(d["t0"]), ob.PERIOD, d["beta0"], d["w0"], truth=truth,
                    b0=d["b0"], seed=seed)
    pe = np.degrees(r["point"])
    res = dict(seed=seed, wall=time.time() - tw, acq=acquisition_time(pe),
               pe=np.round(pe, 4).tolist(), know=np.round(np.degrees(r["know"]), 4).tolist(),
               sun=r["sun"].astype(int).tolist(),
               hw_max=float(np.abs(r["hw"]).max()), hw_end=float(np.linalg.norm(r["hw"][-1])),
               m_max=float(np.abs(r["m"]).max()), u_max=float(np.abs(r["u"]).max()),
               Hchk=float(r["Hchk"].max()), m_res=float(np.linalg.norm(d["m_res"])),
               r_cm=float(np.linalg.norm(d["r_cm"])), w0=float(np.degrees(np.linalg.norm(d["w0"]))))
    return res


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 48
    done = set()
    if os.path.exists(OUT):
        with open(OUT) as fh:
            done = {json.loads(line)["seed"] for line in fh if line.strip()}
    todo = [s for s in range(n) if s not in done]
    print(f"{len(done)} runs already in {OUT}; running {len(todo)}", flush=True)
    t0 = time.time()
    with Pool(12) as pool, open(OUT, "a") as fh:
        for res in pool.imap_unordered(run, todo):
            fh.write(json.dumps(res) + "\n"); fh.flush()
            print(f"  seed {res['seed']:3d}: acq {res['acq']:5.0f} s, hw_max {res['hw_max']*1e3:.2f} mN m s  "
                  f"[{res['wall']:.0f} s]", flush=True)
    print(f"done in {time.time()-t0:.0f} s")
