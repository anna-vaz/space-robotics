"""Step 4b: Monte Carlo robustness campaign for the docking mission.

Campaign A: 500 missions with the nominal uncertainty model (see montecarlo.UNC)
Campaign B: 200 missions with every real-world error x3 (stress test, find margins)
Plus the nominal (no-error) mission as a reference.
Runs in parallel on all CPU cores but one. Results -> results_step4b.json
"""
import json
import os
import time
from multiprocessing import Pool

import numpy as np

import montecarlo as mc

HERE = os.path.dirname(os.path.abspath(__file__))
N_A, N_B = 500, 200
N_HIST = 25                       # runs whose full history is kept for plots


def to_builtin(o):
    """json.dump fallback for NumPy scalars/arrays (e.g. numpy.bool_)."""
    if isinstance(o, np.generic):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    raise TypeError(type(o))


def job(args):
    seed, scale = args[:2]
    if len(args) > 2:                     # optional Kalman-filter retuning
        mc.UNC["kf_accel_sigma"] = args[2]
    res = mc.run(seed, dispersed=True, scale=scale, keep_history=seed % 1000 < N_HIST)
    res["scale"] = scale
    res["pass"] = bool(mc.passes(res))
    for k in ("hist", "est_err"):
        if res.get(k) is not None:
            res[k] = res[k][:, :].tolist()
    return {k: (float(v) if isinstance(v, (np.floating, float)) else v) for k, v in res.items()}


if __name__ == "__main__":
    t0 = time.time()
    nominal = mc.run(0, dispersed=False, keep_history=True)
    jobs = [(s, 1.0) for s in range(N_A)] + [(1000 + s, 3.0) for s in range(N_B)]
    out = []
    partial = os.path.join(HERE, "results_step4b_partial.jsonl")   # survives a crash
    with Pool(max(1, os.cpu_count() - 1)) as pool, open(partial, "w") as fp:
        for k, res in enumerate(pool.imap_unordered(job, jobs, chunksize=2), 1):
            out.append(res)
            fp.write(json.dumps(res, default=to_builtin) + "\n"); fp.flush()
            if k % 50 == 0:
                print(f"{k}/{len(jobs)} runs done, {time.time() - t0:.0f} s", flush=True)
    nominal_out = {k: (float(v) if isinstance(v, (np.floating, float)) else v)
                   for k, v in nominal.items() if k not in ("hist", "est_err")}
    nominal_out["hist"] = nominal["hist"].tolist()
    with open(os.path.join(HERE, "results_step4b.json"), "w") as f:
        json.dump(dict(nominal=nominal_out, runs=out, unc=mc.UNC, req=mc.m.REQ), f, default=to_builtin)
    print(f"done: {len(out)} runs in {time.time() - t0:.0f} s")
