"""Step 4b, campaigns C and D: missions flown with the Kalman filter retuned.

  python step4b_retuned.py      -> campaign C: the 200 x3-error missions of campaign B
  python step4b_retuned.py 1    -> campaign D: the 500 x1-error missions of campaign A


Diagnosis of campaign B: every failure came from the navigation filter being
over-confident. Its process noise (5e-5 m/s^2) was tuned for the x1 errors, so with
x3 thruster/mass errors it trusted its model too much and its estimate drifted.
Here the same 200 missions are flown with kf_accel_sigma = 2e-4 m/s^2.
"""
import json
import os
import sys
import time
from multiprocessing import Pool

from step4b_montecarlo import HERE, N_A, N_B, job, to_builtin

KF_SIGMA_RETUNED = 2e-4

if __name__ == "__main__":
    t0 = time.time()
    scale = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    if scale == 3.0:
        jobs, name = [(1000 + s, 3.0, KF_SIGMA_RETUNED) for s in range(N_B)], "results_step4b_retuned.json"
    else:
        jobs, name = [(s, scale, KF_SIGMA_RETUNED) for s in range(N_A)], f"results_step4b_retuned_x{scale:g}.json"
    out = []
    with Pool(max(1, os.cpu_count() - 1)) as pool:
        for k, res in enumerate(pool.imap_unordered(job, jobs, chunksize=2), 1):
            res["kf_sigma"] = KF_SIGMA_RETUNED
            out.append(res)
            if k % 50 == 0:
                print(f"{k}/{len(jobs)} runs done, {time.time() - t0:.0f} s", flush=True)
    with open(os.path.join(HERE, name), "w") as f:
        json.dump(dict(runs=out), f, default=to_builtin)
    print(f"done: {len(out)} runs in {time.time() - t0:.0f} s")
