"""Step 4b, campaign C: the x3 stress test again, with the Kalman filter retuned.

Diagnosis of campaign B: every failure came from the navigation filter being
over-confident. Its process noise (5e-5 m/s^2) was tuned for the x1 errors, so with
x3 thruster/mass errors it trusted its model too much and its estimate drifted.
Here the same 200 missions are flown with kf_accel_sigma = 2e-4 m/s^2.
"""
import json
import os
import time
from multiprocessing import Pool

from step4b_montecarlo import HERE, N_B, job, to_builtin

KF_SIGMA_RETUNED = 2e-4

if __name__ == "__main__":
    t0 = time.time()
    jobs = [(1000 + s, 3.0, KF_SIGMA_RETUNED) for s in range(N_B)]
    out = []
    with Pool(max(1, os.cpu_count() - 1)) as pool:
        for k, res in enumerate(pool.imap_unordered(job, jobs, chunksize=2), 1):
            res["kf_sigma"] = KF_SIGMA_RETUNED
            out.append(res)
            if k % 50 == 0:
                print(f"{k}/{len(jobs)} runs done, {time.time() - t0:.0f} s", flush=True)
    with open(os.path.join(HERE, "results_step4b_retuned.json"), "w") as f:
        json.dump(dict(runs=out), f, default=to_builtin)
    print(f"done: {len(out)} runs in {time.time() - t0:.0f} s")
