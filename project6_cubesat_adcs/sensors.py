"""Sensor models: magnetometer, six coarse sun sensors, MEMS rate gyro.

Magnetometer   B_m = [BN] B_true + noise (sigma 100 nT per axis), 1 Hz.  The flight
               software compares it with an ONBOARD field model (IGRF truncated at
               degree 8 -- a typical flight choice); truth uses degree 13.
Coarse sun     one photodiode per face, current I_i = max(0, n_i . s_B) + noise (1 % of
sensors        full scale); s ~ [I+x - I-x, I+y - I-y, I+z - I-z] (one of each pair is
               dark, so the difference is the cosine). No Earth albedo modelled (it would
               add a few degrees of bias in a real mission). Nothing in eclipse.
Gyro           Farrenkopf model, 10 Hz:  w_m = w + b + eta_v,   b' = eta_u
               (angle random walk sigma_v, rate random walk sigma_u, turn-on bias b0)
"""
import numpy as np

import params as pr

SIGMA_MAG = 100e-9           # [T] per axis
SIGMA_CSS = 0.01             # fraction of full-scale current
CSS_THRESHOLD = 0.3          # sum of currents below this -> no Sun (eclipse)
SIGMA_V = 8.7e-5             # [rad/s^0.5]  ARW 0.3 deg/sqrt(h)
SIGMA_U = 3e-7               # [rad/s^1.5]  rate random walk
GYRO_B0 = np.radians([0.2, -0.3, 0.1])   # turn-on bias [rad/s]
DT_GYRO = 0.1


def magnetometer(B_body, rng):
    return B_body + SIGMA_MAG * rng.standard_normal(3)


def coarse_sun_sensors(s_body, shadow, rng):
    """Returns the measured unit Sun vector in the body, or None (eclipse / too dark)."""
    I = np.zeros(6) if shadow else np.maximum(0.0, pr.FACE_NORMALS @ s_body)
    I = np.maximum(0.0, I + SIGMA_CSS * rng.standard_normal(6))
    if I.sum() < CSS_THRESHOLD:
        return None
    s = np.array([I[0] - I[1], I[2] - I[3], I[4] - I[5]])
    return s / np.linalg.norm(s)


def gyro_series(w_true, rng, dt=DT_GYRO, b0=GYRO_B0):
    """Discrete Farrenkopf model (Crassidis & Junkins eq. 6.89-ish):
       b_{k+1} = b_k + sigma_u sqrt(dt) N
       w_m,k+1 = w_k+1 + (b_k+1 + b_k)/2 + sqrt(sigma_v^2/dt + sigma_u^2 dt/12) N"""
    n = len(w_true)
    b = np.empty((n, 3)); b[0] = b0
    for k in range(1, n):
        b[k] = b[k - 1] + SIGMA_U * np.sqrt(dt) * rng.standard_normal(3)
    noise = np.sqrt(SIGMA_V**2 / dt + SIGMA_U**2 * dt / 12) * rng.standard_normal((n, 3))
    bm = np.vstack([b[:1], 0.5 * (b[1:] + b[:-1])])
    return w_true + bm + noise, b
