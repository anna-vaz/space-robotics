# Project 4 – Spacecraft with flexible solar panels (MATLAB)

A satellite bus carrying two long, light solar panels slews about one axis. The panels
bend, so a fast slew leaves them vibrating, and the controller has to work without
knowing exactly how stiff they are. This is the PhD topic *"Robust control of in-orbit
robotic systems in the presence of unknown flexibilities and parametric uncertainty"*,
in its simplest form.

Everything in this project is MATLAB (Control System and Robust Control Toolboxes).

## Plan

1. **Model**: rigid hub + cantilever bending modes (hybrid model), checked against an independent finite-element model. *(done)*
2. **Baseline slew with PD control**: rigid-body design, then measure the panel vibration it causes. *(done)*
3. **Input shaping**: ZV / ZVD command shaping and its sensitivity to errors in the panel frequency. *(done)*
4. **Robust control**: uncertain panel stiffness and damping, an H∞ / μ-synthesis controller, worst-case analysis, comparison with steps 2 and 3.
5. **Simulink, animation and write-up.**

## Step 1 – model

| | |
|---|---|
| hub inertia | 150 kg m² |
| panels | 2 × 3 m, 2 kg/m (6 kg each), root 0.5 m from the axis |
| bending stiffness EI | 82.8 N m² (first cantilever mode 0.40 Hz) |
| whole spacecraft, rigid | J = 207 kg m² (panels are 27.5 %) |
| modal damping | ζ = 0.5 % |

Each panel deflects as $w(x,t)=\sum_i \phi_i(x)\,\eta_i(t)$ with the clamped–free beam modes
$\phi_i$ (normalised so $\rho\int\phi_i^2dx = 1$). A panel element at distance $r+x$ from the
axis moves at $(r+x)\dot\theta+\dot w$, and Lagrange's equations give

$$
J\ddot\theta+\sum_{k=1,2}\delta^T\ddot\eta_k=u,\qquad
\ddot\eta_k+2\zeta\Omega\dot\eta_k+\Omega^2\eta_k+\delta\,\ddot\theta=0,
\qquad \delta_i=\rho\int_0^L (r+x)\,\phi_i(x)\,dx .
$$

**Checks** (`step1_model.m`):

- Cantilever frequencies from $1+\cos\beta L\cosh\beta L=0$ agree with a 40-element
  finite-element beam to 1e-8 – 1e-6.
- The finite-element model of the *free* spacecraft (different coordinates: inertial
  node displacements, no mode shapes) and the hybrid model give the same free
  frequencies; the error of the hybrid model shrinks as modes are added
  (first coupled mode: 9e-5 with 1 mode, 6e-9 with 5).
- One-mode hand formula $\omega=\Omega_1/\sqrt{1-2\delta_1^2/J}$ gives 0.4635 Hz (FE: 0.4634 Hz).
- $\sum_i\delta_i^2\to\rho\int(r+x)^2dx$ (the panel's inertia); the first mode alone carries 92.7 %.
- Torque → hub angle: after removing the 6 states that the torque cannot move,
  the zeros sit at the cantilever frequencies and the poles just above them
  (0.400/0.463 Hz, 2.507/2.531 Hz, 7.019/7.033 Hz) – the pole/zero alternation of a
  collocated actuator and sensor.

![model](figures/step1_model.png)

Run: `matlab -batch step1_model` in this folder.

## Step 2 – a slew designed for a rigid spacecraft

PD on the hub angle, designed as if the spacecraft were rigid
($K_p=J\omega_n^2$, $K_d=2\zeta J\omega_n$, $f_n=0.05$ Hz, $\zeta=0.7$), plus the rigid feedforward
$u_{ff}=J\ddot\theta_r$. Slew of 90° with |u| ≤ 1 N m (`step2_pd_slew.m`).

| command | slew ends | residual tip vibration | pointing error after slew | settled (0.001° and 1 mm) |
|---|---|---|---|---|
| step, PD only (saturates) | 78 s (38° overshoot) | 1.9 mm | – | 107 s |
| bang-bang + feedforward | 40.3 s | 3.0 mm | 52 arcsec | 65 s |
| quintic + feedforward | 48.4 s | 0.2 mm | 7 arcsec | 57 s |

A rigid model says the two feedforward slews are finished at 40.3 and 48.4 s. On the
flexible spacecraft the panels keep ringing for another 9–25 s, because their own
damping is tiny (ζ = 0.5 %, e-folding time 69 s).

- **Collocated PD is always stable** (actuator and sensor both on the hub; checked for
  bandwidths 0.001–3 Hz), but it damps the panels only weakly: 3.2 % at the design
  bandwidth, at most 14 % at 0.3 Hz. With very high gain the hub is pinned and the
  panel rings at its cantilever frequency with only its structural damping.
- **Residual vibration of a bang-bang slew** (bottom right). Each torque switch starts a
  decaying oscillation; adding the three gives
  $|q| = \frac{F}{\omega^2}\,|1-e^{\lambda T/2}|^2$, nearly zero when $T$ is a whole number of
  *two* vibration periods (every 4.33 s). The hand formula matches the simulation to
  0.11 mm (peak 3.3 mm). This is the idea behind input shaping (step 3).
- **Smooth profiles help**: the quintic (zero acceleration at both ends) leaves ~10× less
  vibration for the same peak torque, at the cost of 8 s longer slews.

![step 2](figures/step2_pd_slew.png)

Run: `matlab -batch step2_pd_slew` (about 30 s).

## Step 3 – input shaping

Convolve the bang-bang command with a few impulses spaced half a vibration period apart:
the vibration started by one impulse is cancelled by the next (Singer & Seering 1990).
Shapers are tuned to the closed-loop panel mode (0.4623 Hz, ζ = 3.2 %) of step 2
(`input_shaper.m`, `step3_input_shaping.m`).

| shaper | impulses | slew ends | residual tip | settled |
|---|---|---|---|---|
| none | 1 | 40.32 s | 2.95 mm | 65.4 s |
| ZV | 2, over ½ period | 41.40 s | 0.018 mm | 52.3 s |
| ZVD | 3, over 1 period | 42.48 s | 0.015 mm | 52.8 s |
| EI (5 %) | 3, over 1 period | 42.48 s | 0.137 mm | 53.5 s |

At the nominal panel stiffness every shaper removes 95–99.5 % of the vibration for
1–2 s of extra slew time. The remaining ~10 s to settle is the slow (0.05 Hz) PD loop
correcting the hub error the panels caused during the slew, not vibration.

**But the shaper must know the panel frequency.** Sweeping the true stiffness from 0.5× to
1.5× the design value (bottom right, log scale), the range where the residual stays below
0.5 mm is:

| shaper | stiffness range | panel frequency error tolerated |
|---|---|---|
| none | nowhere | – |
| ZV | 0.85 – 1.44 × | −8 % … +20 % |
| ZVD | 0.65 – 1.50 × | −20 % … +23 % |
| EI | 0.55 – 1.50 × | −26 % … +23 % |

The dotted curves are the hand formula: shaped residual = unshaped residual ×
$|\sum_i A_i e^{\lambda(t_n-t_i)}|$ with the *true* panel pole λ. It agrees with the simulation
(median difference 0.014 mm; worst 6 %, at 0.5× stiffness); near the design point the simulated
floor of ~0.02 mm is the 2.5 Hz mode, which the shapers do not target.

Input shaping is open loop: it does nothing against disturbances or unexpected changes and
does not add damping. Step 4 attacks the same uncertainty with feedback.

![step 3](figures/step3_input_shaping.png)

Run: `matlab -batch step3_input_shaping` (about 30 s; needs `results_step2.mat` from step 2).
