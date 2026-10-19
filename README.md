# Space Robotics Simulations

Small, self-contained Python simulations of space robotic systems.

## Projects

1. **`project1_free_floating/`** – planar free-floating space manipulator
   (a satellite base carrying a 2-link robotic arm, no external forces).
   Shows how arm motion disturbs the base attitude and position, and verifies
   conservation of linear and angular momentum.

   ![free-floating robot](project1_free_floating/figures/free_floating.gif)

2. **`project2_rendezvous/`** – autonomous rendezvous and docking in low Earth orbit.
   Clohessy–Wiltshire relative motion (validated against the nonlinear two-body
   problem), LQR translation control, Euler-parameter attitude control with a
   Lyapunov tracking law, a Kalman filter for navigation, and a 1 400-mission
   Monte Carlo robustness study.

   ![docking](project2_rendezvous/figures/docking.gif)

## Setup

```bash
py -3.14 -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```
