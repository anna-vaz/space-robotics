# Space Robotics Simulations

Small, self-contained Python simulations of space robotic systems.

## Projects

1. **`project1_free_floating/`** – planar free-floating space manipulator
   (a satellite base carrying a 2-link robotic arm, no external forces).
   Shows how arm motion disturbs the base attitude and position, and verifies
   conservation of linear and angular momentum.

   ![free-floating robot](project1_free_floating/figures/free_floating.gif)

## Setup

```bash
py -3.14 -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```
