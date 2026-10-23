# Project 5 — Free-floating manipulator in ROS 2 + Gazebo (zero-g)

**Status: in progress (step 1 of 5 done).**

Project 1's planar space robot (100 kg base, two 1 m / 10 kg links) rebuilt as a
ROS 2 Jazzy package and simulated in Gazebo Harmonic with gravity switched off.
The arm is torque-controlled through `ros2_control`; the base has no actuators and
is free to react. Every Gazebo result is checked against the independent Python
models from Projects 1 / 1b.

## Layout
- `zero_g_manipulator/` — the ROS 2 package (URDF/xacro, zero-g world, controllers, launch, test nodes)
- `scripts/` — `setup_workspace.sh` (one-time build in `~/ros2_ws`), `run_stepN.sh` (headless runs)
- `stepN_*.py` — Windows-side analysis against the Python models; `figures/`

## Step 1 — the robot in zero-g Gazebo, checked against Project 1
Open-loop test: 2 s with no torque (rest check), then one sine period of joint
torques (3 / −1.5 N m, 4 s), then 6 s of free drift. The torques Gazebo actually
received are replayed in Project 1's Lagrange model.

| check | result |
|---|---|
| URDF mass/inertia/geometry vs P1 params | identical |
| motion with no torque (zero-g) | exactly 0 |
| out-of-plane motion | exactly 0 |
| base angle after the test, Gazebo vs P1 | −55.067° vs −55.063° (max error 0.022°) |
| joint 2 after 583° of rotation | max error 0.12° |
| system CoM drift (must be 0) | 0.34 mm at 1 ms step, 0.17 mm at 0.5 ms → first-order integration error, not physics |

![step 1](figures/step1_gazebo_vs_python.png)

Run it (Ubuntu / WSL):
```bash
bash /mnt/d/Projects/space-robotics/project5_ros2_gazebo/scripts/setup_workspace.sh   # once
bash /mnt/d/Projects/space-robotics/project5_ros2_gazebo/scripts/run_step1.sh
```
then on Windows: `.venv\Scripts\python project5_ros2_gazebo\step1_compare.py`.
To watch it live: `ros2 launch zero_g_manipulator sim.launch.py` (opens the Gazebo window).
