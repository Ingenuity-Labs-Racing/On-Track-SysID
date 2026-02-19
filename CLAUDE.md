# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

On-Track-SysID is a ROS (Catkin) Python package for real-time system identification of tire dynamics on scaled autonomous racing vehicles. It collects on-track driving data, trains a neural network to learn errors in a nominal vehicle model, iteratively refines Pacejka tire parameters, and generates steering lookup tables (LUTs) for model predictive control.

Based on: [Learning-Based On-Track System Identification for Scaled Autonomous Racing in Under a Minute](https://arxiv.org/abs/2411.17508)

## Build & Run

```bash
# Build (within a catkin workspace)
catkin build on_track_sys_id

# Launch the ROS node
roslaunch on_track_sys_id sys_id.launch racecar_version:=NUC5 save_LUT_name:=my_lut plot_model:=True
```

No automated tests exist. Validation is done manually via real vehicle or simulation.

## Architecture

The pipeline runs as a single ROS node (`src/on_track_sys_id.py`) that orchestrates a sequential workflow:

1. **Data Collection** — Subscribes to odometry (`nav_msgs/Odometry`) and steering (`ackermann_msgs/AckermannDriveStamped`) topics at 50 Hz for ~30 seconds
2. **Iterative Training Loop** (default 6 iterations):
   - `train_model.py` — Filters data (3rd-order Butterworth), augments by mirroring, trains a `SimpleNN` (4→8→2 feedforward net) to predict lateral velocity and yaw rate errors
   - `generate_predictions.py` / `generate_inputs_errors.py` — Compute model predictions and NN training targets
   - `vehicle_dynamics_stown.py` — Single-track (bicycle) model with Pacejka or linear tire forces
   - `solve_pacejka.py` — Fits Pacejka coefficients via `scipy.optimize.least_squares`
3. **Output** — `simulate_model.py` generates a steering-angle-vs-velocity LUT; `save_model.py` persists identified parameters

### Key files

| File | Role |
|------|------|
| `src/on_track_sys_id.py` | Main ROS node — data collection, lifecycle management |
| `src/helpers/train_model.py` | Core training loop orchestrating filtering, NN training, simulation, and Pacejka fitting |
| `src/helpers/vehicle_dynamics_stown.py` | Single-track vehicle dynamics model |
| `src/helpers/solve_pacejka.py` | Pacejka parameter optimization |
| `src/helpers/simulate_model.py` | ODE-based simulation and LUT generation |
| `src/helpers/SimpleNN.py` | 3-layer feedforward neural network (PyTorch) |
| `params/nn_params.yaml` | NN hyperparameters (iterations, epochs, learning rate) |
| `params/pacejka_params.yaml` | Initial Pacejka coefficients and reference values |
| `models/{VERSION}/` | Per-vehicle identified parameters (mass, inertia, tire coefficients) |

## Dependencies

ROS packages: `rospy`, `nav_msgs`, `ackermann_msgs`, `std_msgs`, `rospkg`. Python: `torch`, `numpy`, `scipy`, `matplotlib`, `pyyaml`, `tqdm`. No `requirements.txt` — assumes ROS and PyTorch are pre-installed in the environment.

## Conventions

- Vehicle configs live in `models/{VERSION}/{VERSION}_pacejka.txt` (YAML format despite `.txt` extension)
- Pacejka tire formula: `F_y = F_z * D * sin(C * arctan(B*α - E*(B*α - arctan(B*α))))`
- State vector: `[x, y, steering_angle, yaw, v_x, v_y, yaw_rate]`
- The package integrates into the ForzaETH Race Stack; generated LUTs must be manually moved to the steering_lookup config directory
