

# VTOL Digital Twin — Autonomous UAV Simulation

A digital twin of a quad-rotor VTOL UAV with a pusher propeller, built with
**ROS 2 Humble** and **Gazebo Classic**. The drone flies a fully autonomous
mission — takeoff, a 4-waypoint patrol on smooth minimum-snap trajectories,
and landing — using only **estimated state** from simulated noisy sensors.


https://github.com/user-attachments/assets/daa1302b-c66d-4dc0-8d58-979194f06346


## Delivered mission

HOVER → 4-waypoint square patrol (min-snap trajectories) → autonomous landing

The flight controller never sees ground truth: it flies on the output of an
**Error-State Kalman Filter** fusing a noisy IMU (100 Hz) with simulated GPS
(10 Hz, σ = 0.5 m). Typical position estimate error: **0.15–0.30 m** — the
filter beats its own sensor.

## Architecture

| Node | Language | Role |
|---|---|---|
| `hover_pid_node` | C++ | Mission state machine, cascaded attitude/position control, min-snap waypoint trajectories, crash guard |
| `aerodynamics_node` | C++ | Physics: rotor thrust, body drag (linear + quadratic), wing lift/drag polars with stall model, pusher propeller |
| `eskf_node` | C++ / Eigen | 9-state ESKF: IMU prediction with covariance Jacobians, GPS position updates |
| `mpc_node` | Python / CasADi | Model Predictive Control: RK4-discretized point-mass model, 20-step horizon, ipopt, 10 Hz — commands hover/brake/land modes with automatic PD fallback |

## Packages

- `vtol_description` — URDF model (4 lift rotors + rear pusher, force plugins)
- `vtol_simulation` — Gazebo world + launch files
- `vtol_control` — flight controller, aerodynamics, MPC
- `vtol_estimation` — ESKF state estimation

## Run it



Dependencies: `libeigen3-dev`, `pip3 install casadi numpy`

## Test results (from flight telemetry)

- Hover hold: z = 1.98 ± 0.01 m, x/y within ±1 m, yaw within ±4°
- Waypoint square: all corners captured (0.8 m / 0.5 m/s window)
- Forward transition experiments: 14.1 m/s, wings carrying **88% of weight**
  at α = 3.9° (v9); stall recovery verified; cruise trim with climb-rate
  damping held vz within ±0.3 m/s (v10.2)
- ESKF: 0.15–0.30 m position error vs 0.5 m raw GPS

## Roadmap (future work)

- Full hover ↔ cruise transition automation (stability work in progress —
  see commit history for the v9–v10.x telemetry-driven iteration)
- Vision-based precision landing (camera + OpenCV)
- Control surfaces + energy consumption logging
- Docker image, unit tests (gtest), CI pipeline
