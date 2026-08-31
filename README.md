
# Hybrid Finite-Time Control for Quadrotor Trajectory Tracking  
### Semester Project – Sensors-Based Controls  
### Master in Computer Vision & Robotics (CVR), 2025  
### University of Burgundy  

---

# Table of Contents

1. [Abstract](#1-abstract)  
2. [Introduction](#2-introduction)  
3. [Quadrotor Modeling](#3-quadrotor-modeling)  
4. [Hybrid Control Architecture](#4-hybrid-control-architecture)  
5. [System Block Diagrams](#5-system-block-diagrams)  
6. [Simulation Setup](#6-simulation-setup)  
7. [Simulation Results](#7-simulation-results)  
8. [Additional Analysis](#8-additional-analysis)  
9. [ROS2 Gazebo Experiment](#9-ros2-gazebo-experiment)  
10. [Hardware Experiment](#10-hardware-experiment)  
11. [How to Run](#11-how-to-run)  
12. [Conclusion](#12-conclusion)

---

# 1. Abstract

This project presents a hybrid finite‑time control scheme for quadrotor trajectory tracking.  
The controller combines:

- Adaptive Integral Sliding Mode (AISM) for altitude  
- Backstepping for planar position  
- Nonsingular Terminal Sliding Mode (NTSM) for attitude  
- A Finite‑Time Disturbance Observer (FDO) for robustness  

The objective is accurate tracking of a 3D circular path under disturbances, validated both in simulation and on the AR.Drone 2.0 platform.

---

# 2. Introduction

Quadrotors are inherently unstable systems sensitive to modeling uncertainties and disturbances.  
To enhance stability and robustness, the following hybrid controller is designed:

- AISM regulates altitude  
- Backstepping governs X–Y motion  
- NTSM stabilizes roll, pitch, and yaw  
- FDO estimates unknown disturbances  

This README provides a concise version of the full report including models, diagrams, and results.

---

# 3. Quadrotor Modeling

Quadrotor dynamics are divided into **translational** and **rotational** components.

---

## 3.1 Translational Dynamics

$$
m \dot{v} = mg + R 
\begin{bmatrix}
0 \\ 0 \\ T
\end{bmatrix}
+ d_t
$$

- \(m\): mass  
- \(R\): rotation matrix  
- \(T\): thrust  
- \(d_t\): translational disturbance  

---

## 3.2 Rotational Dynamics

$$
I\dot{\omega} = \tau - \omega \times (I\omega) + d_r
$$

- \(I\): inertia matrix  
- \(\tau\): control torques  
- \(d_r\): rotational disturbance  

---

## Quadrotor Body Axes

<p align="center">
  <img src="media/frame_diagram.png" width="50%">
</p>
<p align="center"><strong>Figure 3.1:</strong> Quadrotor frames and rotation axes.</p>

---

## AR.Drone 2.0 Platform

<p align="center">
  <img src="media/quadrotor.png" width="50%">
</p>
<p align="center"><strong>Figure 3.2:</strong> AR.Drone 2.0 used for hardware tests.</p>

---

# 4. Hybrid Control Architecture

---

## 4.1 Altitude Control (AISM)

$$
s_z = \dot{e}_z + c_z e_z + k_z \int e_z \, dt
$$

AISM enhances robustness and ensures finite‑time convergence.

---

## 4.2 Position Control (Backstepping)

Backstepping transforms position errors into desired roll and pitch angles for X–Y regulation.

---

## 4.3 Attitude Control (NTSM)

$$
s = e + |e|^{m/n}\,\text{sign}(e)
$$

The NTSM sliding surface ensures fast, nonsingular convergence.

---

## 4.4 Disturbance Observer (FDO)

The FDO estimates external disturbances affecting both translation and rotation.

---

# 5. System Block Diagrams

---

## Hybrid Finite‑Time Control Architecture

<p align="center">
  <img src="media/hybrid_control_block_new.png" width="70%">
</p>
<p align="center"><strong>Figure 5.1:</strong> Overall controller structure.</p>

---

## Backstepping Structure

<p align="center">
  <img src="media/backstepping_virtual_control_new.png" width="70%">
</p>
<p align="center"><strong>Figure 5.2:</strong> Backstepping‑based virtual control mapping.</p>

---

## Simulation Control Flow

<p align="center">
  <img src="media/high_level_block_diagram_new.png" width="70%">
</p>
<p align="center"><strong>Figure 5.3:</strong> High‑level simulation signal flow.</p>

---

## Software Implementation Diagram

<p align="center">
  <img src="media/implementation_architecture.png" width="70%">
</p>
<p align="center"><strong>Figure 5.4:</strong> Software architecture of the controller.</p>

---

## ROS2 Hardware Control Pipeline

<p align="center">
  <img src="media/ros2_px4_pipeline.png" width="70%">
</p>
<p align="center"><strong>Figure 5.5:</strong> ROS2–PX4 command pipeline for hardware implementation.</p>

---

# 6. Simulation Setup

- Simulation time: **20 s**  
- Time step: **0.002 s**  
- Control frequency: **50 Hz**  
- Trajectory: 3D circular path with rising altitude  
- Disturbance: \(0.5\cos(t)\) on selected channels  

---

# 7. Simulation Results

---

## 7.1 3D Trajectory  

<p align="center">
  <img src="media/01_3d_trajectory.png" width="70%">
</p>
<p align="center"><strong>Figure 7.1:</strong> 3D trajectory tracking.</p>

---

## 7.2 XY Trajectory  

<p align="center">
  <img src="media/02_xy_trajectory.png" width="70%">
</p>
<p align="center"><strong>Figure 7.2:</strong> Top‑view circular path tracking.</p>

---

## 7.3 Position Tracking  

<p align="center">
  <img src="media/03_position_tracking.png" width="70%">
</p>
<p align="center"><strong>Figure 7.3:</strong> Desired vs. actual position tracking.</p>

---

## 7.4 Tracking Errors  

<p align="center">
  <img src="media/04_tracking_errors.png" width="70%">
</p>
<p align="center"><strong>Figure 7.4:</strong> Position error evolution.</p>

---

## 7.5 Disturbance Rejection  

<p align="center">
  <img src="media/05_disturbance_rejection.png" width="70%">
</p>
<p align="center"><strong>Figure 7.5:</strong> Disturbance rejection capability.</p>

---

## 7.6 Velocity Profiles  

<p align="center">
  <img src="media/06_velocity.png" width="70%">
</p>
<p align="center"><strong>Figure 7.6:</strong> Translational velocity responses.</p>

---

## 7.7 Summary Plot  

<p align="center">
  <img src="media/07_summary.png" width="70%">
</p>
<p align="center"><strong>Figure 7.7:</strong> Summary of tracking performance.</p>

---

## 7.8 Overview  

<p align="center">
  <img src="media/08_overview.png" width="70%">
</p>
<p align="center"><strong>Figure 7.8:</strong> Combined overview of tracking behavior.</p>

---

## 7.9 Animation (GIF)  

<p align="center">
  <img src="media/09_animation.gif" width="70%">
</p>
<p align="center"><strong>Figure 7.9:</strong> Animation of quadrotor trajectory.</p>

---

# 8. Additional Analysis

---

## Sliding Surfaces  

<p align="center">
  <img src="media/sliding_surfaces.png" width="70%">
</p>
<p align="center"><strong>Figure 8.1:</strong> Convergence of sliding surfaces.</p>

---

## FDO Performance  

<p align="center">
  <img src="media/fdo_performance.png" width="70%">
</p>
<p align="center"><strong>Figure 8.2:</strong> Disturbance observer estimation.</p>

---

## Control Effort  

<p align="center">
  <img src="media/fig_6_5_control_effort.png" width="70%">
</p>
<p align="center"><strong>Figure 8.3:</strong> Altitude control effort.</p>

---

## Control Inputs  

<p align="center">
  <img src="media/control_inputs.png" width="70%">
</p>
<p align="center"><strong>Figure 8.4:</strong> Motor torque inputs.</p>

---

## Disturbance Response  

<p align="center">
  <img src="media/fig_6_4_disturbance_response.png" width="70%">
</p>
<p align="center"><strong>Figure 8.5:</strong> Response under disturbance inputs.</p>

---

## XYZ Error Components  

<p align="center">
  <img src="media/fig_6_3_xyz_tracking_errors.png" width="70%">
</p>
<p align="center"><strong>Figure 8.6:</strong> Component‑wise position errors.</p>

---

## Altitude Tracking  

<p align="center">
  <img src="media/fig_6_2_altitude_tracking.png" width="70%">
</p>
<p align="center"><strong>Figure 8.7:</strong> Altitude tracking with AISM.</p>

---

# 9. ROS2 Gazebo Experiment

## Simulation Flight Trajectory  

<p align="center">
  <a href="https://youtu.be/D-HHGSvnXh8">
    <img src="https://img.youtube.com/vi/D-HHGSvnXh8/hqdefault.jpg" width="60%">
  </a>
</p>
<p align="center"><strong>Figure 9.1:</strong> Click to watch the simulation drone flight video (watch on 2x).</p>

---

# 10. Hardware Experiment

## Real Flight Trajectory

<p align="center">
  <a href="https://youtube.com/shorts/qWHe0PwsiY0">
    <img src="https://img.youtube.com/vi/qWHe0PwsiY0/hqdefault.jpg" width="60%">
  </a>
</p>
<p align="center"><strong>Figure 10.1:</strong> Click to watch the real AR.Drone 2.0 flight video.</p>

---

# 11. How to Run

This repository contains three main implementations:

---

## 1. HFTC Simulation (Python, no hardware required)

**Requirements:**  
- Python 3.x  
- numpy

**Install dependencies:**
```bash
pip install numpy
```

**Run the simulation:**
```bash
python -m hftc_simulation.main
```
or
```bash
python /home/usama/Drone\ Git/Quadrotor-Trajectory-Tracking-and-Robust-Control/hftc_simulation/main.py
```
The simulation will use all controllers and generate results and plots automatically.

---

## 2. AR.Drone 2.0 HFTC Controller (Python, AR.Drone hardware)

**Requirements:**  
- Python 3.x  
- numpy  
- matplotlib  
- (optional) ROS (if using with real AR.Drone 2.0 hardware)  
- AR.Drone 2.0 (for hardware‑in‑the‑loop)

**Install dependencies:**
```bash
pip install numpy matplotlib
```
(Install ROS if required for your hardware setup.)

**Usage:**
1. Connect your computer to the AR.Drone 2.0 WiFi (if using hardware).
2. Write a Python script in the package directory to import and use the classes, for example:
    ```python
    from ardrone_hftc_AR_dorne_2.0 import HFTCController, DroneInterface, TrajectoryGenerator

    drone = DroneInterface()
    trajectory = TrajectoryGenerator().generate()
    controller = HFTCController(drone, trajectory)
    # ...run your control loop...
    ```
3. Run your script:
    ```bash
    python3 your_script.py
    ```

---

## 3. ROS2 Implementation with Gazebo and PX4

**Requirements:**  
- ROS2 Humble  
- PX4 Autopilot (with Gazebo simulation)  
- colcon  
- numpy, dataclasses (Python 3.7+)  
- QGroundControl (for monitoring)  
- Micro XRCE Agent

**Build & Run Instructions:**

1. **Build the ROS2 package:**
    ```bash
    cd ~/drone_ws
    colcon build --packages-select arsmc_controller
    source install/setup.bash
    ```

2. **Start PX4 SITL with Gazebo:**
    ```bash
    cd ~/PX4-Autopilot
    make px4_sitl gz_x500
    ```

3. **Start the Micro XRCE Agent:**
    ```bash
    source /opt/ros/humble/setup.bash
    MicroXRCEAgent udp4 -p 8888
    ```

4. **(Optional) Start QGroundControl:**
    ```bash
    cd ~/Downloads
    ./QGroundControl-x86_64.AppImage
    ```

5. **(Optional) Start RViz2 for visualization:**
    ```bash
    source /opt/ros/humble/setup.bash
    ros2 run rviz2 rviz2
    ```

6. **Run the ARSMC controller node:**
    ```bash
    source /opt/ros/humble/setup.bash
    ros2 run arsmc_controller arsmc_controller
    ```

---

# 12. Conclusion

The hybrid finite‑time controller demonstrates strong tracking accuracy, fast convergence, and robustness to disturbances.  
Both simulation and hardware results verify the effectiveness of combining AISM, Backstepping, NTSM, and FDO for quadrotor trajectory tracking tasks.

