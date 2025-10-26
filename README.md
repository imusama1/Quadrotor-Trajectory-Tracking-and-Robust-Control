# Quadrotor Trajectory Tracking and Robust Control
Robust adaptive control for quadrotor trajectory tracking

## Overview
This project explores the design of a robust adaptive control system for a quadrotor capable of following a three-dimensional trajectory while maintaining stability under external disturbances and physical constraints.  
The controller will be based on the Adaptive Recursive Sliding Mode Control (ARSMC) approach and will first be developed in MATLAB/Simulink, then integrated into a ROS 2 environment for simulation and testing.

**Goal:** Enable the quadrotor to follow a desired trajectory and maintain or recover its path when affected by disturbances or environmental constraints.

---

## Motivation
Conventional control strategies such as PID and LQR often perform poorly when the system is subject to uncertainty, wind, or actuator limits.  
This work aims to develop a control method that:

- achieves precise trajectory tracking in three-dimensional space,  
- maintains or re-acquires the intended path after disturbances, and  
- can be implemented in a ROS 2 simulation framework and eventually on real hardware.

---

## Development Plan

| Phase | Description | Status |
|-------|--------------|--------|
| 1 | Problem formulation, system modelling, and literature study | In progress |
| 2 | Controller design and MATLAB/Simulink simulation | Planned |
| 3 | ROS 2 implementation of the controller node | Planned |
| 4 | Gazebo simulation and trajectory validation | Planned |
| 5 | Hardware-in-the-loop and real-time testing | Future work |

---

## System Overview

```mermaid
flowchart TD
    A["Reference Trajectory (x, y, z, \psi)"] --> B[Trajectory Planner]
    B --> C[ARSMC Controller]
    C --> D[Quadrotor Dynamics (Simulation / ROS 2 Node)]
    D --> E[Feedback Sensors (IMU, GPS, Encoders)]
    E --> C
````

*Figure 1. Conceptual closed-loop structure for robust trajectory tracking.*

### Intended Controller Design

The planned controller will integrate the following elements:

  * **Adaptive Recursive Sliding Mode Control (ARSMC):** ensures robustness to modelling errors and disturbances.
  * **Adaptive parameter estimation:** adjusts control gains in real time to handle unknown bounds.
  * **Trajectory generation and feedback loop:** continuously compares desired and actual states to minimise tracking error.

Expected outcomes include low steady-state error, smooth control inputs, and resilience to environmental perturbations.

### Tools and Frameworks

  * **MATLAB / Simulink** – modelling and simulation
  * **ROS 2 (Humble)** – controller implementation and communication
  * **Gazebo / Ignition** – virtual environment for testing
  * **Python / C++** – software development
  * **PX4 / MAVROS** – interface with flight-control hardware

### Current Status

The project is currently in the problem definition and exploration stage.

Work is focused on understanding the system dynamics, reviewing related control strategies, and preparing the simulation environment.
The repository will be updated as development progresses.

### Reference

Chen, L., Liu, Z., Gao, H., & Wang, G. (2022).  
Robust Adaptive Recursive Sliding Mode Attitude Control for a Quadrotor with Unknown Disturbances.  
*ISA Transactions, 122,* 114–125.  
https://doi.org/10.1016/j.isatra.2021.04.046

### Authors

**Muhammad Usama Javaid**

MSc – Computer Vision & Robotics (VIBOT)

Université de Bourgogne  
[LinkedIn](https://www.linkedin.com/in/imusama/) | [GitHub](https://github.com/imusama1)

**Team Members:**

  * [Ahmed Khalil](https://github.com/ahmad-laradev)

### License

This repository is created for academic and research purposes under the VIBOT MSc program.

Use or reference is permitted with proper credit.

```
```
