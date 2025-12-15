# AR.Drone 2.0 HFTC Controller Package

## Overview
Implements Backstepping + Integral Sliding Mode + Disturbance Observer control for AR.Drone 2.0.

## Requirements
- Python 3.x
- numpy
- matplotlib
- (optional) ROS (if using with real AR.Drone 2.0 hardware)
- AR.Drone 2.0 (for hardware-in-the-loop)

## Installation
Install dependencies:
```bash
pip install numpy matplotlib
```
(Install ROS if required for your hardware setup.)

## Usage
1. Connect your computer to the AR.Drone 2.0 WiFi (if using hardware).
2. Write a Python script in this directory to import and use the classes, for example:
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
