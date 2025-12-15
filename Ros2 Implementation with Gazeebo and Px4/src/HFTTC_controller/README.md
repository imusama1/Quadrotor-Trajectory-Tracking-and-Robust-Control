# ARSMC Controller Package

Adaptive Robust Sliding Mode Controller for PX4 quadrotor  
Implementation based on AISM + Backstepping control from the referenced paper.

## Requirements

- ROS2 Humble
- PX4 Autopilot (with Gazebo simulation)
- colcon
- numpy, dataclasses (Python 3.7+)
- QGroundControl (for monitoring)
- Micro XRCE Agent

## Build & Run Instructions

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

## Notes

- Make sure all dependencies are installed and sourced before running.
- The controller parameters and configuration are in the Python package.
- Data logging and disturbance simulation are included in the package.
- For custom configuration, edit the relevant dataclasses in the code.

---
