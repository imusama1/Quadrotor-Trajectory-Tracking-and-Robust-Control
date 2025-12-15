# Controllers Package

## Overview
This package provides several controllers for quadrotor simulation:
- AISMController
- BacksteppingController
- NTSMController
- HFTCController

## Requirements
- Python 3.x
- numpy

## Installation
Install dependencies:
```bash
pip install numpy
```

## Usage
These controllers are used as part of the main simulation.  
You do **not** run them directly.  
Instead, run the main simulation script:

```bash
python -m hftc_simulation.main
```
or
```bash
python /home/usama/Drone Git/Quadrotor-Trajectory-Tracking-and-Robust-Control/hftc_simulation/main.py
```

The simulation will automatically use the controllers and generate results and plots.
