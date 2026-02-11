"""
ARSMC Controller Package
Adaptive Robust Sliding Mode Controller for PX4 quadrotor.

Implementation based on AISM + Backstepping control from paper.
"""

__version__ = "1.0.0"
__author__ = "Usama"

from .config import (
    ControllerConfig,
    SystemParams,
    TrajectoryParams,
    AISMParams,
    BacksteppingParams,
    ControlOutputParams,
    DisturbanceParams,
    LoggingParams,
    OccupancyGridParams,
    get_default_config
)

from .controllers import (
    AISMController,
    BacksteppingController
)

from .trajectory import TrajectoryGenerator

from .disturbance import DisturbanceManager

from .data_logger import FlightDataLogger

from .utils import clamp, quaternion_to_yaw

from .occupancy_grid_mapping import (
    OccupancyGridMapper,
    GridConfig
)

__all__ = [
    # Configuration
    'ControllerConfig',
    'SystemParams',
    'TrajectoryParams',
    'AISMParams',
    'BacksteppingParams',
    'ControlOutputParams',
    'DisturbanceParams',
    'LoggingParams',
    'OccupancyGridParams',
    'get_default_config',
    # Controllers
    'AISMController',
    'BacksteppingController',
    # Trajectory
    'TrajectoryGenerator',
    # Disturbance
    'DisturbanceManager',
    # Logging
    'FlightDataLogger',
    # Utils
    'clamp',
    'quaternion_to_yaw',
    # Occupancy Grid
    'OccupancyGridMapper',
    'GridConfig',
]
