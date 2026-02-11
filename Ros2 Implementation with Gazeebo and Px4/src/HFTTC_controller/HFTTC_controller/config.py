"""Configuration parameters for ARSMC controller."""

from dataclasses import dataclass, field
from typing import List


@dataclass
class SystemParams:
    """System physical parameters."""
    mass: float = 2.0  # kg
    gravity: float = 9.81  # m/s²


@dataclass
class TrajectoryParams:
    """Trajectory parameters."""
    hover_height: float = -2.0  # NED convention (negative = up)
    t_hover: float = 5.0  # seconds
    circle_radius: float = 2.25  # meters
    circle_period: float = 20.0  # seconds
    circle_center_x: float = 0.0
    circle_center_y: float = 0.0


@dataclass
class AISMParams:
    """AISM (Adaptive Integral Sliding Mode) controller parameters."""
    c_z: float = 1.8
    k_z: float = 0.8
    h_z: float = 9.0
    phi1_z: float = 3.0
    varrho_z: float = 0.15
    d1_hat_limit: float = 2.5
    integral_limit: float = 2.0


@dataclass
class BacksteppingParams:
    """Backstepping controller parameters."""
    cx: float = 2.5
    kx: float = 2.5
    cy: float = 2.5
    ky: float = 2.5


@dataclass
class ControlOutputParams:
    """Control output scaling parameters."""
    horizontal_gain: float = 0.05
    vertical_gain: float = 0.08
    horizontal_limit: float = 1.0
    vertical_limit: float = 0.8


@dataclass
class DisturbanceParams:
    """Disturbance simulation parameters."""
    magnitude_range: List[float] = field(default_factory=lambda: [1.5, 4.0])
    duration_range: List[float] = field(default_factory=lambda: [2.0, 5.0])
    probability: float = 0.7
    timing_offset_range: List[float] = field(default_factory=lambda: [0.1, 0.8])


@dataclass
class LoggingParams:
    """Data logging parameters."""
    max_loops: int = 4
    output_dir_base: str = '/home/usama/Drone/Python Implementation'
    min_samples_to_save: int = 100


@dataclass
class OccupancyGridParams:
    """Occupancy grid mapping parameters."""
    # Grid dimensions (meters)
    x_min: float = -10.0
    x_max: float = 10.0
    y_min: float = -10.0
    y_max: float = 10.0
    z_min: float = -5.0
    z_max: float = 5.0
    
    # Grid resolution (meters per cell)
    resolution: float = 0.1
    
    # Occupancy thresholds
    occupied_threshold: float = 0.65
    free_threshold: float = 0.35
    
    # Safety margin around obstacles (meters)
    safety_margin: float = 0.5
    
    # Enable 3D grid
    use_3d: bool = False


@dataclass
class ControllerConfig:
    """Complete controller configuration."""
    system: SystemParams = field(default_factory=SystemParams)
    trajectory: TrajectoryParams = field(default_factory=TrajectoryParams)
    aism: AISMParams = field(default_factory=AISMParams)
    backstepping: BacksteppingParams = field(default_factory=BacksteppingParams)
    control_output: ControlOutputParams = field(default_factory=ControlOutputParams)
    disturbance: DisturbanceParams = field(default_factory=DisturbanceParams)
    logging: LoggingParams = field(default_factory=LoggingParams)
    occupancy_grid: OccupancyGridParams = field(default_factory=OccupancyGridParams)
    control_rate: float = 50.0  # Hz


def get_default_config() -> ControllerConfig:
    """Get default controller configuration."""
    return ControllerConfig()
