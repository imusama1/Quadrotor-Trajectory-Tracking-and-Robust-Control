"""
AR.Drone 2.0 HFTC Controller Package
====================================
Backstepping + Integral Sliding Mode + Disturbance Observer
"""

from .controller import HFTCController
from .drone_interface import DroneInterface
from .trajectory import TrajectoryGenerator
from .observers import DisturbanceObserver
from .plotting import ResultsPlotter

__version__ = "1.0.0"
__all__ = [
    'HFTCController',
    'DroneInterface', 
    'TrajectoryGenerator',
    'DisturbanceObserver',
    'ResultsPlotter'
]
