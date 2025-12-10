"""
HFTC Simulation Package
Hybrid Finite-Time Trajectory Tracking Control of a Quadrotor
Based on: N. Wang et al. - ISA Transactions 90 (2019) 278-286
"""

from .params import QuadrotorParams, ControlParams
from .trajectory import desired_trajectory, disturbances
from .controllers.aism import AISMController
from .controllers.backstepping import BacksteppingController
from .controllers.ntsm import NTSMController
from .observers.fdo import FiniteTimeDisturbanceObserver
from .controllers.hftc import HFTCController
from .simulation import Simulator
from .plotting import PlotGenerator

__version__ = "1.0.0"
__author__ = "HFTC Implementation"
