"""
ARSMC Controller Package
Adaptive Robust Sliding Mode Controller for PX4 quadrotor.

Implementation based on AISM + Backstepping control from paper.
"""

__version__ = "1.0.0"
__author__ = "Usama"

from .arsmc_controller import PX4OuterLoop, main
