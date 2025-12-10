"""Base Controller Class"""

import numpy as np
from abc import ABC, abstractmethod


class BaseController(ABC):
    """Abstract base class for all controllers"""
    
    def __init__(self):
        self.history = []
    
    @staticmethod
    def saturate(x, x_min, x_max):
        """Saturation function"""
        return np.clip(x, x_min, x_max)
    
    @abstractmethod
    def compute(self, *args, **kwargs):
        """Compute control output - to be implemented by subclasses"""
        pass
    
    def reset(self):
        """Reset controller state"""
        self.history = []
