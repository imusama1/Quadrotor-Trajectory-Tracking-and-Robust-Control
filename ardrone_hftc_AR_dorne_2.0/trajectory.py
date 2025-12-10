"""
Trajectory Generation for AR.Drone HFTC Controller
"""

import math
from .config import TrajectoryConfig


class TrajectoryGenerator:
    """
    Generates reference trajectories for the controller.
    
    Default: Circular trajectory starting at origin
    """
    
    def __init__(self, radius=None, period=None, hover_height=None):
        self.radius = radius or TrajectoryConfig.RADIUS
        self.period = period or TrajectoryConfig.PERIOD
        self.omega = 2 * math.pi / self.period
        self.hover_height = hover_height or TrajectoryConfig.HOVER_HEIGHT
    
    def get_circle_reference(self, t):
        """
        Circular trajectory reference.
        
        Parameterization (starts at origin, goes right first):
            x_d(t) = r · sin(ωt)
            y_d(t) = r · (1 - cos(ωt))
            z_d(t) = h (constant)
        
        Args:
            t: Time in seconds
            
        Returns:
            dict: Position, velocity, and acceleration references
        """
        angle = self.omega * t
        
        # Position
        x_d = self.radius * math.sin(angle)
        y_d = self.radius * (1 - math.cos(angle))
        z_d = self.hover_height
        
        # Velocity (feedforward)
        vx_d = self.radius * self.omega * math.cos(angle)
        vy_d = self.radius * self.omega * math.sin(angle)
        vz_d = 0.0
        
        # Acceleration (feedforward)
        ax_d = -self.radius * self.omega**2 * math.sin(angle)
        ay_d = self.radius * self.omega**2 * math.cos(angle)
        az_d = 0.0
        
        return {
            'x_d': x_d, 'y_d': y_d, 'z_d': z_d,
            'vx_d': vx_d, 'vy_d': vy_d, 'vz_d': vz_d,
            'ax_d': ax_d, 'ay_d': ay_d, 'az_d': az_d
        }
    
    def get_hover_reference(self, x=0.0, y=0.0, z=None):
        """
        Stationary hover reference.
        
        Args:
            x, y: Hover position
            z: Hover altitude (default: hover_height)
            
        Returns:
            dict: Position, velocity, and acceleration references
        """
        z = z or self.hover_height
        
        return {
            'x_d': x, 'y_d': y, 'z_d': z,
            'vx_d': 0.0, 'vy_d': 0.0, 'vz_d': 0.0,
            'ax_d': 0.0, 'ay_d': 0.0, 'az_d': 0.0
        }
    
    def get_line_reference(self, t, start, end, duration):
        """
        Linear trajectory between two points.
        
        Args:
            t: Time in seconds
            start: (x, y, z) start position
            end: (x, y, z) end position
            duration: Time to complete trajectory
            
        Returns:
            dict: Position, velocity, and acceleration references
        """
        # Clamp progress to [0, 1]
        progress = min(1.0, max(0.0, t / duration))
        
        # Linear interpolation
        x_d = start[0] + progress * (end[0] - start[0])
        y_d = start[1] + progress * (end[1] - start[1])
        z_d = start[2] + progress * (end[2] - start[2])
        
        # Constant velocity during motion
        if t < duration:
            vx_d = (end[0] - start[0]) / duration
            vy_d = (end[1] - start[1]) / duration
            vz_d = (end[2] - start[2]) / duration
        else:
            vx_d = vy_d = vz_d = 0.0
        
        return {
            'x_d': x_d, 'y_d': y_d, 'z_d': z_d,
            'vx_d': vx_d, 'vy_d': vy_d, 'vz_d': vz_d,
            'ax_d': 0.0, 'ay_d': 0.0, 'az_d': 0.0
        }
