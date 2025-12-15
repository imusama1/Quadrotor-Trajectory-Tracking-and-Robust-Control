"""Trajectory generation for ARSMC controller."""

import math
from dataclasses import dataclass
from typing import Tuple, Optional

from .config import TrajectoryParams


@dataclass
class TrajectoryState:
    """Complete trajectory state with position, velocity, and acceleration."""
    x: float
    x_dot: float
    x_ddot: float
    y: float
    y_dot: float
    y_ddot: float
    z: float
    z_dot: float
    z_ddot: float
    yaw: float


class TrajectoryGenerator:
    """Generates circle trajectory for quadrotor."""

    def __init__(self, params: TrajectoryParams, initial_yaw: float = 0.0):
        self.params = params
        self.initial_yaw = initial_yaw
        self.circle_start_time: Optional[float] = None
        self.current_loop: int = 0

    def reset(self, initial_yaw: float = 0.0):
        """Reset trajectory generator state."""
        self.initial_yaw = initial_yaw
        self.circle_start_time = None
        self.current_loop = 0

    def get_trajectory(self, t: float) -> Tuple[TrajectoryState, bool]:
        """
        Generate trajectory state at time t.
        
        Args:
            t: Time since trajectory start (seconds)
            
        Returns:
            Tuple of (TrajectoryState, loop_completed_flag)
        """
        p = self.params
        loop_completed = False

        # Hover phase
        if t < p.t_hover:
            return TrajectoryState(
                x=p.circle_center_x, x_dot=0.0, x_ddot=0.0,
                y=p.circle_center_y, y_dot=0.0, y_ddot=0.0,
                z=p.hover_height, z_dot=0.0, z_ddot=0.0,
                yaw=self.initial_yaw
            ), False

        # Initialize circle start time
        if self.circle_start_time is None:
            self.circle_start_time = p.t_hover

        # Circle trajectory
        t_circle = t - self.circle_start_time
        angle = 2 * math.pi * t_circle / p.circle_period
        angle_dot = 2 * math.pi / p.circle_period

        # Loop detection
        current_loop = int(t_circle / p.circle_period)
        if current_loop > self.current_loop:
            self.current_loop = current_loop
            loop_completed = True

        # Position
        x_d = p.circle_center_x + p.circle_radius * math.cos(angle)
        y_d = p.circle_center_y + p.circle_radius * math.sin(angle)

        # Velocity
        x_d_dot = -p.circle_radius * math.sin(angle) * angle_dot
        y_d_dot = p.circle_radius * math.cos(angle) * angle_dot

        # Acceleration
        x_d_ddot = -p.circle_radius * math.cos(angle) * angle_dot ** 2
        y_d_ddot = -p.circle_radius * math.sin(angle) * angle_dot ** 2

        return TrajectoryState(
            x=x_d, x_dot=x_d_dot, x_ddot=x_d_ddot,
            y=y_d, y_dot=y_d_dot, y_ddot=y_d_ddot,
            z=p.hover_height, z_dot=0.0, z_ddot=0.0,
            yaw=self.initial_yaw
        ), loop_completed

    def get_current_angle(self, t: float) -> float:
        """Get current angle on circle trajectory."""
        if self.circle_start_time is None or t < self.params.t_hover:
            return 0.0
        t_circle = t - self.circle_start_time
        return 2 * math.pi * t_circle / self.params.circle_period
