"""Disturbance simulation for ARSMC controller."""

import math
import random
from dataclasses import dataclass
from typing import Tuple, Optional, Callable

from .config import DisturbanceParams, TrajectoryParams


@dataclass
class DisturbanceState:
    """Current disturbance state."""
    active: bool = False
    start_time: Optional[float] = None
    magnitude: float = 0.0
    duration: float = 0.0
    direction_angle: float = 0.0
    x_offset: float = 0.0
    y_offset: float = 0.0


class DisturbanceManager:
    """Manages random disturbances for trajectory tracking."""

    def __init__(self, params: DisturbanceParams, trajectory_params: TrajectoryParams,
                 logger: Optional[Callable] = None):
        self.params = params
        self.trajectory_params = trajectory_params
        self.logger = logger
        
        self.state = DisturbanceState()
        self.last_disturbance_loop: int = -1
        self.disturbance_counter: int = 0
        self._disturbed_setpoint: Tuple[float, float] = (0.0, 0.0)

    def reset(self):
        """Reset disturbance manager state."""
        self.state = DisturbanceState()
        self.last_disturbance_loop = -1
        self.disturbance_counter = 0

    def try_trigger(self, current_loop: int, current_angle: float) -> bool:
        """
        Try to trigger a disturbance based on probability.
        
        Args:
            current_loop: Current trajectory loop number
            current_angle: Current angle on circle trajectory
            
        Returns:
            True if disturbance was triggered
        """
        if self.state.active:
            return False
            
        if current_loop <= self.last_disturbance_loop:
            return False
            
        if random.random() >= self.params.probability:
            return False

        self._activate_disturbance(current_angle)
        self.last_disturbance_loop = current_loop
        return True

    def _activate_disturbance(self, current_angle: float):
        """Activate a new disturbance."""
        self.state.active = True
        self.disturbance_counter += 1
        
        # Calculate current desired position
        p = self.trajectory_params
        x_d = p.circle_center_x + p.circle_radius * math.cos(current_angle)
        y_d = p.circle_center_y + p.circle_radius * math.sin(current_angle)
        
        # Random parameters
        self.state.magnitude = random.uniform(*self.params.magnitude_range)
        self.state.duration = random.uniform(*self.params.duration_range)
        self.state.direction_angle = random.uniform(0, 2 * math.pi)
        
        # Calculate offset
        self.state.x_offset = self.state.magnitude * math.cos(self.state.direction_angle)
        self.state.y_offset = self.state.magnitude * math.sin(self.state.direction_angle)
        
        # Store disturbed setpoint
        self._disturbed_setpoint = (
            x_d + self.state.x_offset,
            y_d + self.state.y_offset
        )
        
        if self.logger:
            dir_deg = math.degrees(self.state.direction_angle) % 360
            self.logger(
                f"🌪️ RANDOM DISTURBANCE #{self.disturbance_counter}! "
                f"Push: {self.state.magnitude:.1f}m, "
                f"Direction: {dir_deg:.0f}°, "
                f"Duration: {self.state.duration:.1f}s"
            )

    def update(self, current_time: float):
        """
        Update disturbance state based on current time.
        
        Args:
            current_time: Current time in seconds
        """
        if not self.state.active:
            return
            
        if self.state.start_time is None:
            self.state.start_time = current_time
            return
            
        elapsed = current_time - self.state.start_time
        if elapsed >= self.state.duration:
            self.state.active = False
            self.state.start_time = None
            if self.logger:
                self.logger(f"🔄 Recovery after disturbance #{self.disturbance_counter}")

    def get_setpoint(self, x_d: float, y_d: float, current_time: float) -> Tuple[float, float]:
        """
        Get possibly disturbed setpoint.
        
        Args:
            x_d: Desired x position
            y_d: Desired y position
            current_time: Current time in seconds
            
        Returns:
            Tuple of (x_setpoint, y_setpoint)
        """
        self.update(current_time)
        
        if not self.state.active:
            return x_d, y_d
            
        return self._disturbed_setpoint

    @property
    def is_active(self) -> bool:
        """Check if disturbance is currently active."""
        return self.state.active
