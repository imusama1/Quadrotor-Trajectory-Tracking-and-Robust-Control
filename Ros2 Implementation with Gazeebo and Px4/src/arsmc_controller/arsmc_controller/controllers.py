"""AISM and Backstepping controllers for ARSMC."""

import math
from dataclasses import dataclass
from typing import Tuple

from .config import AISMParams, BacksteppingParams, ControlOutputParams
from .utils import clamp


@dataclass
class AISMState:
    """Internal state of AISM controller."""
    ez_int: float = 0.0
    d1_hat: float = 0.0
    s_z: float = 0.0


class AISMController:
    """
    Adaptive Integral Sliding Mode controller for altitude.
    
    Based on Paper Section 3.1, Equations (22-25).
    """

    def __init__(self, params: AISMParams, output_params: ControlOutputParams, dt: float):
        self.params = params
        self.output_params = output_params
        self.dt = dt
        self.state = AISMState()

    def reset(self):
        """Reset controller state."""
        self.state = AISMState()

    def compute(self, z: float, z_dot: float, 
                z_d: float, z_d_dot: float, z_d_ddot: float) -> Tuple[float, AISMState]:
        """
        Compute AISM control for altitude.
        
        Args:
            z: Current altitude
            z_dot: Current vertical velocity
            z_d: Desired altitude
            z_d_dot: Desired vertical velocity
            z_d_ddot: Desired vertical acceleration
            
        Returns:
            Tuple of (corrected_z_d, controller_state)
        """
        p = self.params
        
        # Tracking errors
        ez = z - z_d
        ez_dot = z_dot - z_d_dot

        # Integral term (paper Eq. 22)
        self.state.ez_int += ez * self.dt
        self.state.ez_int = clamp(self.state.ez_int, -p.integral_limit, p.integral_limit)

        # Sliding surface (paper Eq. 22)
        s_z = p.c_z * ez + ez_dot + p.k_z * self.state.ez_int
        self.state.s_z = s_z

        # Adaptive law (paper Eq. 25)
        if abs(s_z) > 0.05:
            d1_hat_dot = p.phi1_z * abs(s_z) * math.tanh(abs(s_z) / p.varrho_z)
            self.state.d1_hat += d1_hat_dot * self.dt
        else:
            # Gradual decay when no significant errors
            self.state.d1_hat *= 0.98
        
        self.state.d1_hat = clamp(self.state.d1_hat, 0.0, p.d1_hat_limit)

        # Control law (paper Eq. 24)
        control_effort = (z_d_ddot - p.c_z * ez_dot - p.k_z * ez 
                         - p.h_z * s_z - self.state.d1_hat * math.tanh(s_z / p.varrho_z))
        
        # Convert to position correction
        z_corr = self.output_params.vertical_gain * control_effort
        z_d_corrected = z_d + clamp(z_corr, -self.output_params.vertical_limit, 
                                     self.output_params.vertical_limit)

        return z_d_corrected, self.state


class BacksteppingController:
    """
    Backstepping controller for horizontal position.
    
    Based on Paper Section 3.2, Equations (26-30).
    """

    def __init__(self, c: float, k: float, output_params: ControlOutputParams):
        self.c = c
        self.k = k
        self.output_params = output_params

    def compute(self, pos: float, vel: float,
                pos_d: float, vel_d: float, acc_d: float) -> float:
        """
        Compute backstepping control for one axis.
        
        Args:
            pos: Current position
            vel: Current velocity
            pos_d: Desired position
            vel_d: Desired velocity
            acc_d: Desired acceleration
            
        Returns:
            Corrected desired position
        """
        c, k = self.c, self.k
        
        # Step 1: First error (paper Eq. 26)
        e = pos - pos_d
        
        # Virtual control (paper Eq. 27)
        alpha = -c * e + vel_d
        
        # Step 2: New error (paper Eq. 29)
        z = vel - alpha
        
        # Control law (paper Eq. 30)
        virtual_control = -(e + c * (-c * e + z) - acc_d + k * z)
        
        # Convert to position correction
        pos_corr = self.output_params.horizontal_gain * virtual_control
        pos_d_corrected = pos_d + clamp(pos_corr, -self.output_params.horizontal_limit,
                                         self.output_params.horizontal_limit)

        return pos_d_corrected


class HorizontalController:
    """Combined horizontal (X-Y) controller using backstepping."""

    def __init__(self, params: BacksteppingParams, output_params: ControlOutputParams):
        self.x_controller = BacksteppingController(params.cx, params.kx, output_params)
        self.y_controller = BacksteppingController(params.cy, params.ky, output_params)

    def compute(self, x: float, vx: float, x_d: float, vx_d: float, ax_d: float,
                y: float, vy: float, y_d: float, vy_d: float, ay_d: float) -> Tuple[float, float]:
        """
        Compute horizontal control for both axes.
        
        Returns:
            Tuple of (corrected_x_d, corrected_y_d)
        """
        x_d_corrected = self.x_controller.compute(x, vx, x_d, vx_d, ax_d)
        y_d_corrected = self.y_controller.compute(y, vy, y_d, vy_d, ay_d)
        return x_d_corrected, y_d_corrected
