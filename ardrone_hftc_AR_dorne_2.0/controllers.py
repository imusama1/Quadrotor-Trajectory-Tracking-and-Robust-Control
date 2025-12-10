"""
Control Laws for AR.Drone HFTC Controller
"""

import math
from .config import BacksteppingConfig, SlidingModeConfig, ControlLoopConfig


class BacksteppingController:
    """
    Backstepping position controller (adapted from Paper Eq. 15-18).
    
    ORIGINAL PAPER:
        Step 1: Virtual control α = -c·e + v_d
        Step 2: Error z = v - α = v + c·e - v_d
        Step 3: Control u = -e - c·ė - k·z + a_d - d_hat
    
    ADAPTATION:
        Output is velocity command (not thrust).
        AR.Drone firmware converts velocity ref → attitude → motors.
    """
    
    def __init__(self):
        self.c_x = BacksteppingConfig.C_X
        self.c_y = BacksteppingConfig.C_Y
        self.k_x = BacksteppingConfig.K_X
        self.k_y = BacksteppingConfig.K_Y
    
    def compute(self, ex, ey, vx, vy, vx_d, vy_d, ax_d, ay_d, d_hat_x, d_hat_y):
        """
        Compute backstepping control law.
        
        Args:
            ex, ey: Position errors
            vx, vy: Current velocities
            vx_d, vy_d: Desired velocities
            ax_d, ay_d: Desired accelerations (feedforward)
            d_hat_x, d_hat_y: Disturbance estimates
            
        Returns:
            tuple: (u_x, u_y, z_x, z_y)
        """
        # Velocity errors
        evx = vx - vx_d
        evy = vy - vy_d
        
        # Virtual control errors (Paper Eq. 16)
        z_x = evx + self.c_x * ex
        z_y = evy + self.c_y * ey
        
        # Backstepping control law (Paper Eq. 17-18, adapted)
        u_x = -ex - self.c_x * evx - self.k_x * z_x + ax_d - d_hat_x
        u_y = -ey - self.c_y * evy - self.k_y * z_y + ay_d - d_hat_y
        
        return u_x, u_y, z_x, z_y


class SlidingModeController:
    """
    Integral Sliding Mode for altitude (Paper Eq. 23-25).
    
    Sliding surface with integral:
        s = c·e + ė + k·∫e
    
    Control law:
        u = -c·ė - k·e - h·s - d_hat·tanh(s/ε)
    """
    
    def __init__(self):
        self.c_z = SlidingModeConfig.C_Z
        self.k_z = SlidingModeConfig.K_Z
        self.h_z = SlidingModeConfig.H_Z
        self.varrho = SlidingModeConfig.VARRHO
        
        self.int_ez = 0.0
        self.int_max = ControlLoopConfig.INT_MAX
    
    def compute(self, ez, vz, d_hat_z, dt):
        """
        Compute sliding mode control law.
        
        Args:
            ez: Altitude error
            vz: Vertical velocity
            d_hat_z: Disturbance estimate
            dt: Time step
            
        Returns:
            tuple: (u_z, s_z)
        """
        # Update integral with anti-windup
        self.int_ez += ez * dt
        self.int_ez = max(-self.int_max, min(self.int_max, self.int_ez))
        
        # Sliding surface
        s_z = self.c_z * ez + vz + self.k_z * self.int_ez
        
        # Control law
        u_z = -self.c_z * vz - self.k_z * ez - self.h_z * s_z - d_hat_z * math.tanh(s_z / self.varrho)
        
        return u_z, s_z
    
    def reset(self):
        """Reset integral state."""
        self.int_ez = 0.0
