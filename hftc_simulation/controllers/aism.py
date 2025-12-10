"""
Adaptive Integral Sliding Mode (AISM) Controller for Altitude
Equations (21-25) from the paper
"""

import numpy as np
from .base import BaseController
from ..params import QuadrotorParams, ControlParams


class AISMController(BaseController):
    """AISM controller for altitude control"""
    
    def __init__(self):
        super().__init__()
        self.params = QuadrotorParams()
        self.ctrl_params = ControlParams()
        
        # Controller states
        self.d1_hat_max = 0.0
        self.sz_integral = 0.0
    
    def compute(self, z, z_dot, zd, zd_dot, zd_ddot, phi, theta, t, dt):
        """
        AISM controller for altitude - EXACT implementation
        
        Args:
            z: Current altitude
            z_dot: Current vertical velocity
            zd: Desired altitude
            zd_dot: Desired vertical velocity
            zd_ddot: Desired vertical acceleration
            phi: Roll angle
            theta: Pitch angle
            t: Current time
            dt: Time step
            
        Returns:
            float: Thrust control input u1
        """
        cz = self.ctrl_params.cz
        kz = self.ctrl_params.kz
        hz = self.ctrl_params.hz
        phi1 = self.ctrl_params.phi1
        rho = self.ctrl_params.rho
        
        # Tracking error (Eq. 21)
        ez = z - zd
        
        # AISM manifold (Eq. 22)
        self.sz_integral += ez * dt
        self.sz_integral = self.saturate(self.sz_integral, -5.0, 5.0)
        
        sz = cz * ez + (z_dot - zd_dot) + kz * self.sz_integral
        sz = self.saturate(sz, -10.0, 10.0)
        
        # Adaptive law (Eq. 25)
        d_d1_hat_max = phi1 * sz * np.tanh(sz / rho)
        self.d1_hat_max += d_d1_hat_max * dt
        self.d1_hat_max = self.saturate(self.d1_hat_max, 0.0, 5.0)
        
        # Control law (Eq. 24)
        cos_phi_theta = np.cos(phi) * np.cos(theta)
        if abs(cos_phi_theta) < 0.1:
            cos_phi_theta = 0.1 * np.sign(cos_phi_theta) if cos_phi_theta != 0 else 0.1
        
        u1 = (self.params.M / cos_phi_theta) * (
            self.params.g + zd_ddot - cz * (z_dot - zd_dot) - 
            kz * ez - hz * sz - self.d1_hat_max * np.tanh(sz / rho)
        )
        
        # Ensure positive thrust with reasonable limits
        u1 = self.saturate(u1, 0.5, 20.0)
        
        return u1
    
    def reset(self):
        """Reset controller state"""
        super().reset()
        self.d1_hat_max = 0.0
        self.sz_integral = 0.0
