"""
Backstepping Controller for Horizontal Position
Equations (26-41) from the paper
"""

import numpy as np
from .base import BaseController
from ..params import QuadrotorParams, ControlParams
from ..trajectory import disturbances


class BacksteppingController(BaseController):
    """Backstepping controller for x and y position"""
    
    def __init__(self):
        super().__init__()
        self.params = QuadrotorParams()
        self.ctrl_params = ControlParams()
    
    def compute(self, x, x_dot, y, y_dot, 
                xd, xd_dot, xd_ddot,
                yd, yd_dot, yd_ddot,
                u1, psi, t):
        """
        Backstepping controller for x and y - EXACT implementation
        
        Args:
            x, x_dot: X position and velocity
            y, y_dot: Y position and velocity
            xd, xd_dot, xd_ddot: Desired X trajectory
            yd, yd_dot, yd_ddot: Desired Y trajectory
            u1: Thrust control input
            psi: Yaw angle
            t: Current time
            
        Returns:
            tuple: (phi_d, theta_d) desired roll and pitch angles
        """
        cx = self.ctrl_params.cx
        kx = self.ctrl_params.kx
        cy = self.ctrl_params.cy
        ky = self.ctrl_params.ky
        
        # X position control (Eqs. 26-33)
        ex = x - xd
        alpha_x = -cx * ex + xd_dot
        zx = x_dot - alpha_x
        
        # Get disturbance d1
        d1 = disturbances(t)[0]
        
        # Virtual control vx (Eq. 30)
        denom = u1 + d1
        if abs(denom) < 0.1:
            denom = 0.1 * np.sign(denom) if denom != 0 else 0.1
        
        vx = -(self.params.M / denom) * (
            ex + cx * (-cx * ex + zx) - xd_ddot + kx * zx
        )
        
        # Y position control (Eqs. 34-41)
        ey = y - yd
        alpha_y = -cy * ey + yd_dot
        zy = y_dot - alpha_y
        
        # Virtual control vy (Eq. 38)
        vy = -(self.params.M / denom) * (
            ey + cy * (-cy * ey + zy) - yd_ddot + ky * zy
        )
        
        # Limit vx, vy
        vx = self.saturate(vx, -0.8, 0.8)
        vy = self.saturate(vy, -0.8, 0.8)
        
        # Desired roll and pitch (Eq. 16)
        arg_phi = vx * np.sin(psi) - vy * np.cos(psi)
        arg_phi = self.saturate(arg_phi, -0.7, 0.7)
        phi_d = np.arcsin(arg_phi)
        
        cos_phi_d = np.cos(phi_d)
        if abs(cos_phi_d) < 0.1:
            cos_phi_d = 0.1 * np.sign(cos_phi_d) if cos_phi_d != 0 else 0.1
        
        arg_theta = (vx * np.cos(psi) + vy * np.sin(psi)) / cos_phi_d
        arg_theta = self.saturate(arg_theta, -0.7, 0.7)
        theta_d = np.arcsin(arg_theta)
        
        # Limit desired angles (±30 degrees)
        phi_d = self.saturate(phi_d, -np.pi/6, np.pi/6)
        theta_d = self.saturate(theta_d, -np.pi/6, np.pi/6)
        
        return phi_d, theta_d
