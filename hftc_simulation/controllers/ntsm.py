"""
Non-singular Terminal Sliding Mode (NTSM) Controller for Attitude
Equations (48-60) from the paper
"""

import numpy as np
from .base import BaseController
from ..params import QuadrotorParams, ControlParams


class NTSMController(BaseController):
    """NTSM controller with FDO integration for attitude control"""
    
    def __init__(self):
        super().__init__()
        self.params = QuadrotorParams()
        self.ctrl_params = ControlParams()
    
    def compute(self, eta, omega, eta_d, omega_d, omega_dot_d, d_hat):
        """
        NTSM controller with FDO integration
        
        Args:
            eta: Current attitude [phi, theta, psi]
            omega: Current angular velocities [p, q, r]
            eta_d: Desired attitude
            omega_d: Desired angular velocities
            omega_dot_d: Desired angular accelerations
            d_hat: Disturbance estimate from FDO
            
        Returns:
            tuple: (u2, u3, u4, s, f_omega) control inputs, sliding surface, nonlinear terms
        """
        beta = self.ctrl_params.beta
        m = self.ctrl_params.m
        n = self.ctrl_params.n
        K = self.ctrl_params.K
        
        # Tracking errors (Eq. 48)
        e_eta = eta - eta_d
        e_omega = omega - omega_d
        
        # NTSM manifold (Eq. 49)
        sign_e_omega = np.sign(e_omega + 1e-10)
        abs_e_omega = np.abs(e_omega) + 1e-10
        
        power_term = abs_e_omega**(float(m)/float(n)) * sign_e_omega
        s = e_eta + (1.0/beta) * power_term
        
        # Nonlinear terms f(Ω) from Eq. (14)
        p, q, r = omega
        omega_r = 0.0  # Simplified
        
        f_omega = np.array([
            self.params.a1 * q * r + self.params.a2 * q * omega_r,
            self.params.a3 * p * r + self.params.a4 * p * omega_r,
            self.params.a5 * p * q
        ])
        
        # Control matrix g and its inverse
        g_matrix = np.diag([self.params.b1, self.params.b2, self.params.b3])
        g_inv = np.linalg.inv(g_matrix)
        
        # Control law (Eq. 51)
        abs_e_omega_safe = np.maximum(abs_e_omega, 1e-6)
        diag_term = np.diag(abs_e_omega_safe**(2.0 - float(m)/float(n)))
        
        term1 = beta * (float(n)/float(m)) * diag_term @ s
        term2 = K @ np.tanh(s)
        
        u_attitude = -g_inv @ (
            f_omega + term1 + term2 + d_hat - omega_dot_d
        )
        
        # Separate control inputs with saturation
        u2 = self.saturate(u_attitude[0], -30.0, 30.0)
        u3 = self.saturate(u_attitude[1], -30.0, 30.0)
        u4 = self.saturate(u_attitude[2], -20.0, 20.0)
        
        return u2, u3, u4, s, f_omega
