"""
Finite-Time Disturbance Observer (FDO)
Theorem 1 (Equation 44) from the paper
"""

import numpy as np
from ..params import QuadrotorParams, ControlParams
from ..trajectory import disturbances


class FiniteTimeDisturbanceObserver:
    """EXACT implementation of FDO from Theorem 1 (Equation 44)"""
    
    def __init__(self):
        self.params = QuadrotorParams()
        self.ctrl_params = ControlParams()
        
        # Observer states (z0, z1, z2) for each channel (φ, θ, ψ)
        self.z0 = np.zeros(3)  # Estimate of angular velocities
        self.z1 = np.zeros(3)  # Estimate of disturbances d2, d3, d4
        self.z2 = np.zeros(3)  # Estimate of disturbance derivatives
        
        # History for plotting
        self.d_est_history = []
        self.d_true_history = []
        self.z0_history = []
        self.z1_history = []
        self.z2_history = []
    
    @staticmethod
    def sig_alpha(x, alpha, eps=1e-10):
        """sig^alpha(x) = |x|^alpha * sign(x) - EXACT from Lemma 1"""
        return np.abs(x + eps)**alpha * np.sign(x + eps)
    
    def update(self, omega, f_omega, u_attitude, dt, t):
        """
        Update FDO states according to Theorem 1 (Equation 44)
        
        Args:
            omega: Current angular velocities [p, q, r]
            f_omega: Nonlinear dynamics terms
            u_attitude: Attitude control inputs [u2, u3, u4]
            dt: Time step
            t: Current time
            
        Returns:
            np.array: Disturbance estimate z1
        """
        lambda1 = self.ctrl_params.lambda1
        lambda2 = self.ctrl_params.lambda2
        lambda3 = self.ctrl_params.lambda3
        L = self.ctrl_params.L
        
        # Control matrix g
        g_matrix = np.diag([self.params.b1, self.params.b2, self.params.b3])
        
        # Equation (44) - ż0
        mu0 = self.z0 - omega
        zeta0 = -lambda1 * (L**(1.0/3.0)) * self.sig_alpha(mu0, 2.0/3.0) + self.z1
        
        dz0 = zeta0 + f_omega + g_matrix @ u_attitude
        self.z0 += dz0 * dt
        
        # Equation (44) - ż1
        mu1 = self.z1 - zeta0
        zeta1 = -lambda2 * (L**(1.0/2.0)) * self.sig_alpha(mu1, 1.0/2.0) + self.z2
        
        dz1 = zeta1
        self.z1 += dz1 * dt
        
        # Equation (44) - ż2
        mu2 = self.z2 - zeta1
        dz2 = -lambda3 * L * np.sign(mu2 + 1e-10)
        self.z2 += dz2 * dt
        
        # Store for plotting
        true_disturbances = disturbances(t)[1:4]
        self.d_est_history.append(self.z1.copy())
        self.d_true_history.append(true_disturbances.copy())
        self.z0_history.append(self.z0.copy())
        self.z1_history.append(self.z1.copy())
        self.z2_history.append(self.z2.copy())
        
        return self.z1
    
    def reset(self):
        """Reset observer state"""
        self.z0 = np.zeros(3)
        self.z1 = np.zeros(3)
        self.z2 = np.zeros(3)
        self.d_est_history = []
        self.d_true_history = []
        self.z0_history = []
        self.z1_history = []
        self.z2_history = []
