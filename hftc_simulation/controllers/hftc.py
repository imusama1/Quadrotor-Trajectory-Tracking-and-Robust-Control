"""
Hybrid Finite-Time Controller (HFTC) - Complete Implementation
Combines AISM, Backstepping, FDO, and NTSM
"""

import numpy as np
from .aism import AISMController
from .backstepping import BacksteppingController
from .ntsm import NTSMController
from ..observers.fdo import FiniteTimeDisturbanceObserver
from ..trajectory import desired_trajectory


class HFTCController:
    """Complete HFTC implementation combining all controllers"""
    
    def __init__(self):
        # Initialize sub-controllers
        self.aism = AISMController()
        self.backstepping = BacksteppingController()
        self.ntsm = NTSMController()
        self.fdo = FiniteTimeDisturbanceObserver()
        
        # Desired attitudes
        self.phi_d = 0.0
        self.theta_d = 0.0
        self.psi_d = 0.0
        
        # Desired angular velocities
        self.p_d = 0.0
        self.q_d = 0.0
        self.r_d = 0.0
        
        # History
        self.time_history = []
        self.state_history = []
        self.control_history = []
        self.error_history = []
        self.sliding_history = []
    
    def compute_control(self, state, t, dt):
        """
        Compute all control inputs - Complete HFTC
        
        Args:
            state: [x, x_dot, y, y_dot, z, z_dot, phi, p, theta, q, psi, r]
            t: Current time
            dt: Time step
            
        Returns:
            tuple: (u, f_omega, u_attitude, d_hat)
        """
        # Unpack state
        x, x_dot, y, y_dot, z, z_dot, phi, p, theta, q, psi, r = state
        
        # Get desired trajectory
        traj = desired_trajectory(t)
        xd, xd_dot, xd_ddot = traj[0], traj[1], traj[2]
        yd, yd_dot, yd_ddot = traj[3], traj[4], traj[5]
        zd, zd_dot, zd_ddot = traj[6], traj[7], traj[8]
        psid, psid_dot, psid_ddot = traj[9], traj[10], traj[11]
        
        # 1. Altitude control (AISM)
        u1 = self.aism.compute(z, z_dot, zd, zd_dot, zd_ddot, phi, theta, t, dt)
        
        # 2. Position control (Backstepping)
        self.phi_d, self.theta_d = self.backstepping.compute(
            x, x_dot, y, y_dot, xd, xd_dot, xd_ddot, 
            yd, yd_dot, yd_ddot, u1, psi, t
        )
        
        # Set desired yaw
        self.psi_d = psid
        self.r_d = psid_dot
        
        # Desired angular velocities (simplified)
        omega_d = np.array([0.0, 0.0, self.r_d])
        omega_dot_d = np.array([0.0, 0.0, psid_ddot])
        
        # 3. Attitude control (FDO-based NTSM)
        eta = np.array([phi, theta, psi])
        eta_d = np.array([self.phi_d, self.theta_d, self.psi_d])
        omega = np.array([p, q, r])
        
        # Get disturbance estimate
        d_hat = self.fdo.z1
        
        # Get attitude control
        u2, u3, u4, s, f_omega = self.ntsm.compute(
            eta, omega, eta_d, omega_d, omega_dot_d, d_hat
        )
        
        # Combine control inputs
        u = np.array([u1, u2, u3, u4])
        u_attitude = np.array([u2, u3, u4])
        
        # Store history
        self.time_history.append(t)
        self.state_history.append(state.copy())
        self.control_history.append(u.copy())
        self.sliding_history.append(s.copy())
        
        # Calculate errors
        errors = np.array([
            z - zd, x - xd, y - yd, psi - psid,
            phi - self.phi_d, theta - self.theta_d
        ])
        self.error_history.append(errors)
        
        return u, f_omega, u_attitude, d_hat
    
    def update_fdo(self, omega, f_omega, u_attitude, dt, t):
        """Update the FDO with current measurements"""
        return self.fdo.update(omega, f_omega, u_attitude, dt, t)
    
    def reset(self):
        """Reset all controllers"""
        self.aism.reset()
        self.fdo.reset()
        self.phi_d = 0.0
        self.theta_d = 0.0
        self.psi_d = 0.0
        self.time_history = []
        self.state_history = []
        self.control_history = []
        self.error_history = []
        self.sliding_history = []
