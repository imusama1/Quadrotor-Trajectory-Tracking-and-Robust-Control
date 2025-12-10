"""
Fixed-Step Euler Integration Simulation
"""

import numpy as np
from .params import QuadrotorParams
from .trajectory import disturbances
from .controllers.hftc import HFTCController


class Simulator:
    """Fixed-step Euler integration simulator for quadrotor"""
    
    def __init__(self, T=20.0, dt=0.002):
        """
        Initialize simulator
        
        Args:
            T: Simulation time (s)
            dt: Fixed time step (s)
        """
        self.T = T
        self.dt = dt
        self.N = int(T / dt)
        self.params = QuadrotorParams()
        
        # Initial state
        self.initial_state = np.array([
            -2.0, 0.0,   # x, ẋ
            -2.0, 0.0,   # y, ẏ
            0.0, 0.0,    # z, ż
            0.0, 0.0,    # φ, φ̇
            0.0, 0.0,    # θ, θ̇
            0.0, 0.0     # ψ, ψ̇
        ])
    
    def run(self, verbose=True):
        """
        Run simulation
        
        Args:
            verbose: Print progress
            
        Returns:
            tuple: (t_array, states_array, controls_array, errors_array, 
                   sliding_array, fdo_est, fdo_true, controller)
        """
        t_array = np.linspace(0, self.T, self.N)
        
        # Initialize state and controller
        state = self.initial_state.copy()
        controller = HFTCController()
        
        # Storage arrays
        states_array = np.zeros((self.N, 12))
        controls_array = np.zeros((self.N, 4))
        errors_array = np.zeros((self.N, 6))
        sliding_array = np.zeros((self.N, 3))
        
        if verbose:
            print("="*70)
            print("FIXED-STEP EULER SIMULATION OF HFTC")
            print("="*70)
            print(f"Time step: dt = {self.dt:.4f} s")
            print(f"Total steps: N = {self.N}")
            print("="*70)
        
        # Main simulation loop
        for i in range(self.N):
            t = t_array[i]
            states_array[i, :] = state
            
            # Compute control
            u, f_omega, u_attitude, d_hat = controller.compute_control(state, t, self.dt)
            controls_array[i, :] = u
            u1, u2, u3, u4 = u
            
            # Update FDO
            omega = np.array([state[7], state[9], state[11]])
            controller.update_fdo(omega, f_omega, u_attitude, self.dt, t)
            
            # Compute state derivatives
            state_deriv = self._compute_dynamics(state, u, t)
            
            # Euler integration
            state = state + self.dt * state_deriv
            
            # Angle wrapping
            state = self._wrap_angles(state)
            
            # Store sliding surface and errors
            if i < len(controller.sliding_history):
                sliding_array[i, :] = controller.sliding_history[i]
            if i < len(controller.error_history):
                errors_array[i, :] = controller.error_history[i]
            
            if verbose and i % 1000 == 0:
                print(f"Progress: {(i/self.N)*100:.1f}% (t = {t:.2f} s)")
        
        if verbose:
            print("Simulation completed!")
        
        # Get FDO history
        fdo_est = np.array(controller.fdo.d_est_history)
        fdo_true = np.array(controller.fdo.d_true_history)
        
        min_len = min(self.N, len(fdo_est), len(controller.time_history))
        
        return (t_array[:min_len], 
                states_array[:min_len, :], 
                controls_array[:min_len, :], 
                errors_array[:min_len, :],
                sliding_array[:min_len, :],
                fdo_est[:min_len, :],
                fdo_true[:min_len, :],
                controller)
    
    def _compute_dynamics(self, state, u, t):
        """Compute state derivatives from dynamics equations"""
        x, x_dot, y, y_dot, z, z_dot, phi, p, theta, q, psi, r = state
        u1, u2, u3, u4 = u
        
        # Get disturbances
        d = disturbances(t)
        d1, d2, d3, d4 = d
        
        # Trigonometric terms
        sin_phi = np.sin(phi)
        cos_phi = np.cos(phi)
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)
        sin_psi = np.sin(psi)
        cos_psi = np.cos(psi)
        
        # Position dynamics
        thrust_term = np.clip((u1 + d1) / self.params.M, -100.0, 100.0)
        
        x_ddot = (cos_phi * sin_theta * cos_psi + sin_phi * sin_psi) * thrust_term
        y_ddot = (cos_phi * sin_theta * sin_psi - sin_phi * cos_psi) * thrust_term
        z_ddot = -self.params.g + (cos_phi * cos_theta) * thrust_term
        
        # Angular accelerations
        omega_r = 0.0
        
        p_dot = self.params.a1 * q * r + self.params.a2 * q * omega_r + self.params.b1 * u2 + d2
        q_dot = self.params.a3 * p * r + self.params.a4 * p * omega_r + self.params.b2 * u3 + d3
        r_dot = self.params.a5 * p * q + self.params.b3 * u4 + d4
        
        # Euler angle derivatives
        if abs(cos_theta) < 0.01:
            phi_dot = p
            psi_dot = r / 0.01
        else:
            phi_dot = p + (q * sin_phi + r * cos_phi) * np.tan(theta)
            psi_dot = (q * sin_phi + r * cos_phi) / cos_theta
        
        theta_dot = q * cos_phi - r * sin_phi
        
        return np.array([
            x_dot, x_ddot,
            y_dot, y_ddot,
            z_dot, z_ddot,
            phi_dot, p_dot,
            theta_dot, q_dot,
            psi_dot, r_dot
        ])
    
    @staticmethod
    def _wrap_angles(state):
        """Wrap angles to [-π, π]"""
        for idx in [6, 8, 10]:  # phi, theta, psi indices
            if state[idx] > np.pi:
                state[idx] -= 2 * np.pi
            if state[idx] < -np.pi:
                state[idx] += 2 * np.pi
        return state
