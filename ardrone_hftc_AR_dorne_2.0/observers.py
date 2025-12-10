"""
Disturbance Observer for AR.Drone HFTC Controller
"""

import math
from .config import ObserverConfig


class DisturbanceObserver:
    """
    Simplified Finite-Time Disturbance Observer.
    
    Paper uses full FDO (Theorem 1). We use adaptive law:
        d_hat_dot = γ·|s|·tanh(s/ε)
    
    This captures unmodeled dynamics and improves over time.
    """
    
    def __init__(self):
        self.d_hat_x = 0.0
        self.d_hat_y = 0.0
        self.d_hat_z = 0.0
        
        self.gamma_xy = ObserverConfig.GAMMA_XY
        self.gamma_z = ObserverConfig.GAMMA_Z
        self.d_max = ObserverConfig.D_MAX
        self.epsilon = ObserverConfig.EPSILON
        
        # History for analysis
        self.history = {
            'd_hat_x': [],
            'd_hat_y': [],
            'd_hat_z': []
        }
    
    def update_xy(self, z_x, z_y, dt):
        """
        Update XY disturbance estimates.
        
        Args:
            z_x: X virtual control error (backstepping)
            z_y: Y virtual control error (backstepping)
            dt: Time step
            
        Returns:
            tuple: (d_hat_x, d_hat_y)
        """
        # Adaptive update law
        self.d_hat_x += self.gamma_xy * abs(z_x) * math.tanh(z_x / self.epsilon) * dt
        self.d_hat_y += self.gamma_xy * abs(z_y) * math.tanh(z_y / self.epsilon) * dt
        
        # Saturation
        self.d_hat_x = max(-self.d_max, min(self.d_max, self.d_hat_x))
        self.d_hat_y = max(-self.d_max, min(self.d_max, self.d_hat_y))
        
        # Log
        self.history['d_hat_x'].append(self.d_hat_x)
        self.history['d_hat_y'].append(self.d_hat_y)
        
        return self.d_hat_x, self.d_hat_y
    
    def update_z(self, s_z, dt):
        """
        Update Z disturbance estimate.
        
        Args:
            s_z: Z sliding surface
            dt: Time step
            
        Returns:
            float: d_hat_z
        """
        # Only update if sliding surface is significant
        if abs(s_z) > 0.05:
            self.d_hat_z += self.gamma_z * abs(s_z) * math.tanh(s_z / self.epsilon) * dt
            self.d_hat_z = max(-self.d_max, min(self.d_max, self.d_hat_z))
        
        self.history['d_hat_z'].append(self.d_hat_z)
        
        return self.d_hat_z
    
    def reset(self):
        """Reset observer states."""
        self.d_hat_x = 0.0
        self.d_hat_y = 0.0
        self.d_hat_z = 0.0
        self.history = {'d_hat_x': [], 'd_hat_y': [], 'd_hat_z': []}
    
    def get_estimates(self):
        """Get current disturbance estimates."""
        return self.d_hat_x, self.d_hat_y, self.d_hat_z
