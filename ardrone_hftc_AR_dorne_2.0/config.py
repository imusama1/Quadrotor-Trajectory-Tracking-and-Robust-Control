"""
Configuration Parameters for AR.Drone HFTC Controller
"""

import math


class TrajectoryConfig:
    """Trajectory parameters"""
    RADIUS = 0.5            # Circle radius (m)
    PERIOD = 10.0           # Time per circle (s)
    OMEGA = 2 * math.pi / PERIOD
    HOVER_HEIGHT = 1.0      # Default hover height (m)
    NUM_CIRCLES = 2         # Default number of circles


class BacksteppingConfig:
    """
    Backstepping gains (Paper Eq. 15-18, adapted)
    
    Lyapunov: V = 0.5·e² + 0.5·z²
    V_dot = -c·e² - k·z² < 0 (asymptotically stable)
    """
    C_X = 1.5   # X position error gain
    C_Y = 1.5   # Y position error gain
    K_X = 2.0   # X virtual control error gain
    K_Y = 2.0   # Y virtual control error gain


class SlidingModeConfig:
    """
    Integral Sliding Mode gains (Paper Eq. 23-25)
    
    Surface: s = c·e + ė + k·∫e
    Control: u = -c·ė - k·e - h·s - d_hat·tanh(s/ε)
    """
    C_Z = 1.5   # Altitude error gain
    K_Z = 0.8   # Integral gain
    H_Z = 3.0   # Sliding surface gain
    VARRHO = 0.1  # tanh smoothing parameter


class ObserverConfig:
    """
    Disturbance Observer parameters (Simplified FDO)
    
    Adaptive law: d_hat_dot = γ·|s|·tanh(s/ε)
    """
    GAMMA_XY = 0.5  # XY adaptation gain
    GAMMA_Z = 0.3   # Z adaptation gain
    D_MAX = 0.5     # Maximum disturbance estimate
    EPSILON = 0.1   # tanh smoothing


class CommandConfig:
    """
    AR.Drone command scaling
    
    Maps control output to API range [-1, 1]
    """
    SCALE_XY = 0.12     # XY command scale
    SCALE_Z = 0.08      # Z command scale
    MAX_CMD = 0.18      # Safety limit for indoor
    MAX_YAW = 0.1       # Max yaw rate command


class ControlLoopConfig:
    """Control loop timing"""
    DT = 0.05           # Control period (20 Hz)
    MIN_DT = 0.01       # Minimum dt
    MAX_DT = 0.1        # Maximum dt
    INT_MAX = 1.0       # Anti-windup limit
