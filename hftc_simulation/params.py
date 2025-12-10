"""
Quadrotor and Controller Parameters
Table 1 from the paper - EXACT VALUES
"""

import numpy as np


class QuadrotorParams:
    """Physical parameters of the quadrotor from Table 1"""
    
    M = 0.5  # Mass (kg)
    g = 9.81  # Gravity (m/s^2)
    Ix = 6.114e-3  # Inertia about x-axis (kg·m^2)
    Iy = 6.114e-3  # Inertia about y-axis (kg·m^2)
    Iz = 1.223e-2  # Inertia about z-axis (kg·m^2)
    Jr = 1.55e-5  # Rotor inertia (kg·m^2)
    l = 0.5  # Arm length (m)
    b = 3.16e-5  # Thrust coefficient
    d = 7.32e-7  # Drag coefficient
    
    # Derived parameters
    a1 = (Iy - Iz) / Ix
    a2 = Jr / Ix
    a3 = (Iz - Ix) / Iy
    a4 = -Jr / Iy
    a5 = (Ix - Iy) / Iz
    b1 = l / Ix
    b2 = l / Iy
    b3 = 1.0 / Iz


class ControlParams:
    """Tuned control parameters for stable tracking"""
    
    # AISM Controller for Altitude
    cz = 4.0       # Position error gain
    kz = 2.0       # Integral gain
    hz = 6.0       # Sliding surface gain
    phi1 = 1.5     # Adaptation gain
    rho = 0.05     # Tanh smoothing
    
    # Backstepping Controller for Position
    cx = 1.5       # X position gain
    kx = 2.5       # X velocity gain
    cy = 1.5       # Y position gain
    ky = 2.5       # Y velocity gain
    
    # FDO-based NTSM Controller for Attitude
    lambda1 = 5.0   # FDO gain 1
    lambda2 = 2.0   # FDO gain 2
    lambda3 = 3.0   # FDO gain 3
    L = 50.0        # Lipschitz constant
    beta = 2.5      # NTSM beta
    m = 5           # NTSM m
    n = 3           # NTSM n
    K = np.diag([40.0, 40.0, 80.0])  # Attitude control gains
