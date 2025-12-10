"""
Desired Trajectory and Disturbance Functions
Equation (61) from the paper
"""

import numpy as np


def desired_trajectory(t):
    """
    Desired trajectory from equation (61) in the paper
    
    Returns:
        tuple: (xd, xd_dot, xd_ddot, yd, yd_dot, yd_ddot, 
                zd, zd_dot, zd_ddot, psid, psid_dot, psid_ddot)
    """
    zd = 0.5 * t + 5
    xd = np.cos(t)
    yd = np.sin(t) + 2
    psid = np.sin(0.5 * t) + 2
    
    # Derivatives
    zd_dot = 0.5
    xd_dot = -np.sin(t)
    yd_dot = np.cos(t)
    psid_dot = 0.5 * np.cos(0.5 * t)
    
    # Second derivatives
    zd_ddot = 0
    xd_ddot = -np.cos(t)
    yd_ddot = -np.sin(t)
    psid_ddot = -0.25 * np.sin(0.5 * t)
    
    return (xd, xd_dot, xd_ddot, yd, yd_dot, yd_ddot, 
            zd, zd_dot, zd_ddot, psid, psid_dot, psid_ddot)


def disturbances(t):
    """
    External disturbances as specified in the paper
    d_i(t) = 0.5*cos(t)
    
    Returns:
        np.array: [d1, d2, d3, d4] disturbances for altitude, roll, pitch, yaw
    """
    d1 = 0.5 * np.cos(t)  # Altitude disturbance
    d2 = 0.5 * np.cos(t)  # Roll disturbance
    d3 = 0.5 * np.cos(t)  # Pitch disturbance
    d4 = 0.5 * np.cos(t)  # Yaw disturbance
    return np.array([d1, d2, d3, d4])
