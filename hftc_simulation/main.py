#!/usr/bin/env python3
"""
Main entry point for HFTC Simulation
Run this file to execute the complete simulation and generate all plots.

Usage:
    python -m hftc_simulation.main
    OR
    python /home/usama/Drone/Python Implementation/hftc_simulation/main.py
"""

import os
import sys

# Add parent directory to path if running directly
if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from hftc_simulation.simulation import Simulator
from hftc_simulation.plotting import PlotGenerator
from hftc_simulation.trajectory import desired_trajectory


def main():
    """Main function to run HFTC simulation"""
    print("="*70)
    print("HYBRID FINITE-TIME TRAJECTORY TRACKING CONTROL OF A QUADROTOR")
    print("N. Wang et al. - ISA Transactions 90 (2019) 278-286")
    print("="*70)
    
    # Create output directory
    output_dir = os.path.join(os.path.dirname(__file__), '..', 'newplots')
    os.makedirs(output_dir, exist_ok=True)
    
    # Run simulation
    print("\n[1/2] Running simulation...")
    sim = Simulator(T=20.0, dt=0.002)
    results = sim.run(verbose=True)
    
    t, states, controls, errors, sliding, fdo_est, fdo_true, controller = results
    
    # Generate plots
    print("\n[2/2] Generating plots...")
    plotter = PlotGenerator(output_dir=output_dir)
    plotter.generate_all(t, states, controls, errors, sliding, fdo_est, fdo_true, controller)
    
    print("\n" + "="*70)
    print("SIMULATION COMPLETED SUCCESSFULLY!")
    print(f"Results saved to: {os.path.abspath(output_dir)}")
    print("="*70)


if __name__ == "__main__":
    main()
