#!/usr/bin/env python3
"""
AR.Drone 2.0 HFTC Controller - Main Entry Point

Usage:
    python -m ardrone_hftc.main
    OR
    python ardrone_hftc/main.py
"""

import sys
import os

# Add parent to path if running directly
if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ardrone_hftc.controller import HFTCController
from ardrone_hftc.plotting import ResultsPlotter


def main():
    """Main entry point."""
    print("="*60)
    print("AR.DRONE 2.0 HFTC CONTROLLER")
    print("Backstepping + Sliding Mode + Disturbance Observer")
    print("="*60)
    
    ctrl = HFTCController()
    plotter = ResultsPlotter()
    
    if not ctrl.connect():
        print("Failed to connect to drone")
        return 1
    
    try:
        input("\nPress ENTER to takeoff...")
        
        if not ctrl.takeoff():
            print("Takeoff failed")
            return 1
        
        ctrl.run_trajectory(num_circles=2)
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    finally:
        ctrl.land()
        plotter.plot_results(ctrl.log, ctrl)
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
