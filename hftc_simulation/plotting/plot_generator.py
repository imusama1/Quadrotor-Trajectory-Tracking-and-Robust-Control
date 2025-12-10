"""
Main Plot Generator Class
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from ..trajectory import desired_trajectory
from . import figures


class PlotGenerator:
    """Generate all plots and figures for the simulation results"""
    
    def __init__(self, output_dir='newplots'):
        """
        Initialize plot generator
        
        Args:
            output_dir: Directory to save plots
        """
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
    
    def generate_all(self, t, states, controls, errors, sliding, fdo_est, fdo_true, controller):
        """
        Generate all plots
        
        Args:
            t: Time array
            states: State array
            controls: Control array
            errors: Error array
            sliding: Sliding surface array
            fdo_est: FDO estimates
            fdo_true: True disturbances
            controller: Controller object
        """
        # Get desired trajectory
        desired = np.array([desired_trajectory(ti) for ti in t])
        
        print("\n" + "="*70)
        print("GENERATING PLOTS AND FIGURES...")
        print("="*70)
        
        # Conceptual figures
        print("\n[1/18] Control Architecture Hierarchy...")
        figures.create_control_architecture_hierarchy(self.output_dir)
        
        print("[2/18] HFTC Block Diagram...")
        figures.create_hftc_block_diagram(self.output_dir)
        
        print("[3/18] Quadrotor Reference Frames...")
        figures.create_quadrotor_frames_diagram(self.output_dir)
        
        print("[4/18] Backstepping Structure...")
        figures.create_backstepping_structure(self.output_dir)
        
        print("[5/18] System Architecture...")
        figures.create_system_architecture(self.output_dir)
        
        print("[6/18] ROS2 Pipeline...")
        figures.create_ros2_pipeline(self.output_dir)
        
        # Simulation results
        print("\n[7/18] XY Trajectory...")
        figures.plot_xy_plane_trajectory(states, desired, self.output_dir)
        
        print("[8/18] Altitude Tracking...")
        figures.plot_altitude_tracking(t, states, desired, self.output_dir)
        
        print("[9/18] Position Errors...")
        figures.plot_position_errors_combined(t, errors, self.output_dir)
        
        print("[10/18] Disturbance Response...")
        figures.create_disturbance_response_plot(t, states, desired, errors, self.output_dir)
        
        print("[11/18] Control Inputs...")
        figures.plot_control_inputs(t, controls, self.output_dir)
        
        print("[12/18] Position Tracking...")
        figures.plot_position_tracking(t, states, desired, self.output_dir)
        
        print("[13/18] Yaw Tracking...")
        figures.plot_yaw_tracking(t, states, desired, self.output_dir)
        
        print("[14/18] Yaw Error...")
        figures.plot_yaw_error(t, errors, self.output_dir)
        
        print("[15/18] FDO Performance...")
        figures.plot_fdo_performance(t, fdo_est, fdo_true, self.output_dir)
        
        print("[16/18] FDO Errors...")
        figures.plot_fdo_errors(t, fdo_est, fdo_true, self.output_dir)
        
        print("[17/18] Sliding Surfaces...")
        figures.plot_sliding_surfaces(t, sliding, self.output_dir)
        
        print("[18/18] 3D Trajectory and Animation...")
        figures.plot_3d_trajectory_static(states, desired, self.output_dir)
        figures.animate_3d_trajectory(t, states, desired, self.output_dir)
        
        # Generate tables
        print("\n" + "="*70)
        print("GENERATING TABLES...")
        print("="*70)
        tables = figures.generate_all_tables(t, errors, controls, fdo_est, fdo_true, self.output_dir)
        print(tables)
        
        # Print metrics
        figures.print_performance_metrics(t, errors, controls, fdo_est, fdo_true)
        
        self._print_summary()
    
    def _print_summary(self):
        """Print summary of generated files"""
        print("\n" + "="*70)
        print("ALL PLOTS AND TABLES GENERATED SUCCESSFULLY!")
        print("="*70)
        print(f"\nFiles saved to: {self.output_dir}/")
        print("\nConceptual Figures:")
        print("  - fig_2_1_control_architecture_hierarchy.png")
        print("  - fig_4_1_hftc_block_diagram.png")
        print("  - fig_3_1_quadrotor_reference_frames.png")
        print("  - fig_4_2_backstepping_structure.png")
        print("  - fig_5_1_system_architecture.png")
        print("  - fig_5_2_ros2_control_pipeline.png")
        print("\nSimulation Results:")
        print("  - fig_6_1_xy_trajectory_tracking.png")
        print("  - fig_6_2_altitude_tracking.png")
        print("  - fig_6_3_xyz_tracking_errors.png")
        print("  - fig_6_4_disturbance_response.png")
        print("  - fig_6_5_control_effort.png")
        print("  - ... and more")
        print("="*70)
