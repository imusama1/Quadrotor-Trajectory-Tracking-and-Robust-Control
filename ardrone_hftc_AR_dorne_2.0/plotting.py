"""
Results Plotting for AR.Drone HFTC Controller
"""

import numpy as np
from datetime import datetime

try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


class ResultsPlotter:
    """Generate analysis plots from flight data."""
    
    def __init__(self, output_dir='/home/usama/Drone/Python Implementation/ardrone_hftc/results'):
        self.output_dir = output_dir
    
    def plot_results(self, log, controller=None):
        """
        Generate comprehensive results plot.
        
        Args:
            log: Dictionary with logged data
            controller: Optional controller reference for parameters
        """
        if not MATPLOTLIB_AVAILABLE:
            print("Matplotlib not available for plotting")
            return
        
        if len(log['time']) < 10:
            print("Not enough data to plot")
            return
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        t = np.array(log['time'])
        est_x = np.array(log['est_x']) * 100  # to cm
        est_y = np.array(log['est_y']) * 100
        des_x = np.array(log['des_x']) * 100
        des_y = np.array(log['des_y']) * 100
        err_x = np.array(log['err_x']) * 100
        err_y = np.array(log['err_y']) * 100
        
        fig = plt.figure(figsize=(16, 12))
        fig.suptitle('AR.Drone HFTC Circle Trajectory', fontsize=16, fontweight='bold')
        
        gs = fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)
        
        # XY Trajectory
        ax = fig.add_subplot(gs[0:2, 0:2])
        ax.plot(des_x, des_y, 'k--', lw=2, alpha=0.5, label='Desired')
        ax.plot(est_x, est_y, 'b-', lw=2, label='Actual')
        ax.scatter([0], [0], c='g', s=150, marker='o', zorder=10, label='Start')
        ax.scatter([est_x[-1]], [est_y[-1]], c='r', s=150, marker='X', zorder=10, label='End')
        ax.set_xlabel('X (cm)')
        ax.set_ylabel('Y (cm)')
        ax.set_title('XY Trajectory')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        # Tracking Error
        ax = fig.add_subplot(gs[0, 2])
        ax.plot(t, err_x, 'b-', label='X err')
        ax.plot(t, err_y, 'r-', label='Y err')
        ax.axhline(0, color='k', ls='--', alpha=0.3)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Error (cm)')
        ax.set_title('Tracking Error')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Commands
        ax = fig.add_subplot(gs[1, 2])
        ax.plot(t, log['cmd_pitch'], 'b-', label='Pitch')
        ax.plot(t, log['cmd_roll'], 'r-', label='Roll')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Command')
        ax.set_title('Control Commands')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Disturbance Observer
        ax = fig.add_subplot(gs[2, 0])
        ax.plot(t, log['d_hat_x'], 'b-', label='d̂_x')
        ax.plot(t, log['d_hat_y'], 'r-', label='d̂_y')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Estimate')
        ax.set_title('Disturbance Observer')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Virtual Errors
        ax = fig.add_subplot(gs[2, 1])
        ax.plot(t, log['z_x'], 'b-', label='z_x')
        ax.plot(t, log['z_y'], 'r-', label='z_y')
        ax.axhline(0, color='k', ls='--', alpha=0.3)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Virtual Error')
        ax.set_title('Backstepping z = v - α')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Statistics
        ax = fig.add_subplot(gs[2, 2])
        ax.axis('off')
        
        half = len(t) // 2
        rmse_x1 = np.sqrt(np.mean(err_x[:half]**2))
        rmse_y1 = np.sqrt(np.mean(err_y[:half]**2))
        rmse_x2 = np.sqrt(np.mean(err_x[half:]**2))
        rmse_y2 = np.sqrt(np.mean(err_y[half:]**2))
        
        improvement = ((rmse_x1+rmse_y1)-(rmse_x2+rmse_y2))/(rmse_x1+rmse_y1)*100 if (rmse_x1+rmse_y1) > 0 else 0
        
        stats = f"""
HFTC PERFORMANCE
────────────────
Circle 1:
  RMSE X: {rmse_x1:.1f} cm
  RMSE Y: {rmse_y1:.1f} cm

Circle 2:
  RMSE X: {rmse_x2:.1f} cm
  RMSE Y: {rmse_y2:.1f} cm

Improvement: {improvement:.0f}%
(Observer convergence)

Final d̂: ({log['d_hat_x'][-1]:.2f}, {log['d_hat_y'][-1]:.2f})
"""
        ax.text(0.05, 0.95, stats, transform=ax.transAxes, fontsize=10,
                va='top', fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.3))
        
        plt.tight_layout()
        
        import os
        os.makedirs(self.output_dir, exist_ok=True)
        filename = f'{self.output_dir}/hftc_{timestamp}.png'
        plt.savefig(filename, dpi=150)
        print(f"\nSaved: {filename}")
        plt.show()
        
        return filename
