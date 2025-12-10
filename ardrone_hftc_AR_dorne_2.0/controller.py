"""
Main HFTC Controller for AR.Drone 2.0
"""

import time
from .config import CommandConfig, ControlLoopConfig, TrajectoryConfig
from .drone_interface import DroneInterface
from .trajectory import TrajectoryGenerator
from .observers import DisturbanceObserver
from .controllers import BacksteppingController, SlidingModeController


class HFTCController:
    """
    HFTC-Inspired Controller for AR.Drone 2.0
    
    Combines:
    - Backstepping position control (XY)
    - Integral sliding mode (Z)
    - Adaptive disturbance observer
    """
    
    def __init__(self):
        # Components
        self.drone = DroneInterface()
        self.trajectory = TrajectoryGenerator()
        self.observer = DisturbanceObserver()
        self.backstepping = BacksteppingController()
        self.sliding_mode = SlidingModeController()
        
        # State estimation (integrated from velocity)
        self.est_x = 0.0
        self.est_y = 0.0
        self.est_z = 1.0
        
        # Command scaling
        self.cmd_scale_xy = CommandConfig.SCALE_XY
        self.cmd_scale_z = CommandConfig.SCALE_Z
        self.max_cmd = CommandConfig.MAX_CMD
        self.max_yaw = CommandConfig.MAX_YAW
        
        # Data logging
        self.log = {
            'time': [], 'est_x': [], 'est_y': [], 'est_z': [],
            'des_x': [], 'des_y': [], 'des_z': [],
            'err_x': [], 'err_y': [], 'err_z': [],
            'cmd_pitch': [], 'cmd_roll': [], 'cmd_gaz': [],
            'd_hat_x': [], 'd_hat_y': [], 'd_hat_z': [],
            'z_x': [], 'z_y': [], 's_z': [],
            'vx': [], 'vy': [], 'yaw': []
        }
    
    def connect(self):
        """Connect to drone."""
        return self.drone.connect()
    
    def takeoff(self):
        """Execute takeoff."""
        success = self.drone.takeoff()
        if success:
            _, _, _, alt, _ = self.drone.read_state()
            self.trajectory.hover_height = alt
        return success
    
    def land(self):
        """Execute landing."""
        self.drone.land()
    
    def reset(self):
        """Reset controller states."""
        self.est_x = 0.0
        self.est_y = 0.0
        self.observer.reset()
        self.sliding_mode.reset()
        self.log = {k: [] for k in self.log}
    
    def _scale_command(self, u_x, u_y, u_z, yaw_rate=0):
        """Scale control outputs to drone commands."""
        cmd_pitch = max(-self.max_cmd, min(self.max_cmd, u_x * self.cmd_scale_xy))
        cmd_roll = max(-self.max_cmd, min(self.max_cmd, u_y * self.cmd_scale_xy))
        cmd_gaz = max(-self.max_cmd, min(self.max_cmd, u_z * self.cmd_scale_z))
        cmd_yaw = max(-self.max_yaw, min(self.max_yaw, yaw_rate))
        
        return cmd_roll, cmd_pitch, cmd_gaz, cmd_yaw
    
    def control_step(self, t, dt):
        """
        Execute one control step.
        
        Args:
            t: Current time
            dt: Time step
            
        Returns:
            bool: True if successful
        """
        # Read state
        vx, vy, vz, alt, yaw = self.drone.read_state()
        
        # Integrate position (causes drift!)
        self.est_x += vx * dt
        self.est_y += vy * dt
        self.est_z = alt
        
        # Get reference
        ref = self.trajectory.get_circle_reference(t)
        
        # Compute errors
        ex = self.est_x - ref['x_d']
        ey = self.est_y - ref['y_d']
        ez = self.est_z - ref['z_d']
        
        # Backstepping XY control
        d_hat_x, d_hat_y, d_hat_z = self.observer.get_estimates()
        u_x, u_y, z_x, z_y = self.backstepping.compute(
            ex, ey, vx, vy, ref['vx_d'], ref['vy_d'],
            ref['ax_d'], ref['ay_d'], d_hat_x, d_hat_y
        )
        
        # Update XY observer
        self.observer.update_xy(z_x, z_y, dt)
        
        # Sliding mode Z control
        u_z, s_z = self.sliding_mode.compute(ez, vz, d_hat_z, dt)
        
        # Update Z observer
        self.observer.update_z(s_z, dt)
        
        # Yaw stabilization
        yaw_rate = -0.015 * yaw
        
        # Scale and send command
        cmd_roll, cmd_pitch, cmd_gaz, cmd_yaw = self._scale_command(u_x, u_y, u_z, yaw_rate)
        self.drone.send_command(cmd_roll, cmd_pitch, cmd_gaz, cmd_yaw)
        
        # Log data
        self._log_step(t, ref, ex, ey, ez, cmd_pitch, cmd_roll, cmd_gaz,
                       z_x, z_y, s_z, vx, vy, yaw)
        
        return True
    
    def _log_step(self, t, ref, ex, ey, ez, cmd_p, cmd_r, cmd_g, z_x, z_y, s_z, vx, vy, yaw):
        """Log data for one step."""
        d_x, d_y, d_z = self.observer.get_estimates()
        
        self.log['time'].append(t)
        self.log['est_x'].append(self.est_x)
        self.log['est_y'].append(self.est_y)
        self.log['est_z'].append(self.est_z)
        self.log['des_x'].append(ref['x_d'])
        self.log['des_y'].append(ref['y_d'])
        self.log['des_z'].append(ref['z_d'])
        self.log['err_x'].append(ex)
        self.log['err_y'].append(ey)
        self.log['err_z'].append(ez)
        self.log['cmd_pitch'].append(cmd_p)
        self.log['cmd_roll'].append(cmd_r)
        self.log['cmd_gaz'].append(cmd_g)
        self.log['d_hat_x'].append(d_x)
        self.log['d_hat_y'].append(d_y)
        self.log['d_hat_z'].append(d_z)
        self.log['z_x'].append(z_x)
        self.log['z_y'].append(z_y)
        self.log['s_z'].append(s_z)
        self.log['vx'].append(vx)
        self.log['vy'].append(vy)
        self.log['yaw'].append(yaw)
    
    def run_trajectory(self, num_circles=None):
        """
        Execute circle trajectory.
        
        Args:
            num_circles: Number of circles (default from config)
        """
        num_circles = num_circles or TrajectoryConfig.NUM_CIRCLES
        duration = num_circles * self.trajectory.period
        dt = ControlLoopConfig.DT
        
        print(f"\n{'='*50}")
        print("HFTC TRAJECTORY EXECUTION")
        print(f"{'='*50}")
        print(f"Circles: {num_circles}, Radius: {self.trajectory.radius*100:.0f}cm")
        print(f"Backstepping: c={self.backstepping.c_x}, k={self.backstepping.k_x}")
        print(f"Observer: γ={self.observer.gamma_xy}")
        print(f"{'='*50}\n")
        
        self.reset()
        
        start = time.time()
        last_t = start
        
        try:
            while (time.time() - start) < duration:
                t = time.time() - start
                curr_dt = max(ControlLoopConfig.MIN_DT, 
                             min(ControlLoopConfig.MAX_DT, time.time() - last_t))
                last_t = time.time()
                
                self.control_step(t, curr_dt)
                
                # Status output
                if int(t * 2) != int((t - curr_dt) * 2):
                    circle_num = int(t / self.trajectory.period) + 1
                    d_x, d_y, _ = self.observer.get_estimates()
                    print(f"Circle {circle_num} | t={t:5.1f}s | "
                          f"pos=({self.est_x*100:+5.0f},{self.est_y*100:+5.0f})cm | "
                          f"err=({self.log['err_x'][-1]*100:+4.0f},{self.log['err_y'][-1]*100:+4.0f})cm | "
                          f"d̂=({d_x:+.2f},{d_y:+.2f})")
                
                time.sleep(dt)
                
        except KeyboardInterrupt:
            print("\n>>> Interrupted")
        
        finally:
            self.drone.hover()
            time.sleep(0.5)
        
        print("\nTrajectory complete!")
