"""Data logging for ARSMC controller."""

import os
from datetime import datetime
from typing import Dict, Any, Optional, Callable, List

import numpy as np

from .config import LoggingParams


class FlightDataLogger:
    """Logs flight data for offline analysis and plotting."""

    LOGGED_FIELDS = [
        'time', 'x', 'y', 'z',
        'x_d', 'y_d', 'z_d',
        'vx', 'vy', 'vz',
        'err_x', 'err_y', 'err_z',
        'd1_hat_z', 'ez_int', 's_z',
        'disturbance_active', 'loop'
    ]

    def __init__(self, params: LoggingParams, logger: Optional[Callable] = None):
        self.params = params
        self.logger = logger
        self.data: Dict[str, List[float]] = {field: [] for field in self.LOGGED_FIELDS}
        self.active = False
        self.data_saved = False

    def start(self):
        """Start logging."""
        self.active = True

    def stop(self):
        """Stop logging."""
        self.active = False

    def reset(self):
        """Reset all logged data."""
        self.data = {field: [] for field in self.LOGGED_FIELDS}
        self.data_saved = False

    def log(self, t: float, x: float, y: float, z: float,
            x_d: float, y_d: float, z_d: float,
            vx: float, vy: float, vz: float,
            d1_hat_z: float, ez_int: float, s_z: float,
            disturbance_active: bool, current_loop: int):
        """Log a single data point."""
        if not self.active:
            return

        self.data['time'].append(t)
        self.data['x'].append(x)
        self.data['y'].append(y)
        self.data['z'].append(z)
        self.data['x_d'].append(x_d)
        self.data['y_d'].append(y_d)
        self.data['z_d'].append(z_d)
        self.data['vx'].append(vx)
        self.data['vy'].append(vy)
        self.data['vz'].append(vz)
        self.data['err_x'].append(x - x_d)
        self.data['err_y'].append(y - y_d)
        self.data['err_z'].append(z - z_d)
        self.data['d1_hat_z'].append(d1_hat_z)
        self.data['ez_int'].append(ez_int)
        self.data['s_z'].append(s_z)
        self.data['disturbance_active'].append(1.0 if disturbance_active else 0.0)
        self.data['loop'].append(current_loop)

    def save(self, controller_params: Dict[str, Any]) -> Optional[str]:
        """
        Save logged data to disk.
        
        Args:
            controller_params: Dictionary of controller parameters to save
            
        Returns:
            Output directory path if successful, None otherwise
        """
        if self.data_saved:
            return None
            
        if len(self.data['time']) < self.params.min_samples_to_save:
            if self.logger:
                self.logger(f"Not enough data to save ({len(self.data['time'])} samples)")
            return None

        self.data_saved = True
        
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_dir = f'{self.params.output_dir_base}/PX4_SITL_Results_{timestamp}'
            os.makedirs(output_dir, exist_ok=True)
            
            # Save flight data
            np.savez(
                f'{output_dir}/flight_data.npz',
                **{k: np.array(v) for k, v in self.data.items()}
            )
            
            # Save controller parameters
            np.savez(f'{output_dir}/params.npz', **controller_params)
            
            if self.logger:
                self.logger(f"\n{'='*60}")
                self.logger(f"💾 DATA SAVED TO: {output_dir}")
                self.logger(f"{'='*60}")
                self.logger(f"To generate plots, run:")
                self.logger(f"  python3 {self.params.output_dir_base}/plot_px4_results.py {output_dir}")
                self.logger(f"{'='*60}")
                
            return output_dir
            
        except Exception as e:
            if self.logger:
                self.logger(f"Data saving failed: {e}")
            self.data_saved = False
            return None

    @property
    def sample_count(self) -> int:
        """Get number of logged samples."""
        return len(self.data['time'])
