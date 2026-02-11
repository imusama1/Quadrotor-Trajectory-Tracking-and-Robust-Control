"""Occupancy Grid Mapping for Quadrotor Navigation.

This module provides 2D/3D occupancy grid mapping functionality for
obstacle detection and collision-free trajectory planning.
"""

import numpy as np
from typing import Tuple, List, Optional
from dataclasses import dataclass


@dataclass
class GridConfig:
    """Configuration for occupancy grid."""
    # Grid dimensions (meters)
    x_min: float = -10.0
    x_max: float = 10.0
    y_min: float = -10.0
    y_max: float = 10.0
    z_min: float = -5.0
    z_max: float = 5.0
    
    # Grid resolution (meters per cell)
    resolution: float = 0.1
    
    # Occupancy thresholds
    occupied_threshold: float = 0.65
    free_threshold: float = 0.35
    
    # Safety margin around obstacles (meters)
    safety_margin: float = 0.5


class OccupancyGridMapper:
    """2D/3D Occupancy Grid for environment mapping and collision checking."""
    
    def __init__(self, config: Optional[GridConfig] = None, use_3d: bool = False):
        """Initialize occupancy grid mapper.
        
        Args:
            config: Grid configuration parameters
            use_3d: If True, use 3D grid; otherwise use 2D (default)
        """
        self.config = config if config is not None else GridConfig()
        self.use_3d = use_3d
        
        # Calculate grid dimensions
        self.nx = int((self.config.x_max - self.config.x_min) / self.config.resolution)
        self.ny = int((self.config.y_max - self.config.y_min) / self.config.resolution)
        self.nz = int((self.config.z_max - self.config.z_min) / self.config.resolution) if use_3d else 1
        
        # Initialize grid with unknown (0.5 probability)
        if use_3d:
            self.grid = np.ones((self.nx, self.ny, self.nz)) * 0.5
        else:
            self.grid = np.ones((self.nx, self.ny)) * 0.5
        
        # Track update count for each cell
        self.update_count = np.zeros_like(self.grid, dtype=np.int32)
    
    def world_to_grid(self, x: float, y: float, z: float = 0.0) -> Tuple[int, int, int]:
        """Convert world coordinates to grid indices.
        
        Args:
            x, y, z: World coordinates in meters
            
        Returns:
            Tuple of (ix, iy, iz) grid indices
        """
        ix = int((x - self.config.x_min) / self.config.resolution)
        iy = int((y - self.config.y_min) / self.config.resolution)
        iz = int((z - self.config.z_min) / self.config.resolution) if self.use_3d else 0
        
        # Clamp to grid bounds
        ix = np.clip(ix, 0, self.nx - 1)
        iy = np.clip(iy, 0, self.ny - 1)
        iz = np.clip(iz, 0, self.nz - 1) if self.use_3d else 0
        
        return ix, iy, iz
    
    def grid_to_world(self, ix: int, iy: int, iz: int = 0) -> Tuple[float, float, float]:
        """Convert grid indices to world coordinates.
        
        Args:
            ix, iy, iz: Grid indices
            
        Returns:
            Tuple of (x, y, z) world coordinates in meters
        """
        x = self.config.x_min + (ix + 0.5) * self.config.resolution
        y = self.config.y_min + (iy + 0.5) * self.config.resolution
        z = self.config.z_min + (iz + 0.5) * self.config.resolution if self.use_3d else 0.0
        
        return x, y, z
    
    def is_valid_index(self, ix: int, iy: int, iz: int = 0) -> bool:
        """Check if grid indices are valid.
        
        Args:
            ix, iy, iz: Grid indices
            
        Returns:
            True if indices are within grid bounds
        """
        if self.use_3d:
            return (0 <= ix < self.nx and 
                    0 <= iy < self.ny and 
                    0 <= iz < self.nz)
        else:
            return 0 <= ix < self.nx and 0 <= iy < self.ny
    
    def update_cell(self, x: float, y: float, z: float, occupied: bool):
        """Update a single cell's occupancy probability.
        
        Args:
            x, y, z: World coordinates in meters
            occupied: True if cell is occupied, False if free
        """
        ix, iy, iz = self.world_to_grid(x, y, z)
        
        if not self.is_valid_index(ix, iy, iz):
            return
        
        # Bayesian update with log-odds
        prior_odds = self.grid[ix, iy, iz] / (1.0 - self.grid[ix, iy, iz] + 1e-10)
        
        # Sensor model (simple)
        if occupied:
            likelihood = 0.9  # P(measurement | occupied)
        else:
            likelihood = 0.1  # P(measurement | free)
        
        # Update odds
        posterior_odds = prior_odds * (likelihood / (1.0 - likelihood + 1e-10))
        
        # Convert back to probability
        if self.use_3d:
            self.grid[ix, iy, iz] = posterior_odds / (1.0 + posterior_odds)
            self.update_count[ix, iy, iz] += 1
        else:
            self.grid[ix, iy] = posterior_odds / (1.0 + posterior_odds)
            self.update_count[ix, iy] += 1
    
    def update_from_point_cloud(self, points: np.ndarray, sensor_pos: np.ndarray):
        """Update grid from point cloud data.
        
        Args:
            points: Nx3 array of point cloud coordinates [x, y, z]
            sensor_pos: Sensor position [x, y, z]
        """
        for point in points:
            # Mark endpoint as occupied
            self.update_cell(point[0], point[1], point[2] if self.use_3d else 0.0, occupied=True)
            
            # Mark ray from sensor to point as free (simple ray tracing)
            self._update_ray(sensor_pos, point)
    
    def _update_ray(self, start: np.ndarray, end: np.ndarray):
        """Update cells along a ray as free (simplified ray tracing).
        
        Args:
            start: Ray start position [x, y, z]
            end: Ray end position [x, y, z]
        """
        # Bresenham-like algorithm for 3D
        num_steps = int(np.linalg.norm(end - start) / (self.config.resolution * 0.5))
        
        if num_steps < 2:
            return
        
        for i in range(num_steps - 1):  # Don't mark endpoint as free
            alpha = i / num_steps
            point = start + alpha * (end - start)
            self.update_cell(point[0], point[1], point[2] if self.use_3d else 0.0, occupied=False)
    
    def is_occupied(self, x: float, y: float, z: float = 0.0) -> bool:
        """Check if a cell is occupied.
        
        Args:
            x, y, z: World coordinates in meters
            
        Returns:
            True if cell is occupied
        """
        ix, iy, iz = self.world_to_grid(x, y, z)
        
        if not self.is_valid_index(ix, iy, iz):
            return True  # Treat out-of-bounds as occupied
        
        if self.use_3d:
            return self.grid[ix, iy, iz] > self.config.occupied_threshold
        else:
            return self.grid[ix, iy] > self.config.occupied_threshold
    
    def is_free(self, x: float, y: float, z: float = 0.0) -> bool:
        """Check if a cell is free.
        
        Args:
            x, y, z: World coordinates in meters
            
        Returns:
            True if cell is free
        """
        ix, iy, iz = self.world_to_grid(x, y, z)
        
        if not self.is_valid_index(ix, iy, iz):
            return False  # Treat out-of-bounds as not free
        
        if self.use_3d:
            return self.grid[ix, iy, iz] < self.config.free_threshold
        else:
            return self.grid[ix, iy] < self.config.free_threshold
    
    def check_trajectory_collision(self, waypoints: np.ndarray) -> Tuple[bool, Optional[int]]:
        """Check if a trajectory collides with obstacles.
        
        Args:
            waypoints: Nx3 array of waypoint coordinates [x, y, z]
            
        Returns:
            Tuple of (has_collision, collision_index)
            collision_index is the first waypoint that collides, or None if no collision
        """
        for i, waypoint in enumerate(waypoints):
            # Check waypoint and safety margin around it
            if self._check_point_with_margin(waypoint[0], waypoint[1], 
                                            waypoint[2] if self.use_3d else 0.0):
                return True, i
        
        return False, None
    
    def _check_point_with_margin(self, x: float, y: float, z: float) -> bool:
        """Check if point or its safety margin collides with obstacles.
        
        Args:
            x, y, z: World coordinates in meters
            
        Returns:
            True if collision detected
        """
        # Check the point itself
        if self.is_occupied(x, y, z):
            return True
        
        # Check safety margin around the point
        margin_steps = int(self.config.safety_margin / self.config.resolution)
        
        for dx in range(-margin_steps, margin_steps + 1):
            for dy in range(-margin_steps, margin_steps + 1):
                if self.use_3d:
                    for dz in range(-margin_steps, margin_steps + 1):
                        test_x = x + dx * self.config.resolution
                        test_y = y + dy * self.config.resolution
                        test_z = z + dz * self.config.resolution
                        
                        if self.is_occupied(test_x, test_y, test_z):
                            return True
                else:
                    test_x = x + dx * self.config.resolution
                    test_y = y + dy * self.config.resolution
                    
                    if self.is_occupied(test_x, test_y, 0.0):
                        return True
        
        return False
    
    def add_sphere_obstacle(self, center: Tuple[float, float, float], radius: float):
        """Add a spherical obstacle to the grid.
        
        Args:
            center: Sphere center (x, y, z) in meters
            radius: Sphere radius in meters
        """
        cx, cy, cz = center
        ix_c, iy_c, iz_c = self.world_to_grid(cx, cy, cz)
        
        # Determine cells to update
        r_cells = int(radius / self.config.resolution) + 1
        
        for ix in range(max(0, ix_c - r_cells), min(self.nx, ix_c + r_cells + 1)):
            for iy in range(max(0, iy_c - r_cells), min(self.ny, iy_c + r_cells + 1)):
                if self.use_3d:
                    for iz in range(max(0, iz_c - r_cells), min(self.nz, iz_c + r_cells + 1)):
                        x, y, z = self.grid_to_world(ix, iy, iz)
                        dist = np.sqrt((x - cx)**2 + (y - cy)**2 + (z - cz)**2)
                        if dist <= radius:
                            self.grid[ix, iy, iz] = 0.95  # High probability of occupied
                            self.update_count[ix, iy, iz] += 1
                else:
                    x, y, _ = self.grid_to_world(ix, iy, 0)
                    dist = np.sqrt((x - cx)**2 + (y - cy)**2)
                    if dist <= radius:
                        self.grid[ix, iy] = 0.95  # High probability of occupied
                        self.update_count[ix, iy] += 1
    
    def add_box_obstacle(self, min_corner: Tuple[float, float, float], 
                        max_corner: Tuple[float, float, float]):
        """Add a box-shaped obstacle to the grid.
        
        Args:
            min_corner: Minimum corner (x, y, z) in meters
            max_corner: Maximum corner (x, y, z) in meters
        """
        ix_min, iy_min, iz_min = self.world_to_grid(*min_corner)
        ix_max, iy_max, iz_max = self.world_to_grid(*max_corner)
        
        for ix in range(ix_min, ix_max + 1):
            for iy in range(iy_min, iy_max + 1):
                if self.use_3d:
                    for iz in range(iz_min, iz_max + 1):
                        if self.is_valid_index(ix, iy, iz):
                            self.grid[ix, iy, iz] = 0.95
                            self.update_count[ix, iy, iz] += 1
                else:
                    if self.is_valid_index(ix, iy, 0):
                        self.grid[ix, iy] = 0.95
                        self.update_count[ix, iy] += 1
    
    def clear_grid(self):
        """Reset the grid to unknown state."""
        self.grid.fill(0.5)
        self.update_count.fill(0)
    
    def get_occupied_cells(self) -> np.ndarray:
        """Get list of occupied cell coordinates.
        
        Returns:
            Nx3 array of world coordinates [x, y, z] for occupied cells
        """
        if self.use_3d:
            occupied_indices = np.argwhere(self.grid > self.config.occupied_threshold)
        else:
            occupied_indices = np.argwhere(self.grid > self.config.occupied_threshold)
        
        occupied_coords = []
        for idx in occupied_indices:
            if self.use_3d:
                x, y, z = self.grid_to_world(idx[0], idx[1], idx[2])
            else:
                x, y, z = self.grid_to_world(idx[0], idx[1], 0)
            occupied_coords.append([x, y, z])
        
        return np.array(occupied_coords) if occupied_coords else np.array([]).reshape(0, 3)
    
    def get_grid_slice(self, z_height: float = 0.0) -> np.ndarray:
        """Get a 2D slice of the grid at a specific height.
        
        Args:
            z_height: Height in meters for the slice (only for 3D grids)
            
        Returns:
            2D numpy array of occupancy probabilities
        """
        if not self.use_3d:
            return self.grid.copy()
        
        _, _, iz = self.world_to_grid(0.0, 0.0, z_height)
        return self.grid[:, :, iz].copy()
    
    def get_grid_info(self) -> dict:
        """Get grid information and statistics.
        
        Returns:
            Dictionary with grid parameters and statistics
        """
        occupied_cells = np.sum(self.grid > self.config.occupied_threshold)
        free_cells = np.sum(self.grid < self.config.free_threshold)
        unknown_cells = np.sum((self.grid >= self.config.free_threshold) & 
                              (self.grid <= self.config.occupied_threshold))
        total_cells = self.grid.size
        
        return {
            'dimensions': (self.nx, self.ny, self.nz if self.use_3d else 1),
            'resolution': self.config.resolution,
            'bounds': {
                'x': (self.config.x_min, self.config.x_max),
                'y': (self.config.y_min, self.config.y_max),
                'z': (self.config.z_min, self.config.z_max) if self.use_3d else (0, 0)
            },
            'occupied_cells': int(occupied_cells),
            'free_cells': int(free_cells),
            'unknown_cells': int(unknown_cells),
            'total_cells': int(total_cells),
            'occupied_percentage': float(occupied_cells / total_cells * 100),
            'free_percentage': float(free_cells / total_cells * 100),
            'unknown_percentage': float(unknown_cells / total_cells * 100)
        }
