# vds/environment/geography.py

import numpy as np
import pandas as pd

class Geography:
    """
    Manages grid-based geographical data like bathymetry (water depth).
    """
    def __init__(self, grid_data: np.ndarray, cell_size: float, origin: tuple = (0, 0)):
        """
        Initializes the geography grid.

        Args:
            grid_data (np.ndarray): 2D array of depth values (negative values for depth).
            cell_size (float): The size of each grid cell in meters.
            origin (tuple): The (x_north, y_east) coordinate of the top-left corner of the grid.
        """
        self.grid_data = grid_data
        self.cell_size = cell_size
        self.origin_x = origin[0]
        self.origin_y = origin[1]
        self.height, self.width = grid_data.shape

    @classmethod
    def from_csv(cls, file_path: str, cell_size: float, origin: tuple = (0, 0)):
        """
        Factory method to create a Geography instance from a CSV file.
        """
        data = pd.read_csv(file_path, header=None).values
        # Center the origin so the map appears around (0,0) in the world
        map_width_m = data.shape[1] * cell_size
        map_height_m = data.shape[0] * cell_size
        centered_origin = (-map_height_m / 2, -map_width_m / 2)
        return cls(data, cell_size, centered_origin)

    def get_depth_at(self, x_north: float, y_east: float) -> float:
        """
        Gets the water depth at a specific world coordinate.
        Returns a large negative number if out of bounds.
        """
        # Convert world coordinates to grid indices
        col = int((y_east - self.origin_y) / self.cell_size)
        row = int((x_north - self.origin_x) / self.cell_size)

        # Check bounds
        if 0 <= row < self.height and 0 <= col < self.width:
            return self.grid_data[row, col]
        else:
            return -9999  # Out of bounds
