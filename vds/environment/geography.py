# vds/environment/geography.py

import pandas as pd
import numpy as np
from dataclasses import dataclass, field

@dataclass
class Obstruction:
    """Represents a simple circular obstruction."""
    position: np.ndarray = field(default_factory=lambda: np.zeros(2))
    radius: float = 10.0

class Geography:
    """
    Manages geographical data like bathymetry (water depth) and obstructions.
    """
    def __init__(self, depth_data: np.ndarray, cell_size: float):
        """
        Initializes the Geography object.

        Args:
            depth_data (np.ndarray): A 2D numpy array representing the depth grid.
            cell_size (float): The size of each grid cell in meters.
        """
        self.depth_data = depth_data
        self.cell_size = cell_size
        self.grid_height, self.grid_width = depth_data.shape
        self.map_width = self.grid_width * self.cell_size
        self.map_height = self.grid_height * self.cell_size
        self.obstructions: list[Obstruction] = []

    @classmethod
    def from_csv(cls, file_path: str, cell_size: float):
        """
        Factory method to create a Geography instance from a CSV file.

        Args:
            file_path (str): The path to the CSV file.
            cell_size (float): The size of each grid cell in meters.

        Returns:
            Geography: A new instance of the Geography class.
        """
        # Load data using pandas, assuming no header
        data = pd.read_csv(file_path, header=None).values
        print("Geography data loaded.")
        return cls(data, cell_size)

    def get_depth_at(self, x: float, y: float) -> float:
        """
        Gets the water depth at a specific world coordinate (x, y).

        Args:
            x (float): The x-coordinate in world space.
            y (float): The y-coordinate in world space.

        Returns:
            float: The depth at the given coordinate. Returns a large value if out of bounds.
        """
        # Convert world coordinates to grid indices
        grid_j = int(x / self.cell_size)
        grid_i = int(y / self.cell_size)

        # Check if the coordinates are within the grid bounds
        if 0 <= grid_i < self.grid_height and 0 <= grid_j < self.grid_width:
            return self.depth_data[grid_i, grid_j]
        else:
            return 1000.0  # Deep water if out of map bounds

    def add_obstacle(self, center_x: float, center_y: float, radius: float):
        """Creates and adds a circular obstruction to the geography."""
        position = np.array([center_x, center_y])
        obstruction = Obstruction(position=position, radius=radius)
        self.obstructions.append(obstruction)

