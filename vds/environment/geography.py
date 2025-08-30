# vds/environment/geography.py

import pandas as pd
import numpy as np
from dataclasses import dataclass, field
import random

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
        self.depth_data = depth_data
        self.cell_size = cell_size
        self.grid_height, self.grid_width = depth_data.shape
        self.map_width = self.grid_width * self.cell_size
        self.map_height = self.grid_height * self.cell_size
        self.obstructions: list[Obstruction] = []

    @classmethod
    def from_csv(cls, file_path: str, cell_size: float):
        data = pd.read_csv(file_path, header=None).values
        print("Geography data loaded.")
        return cls(data, cell_size)

    def get_depth_at(self, x: float, y: float) -> float:
        grid_j = int(y / self.cell_size) 
        grid_i = int(x / self.cell_size) 
        if 0 <= grid_i < self.grid_height and 0 <= grid_j < self.grid_width:
            return self.depth_data[grid_i, grid_j]
        else:
            return 1000.0

    def add_obstacle(self, center_x: float, center_y: float, radius: float):
        """Creates and adds a circular obstruction at a specific location."""
        position = np.array([center_x, center_y])
        self.obstructions.append(Obstruction(position=position, radius=radius))

    def add_random_obstacles(self, count: int, min_radius: float, max_radius: float, safe_zone_radius: float = 0.0):
        """Generates and adds random obstacles, avoiding a safe zone around the origin."""
        for _ in range(count):
            while True: # Loop until a valid position is found
                rand_x = random.uniform(0, self.map_height)
                rand_y = random.uniform(0, self.map_width)
                position = np.array([rand_x, rand_y])
                
                # Check if the obstacle is outside the safe zone
                if np.linalg.norm(position) > safe_zone_radius:
                    rand_radius = random.uniform(min_radius, max_radius)
                    self.obstructions.append(Obstruction(position=position, radius=rand_radius))
                    break # Exit loop and create the next obstacle
        print(f"Added {count} random obstacles.")

