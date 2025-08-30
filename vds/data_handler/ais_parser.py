# vds/data_handler/ais_parser.py

import pandas as pd
import numpy as np
from scipy.interpolate import interp1d
from dataclasses import dataclass

@dataclass
class AISTargetState:
    """Holds the state of an AIS target at a specific time."""
    x: float = 0.0
    y: float = 0.0
    cog_rad: float = 0.0

class AISTarget:
    """Represents a single AIS target that moves along a pre-defined track."""
    def __init__(self, mmsi: int, track_data: pd.DataFrame):
        self.mmsi = mmsi
        self.state = AISTargetState()

        # Create interpolation functions for the track
        timestamps = track_data['timestamp'].values
        self._interp_x = interp1d(timestamps, track_data['x'].values, bounds_error=False, fill_value="extrapolate")
        self._interp_y = interp1d(timestamps, track_data['y'].values, bounds_error=False, fill_value="extrapolate")
        
        # Convert CoG to radians for consistency
        cog_rad = np.radians(track_data['cog_deg'].values)
        self._interp_cog = interp1d(timestamps, cog_rad, bounds_error=False, fill_value="extrapolate")
        
        # Initialize state to time 0
        self.update(0)

    def update(self, time: float):
        """Updates the target's state based on the simulation time."""
        self.state.x = self._interp_x(time)
        self.state.y = self._interp_y(time)
        self.state.cog_rad = self._interp_cog(time)

def load_ais_targets(file_path: str) -> list[AISTarget]:
    """Loads all AIS tracks from a CSV file and returns a list of AISTarget objects."""
    try:
        df = pd.read_csv(file_path)
    except FileNotFoundError:
        print(f"Warning: AIS data file not found at {file_path}")
        return []
        
    targets = []
    for mmsi, group in df.groupby('mmsi'):
        # Ensure the track has at least 2 points for interpolation
        if len(group) > 1:
            target = AISTarget(mmsi, group)
            targets.append(target)
    print(f"Loaded {len(targets)} AIS targets.")
    return targets
