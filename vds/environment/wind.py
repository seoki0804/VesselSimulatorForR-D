# vds/environment/wind.py

from dataclasses import dataclass

@dataclass
class Wind:
    """
    Represents the environmental wind conditions.
    """
    speed: float  # Wind speed in m/s
    direction: float  # Wind direction in degrees (0=North, 90=East)
