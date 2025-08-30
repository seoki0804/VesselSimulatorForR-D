# vds/environment/current.py

from dataclasses import dataclass

@dataclass
class Current:
    """
    Represents the environmental water current conditions.
    """
    speed: float  # Current speed in m/s
    direction: float  # Current direction in degrees (0=North, 90=East)
