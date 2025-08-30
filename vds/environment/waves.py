# vds/environment/waves.py

from dataclasses import dataclass

@dataclass
class Waves:
    """
    Represents the environmental wave conditions (simplified).
    """
    significant_height: float # Significant wave height (Hs) in meters
    period: float             # Wave period (Tp) in seconds
    direction: float          # Wave direction in degrees (coming from)
