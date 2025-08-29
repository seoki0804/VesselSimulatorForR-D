# vds/models/vessels/base_vessel.py

import numpy as np
from dataclasses import dataclass, field

@dataclass
class VesselSpecifications:
    """Holds the static specifications of a vessel."""
    loa: float     # Length Overall (LOA) in meters
    beam: float    # Beam of the vessel in meters
    draft: float   # Draft of the vessel in meters
    mass: float    # Total mass in kg
    inertia_z: float # Moment of inertia about the z-axis (for yaw)

@dataclass
class VesselState:
    """Holds the dynamic state of a vessel at a single point in time."""
    # Position and orientation in earth-fixed frame (NED)
    # [x, y, z, phi(roll), theta(pitch), psi(yaw)]
    eta: np.ndarray = field(default_factory=lambda: np.zeros(6))
    
    # Velocities in body-fixed frame
    # [u(surge), v(sway), w(heave), p(roll), q(pitch), r(yaw)]
    nu: np.ndarray = field(default_factory=lambda: np.zeros(6))

class BaseVessel:
    """
    Represents a generic vessel, acting as a data container for its
    specifications and dynamic state.
    """
    def __init__(self, specs: VesselSpecifications, initial_state: VesselState = VesselState()):
        self.specs = specs
        self.state = initial_state
    
    @property
    def sog(self) -> float:
        """Speed Over Ground in knots."""
        speed_mps = np.linalg.norm(self.state.nu[0:2]) # Surge and Sway
        return speed_mps * 1.94384 # m/s to knots

    @property
    def cog(self) -> float:
        """Course Over Ground in degrees (0-360)."""
        if self.sog < 0.1:
            # Return heading if speed is negligible
            hdg_rad = self.state.eta[5]
            return np.degrees(hdg_rad) % 360
        
        # Course includes sway velocity, it's the actual path over ground
        course_rad_body = np.arctan2(self.state.nu[1], self.state.nu[0])
        course_rad_earth = self.state.eta[5] + course_rad_body
        return np.degrees(course_rad_earth) % 360
        
    @property
    def rot(self) -> float:
        """Rate of Turn in degrees per minute."""
        yaw_rate_rad_s = self.state.nu[5]
        return np.degrees(yaw_rate_rad_s) * 60

    @property
    def heading(self) -> float:
        """Heading in degrees (0-360)."""
        hdg_rad = self.state.eta[5]
        return np.degrees(hdg_rad) % 360

