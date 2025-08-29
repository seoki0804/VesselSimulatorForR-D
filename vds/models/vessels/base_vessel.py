# vds/models/vessels/base_vessel.py

import numpy as np
from dataclasses import dataclass

@dataclass
class VesselSpecifications:
    """
    Holds the static parameters of the vessel.
    """
    length: float  # Length of the vessel in meters
    width: float   # Width of the vessel in meters
    draft: float   # Draft of the vessel in meters
    displacement: float  # Displacement in metric tons

@dataclass
class VesselState:
    """
    Holds the dynamic state of the vessel at a single point in time.
    Uses 6-DOF (Degrees of Freedom) representation.
    
    eta: Position and orientation in Earth-fixed frame (NED - North-East-Down).
         [x, y, z, phi, theta, psi]
         (meters and radians)
         x: North position
         y: East position
         z: Down position (heave)
         phi: Roll angle
         theta: Pitch angle
         psi: Yaw angle (heading)

    nu: Velocities in body-fixed frame.
        [u, v, w, p, q, r]
        (m/s and rad/s)
        u: Surge velocity (longitudinal)
        v: Sway velocity (transverse)
        w: Heave velocity (vertical)
        p: Roll rate
        q: Pitch rate
        r: Yaw rate
    """
    eta: np.ndarray = np.zeros(6)
    nu: np.ndarray = np.zeros(6)

class BaseVessel:
    """
    Represents a generic vessel, holding its specifications and current state.
    This class acts as a data container.
    """
    def __init__(self, specs: VesselSpecifications, initial_state: VesselState):
        self.specs = specs
        self.state = initial_state

    def get_state_summary(self) -> dict:
        """
        Calculates and returns a dictionary of key performance indicators
        similar to AIS data.
        """
        # Velocities in body-frame
        u, v, _, _, _, r = self.state.nu
        # Position and orientation in earth-frame
        x, y, _, _, _, psi = self.state.eta

        # --- SOG (Speed Over Ground) ---
        # Note: Speed over ground is the magnitude of the velocity vector.
        # Since the transformation matrix is orthogonal, the vector length is preserved.
        # Thus, sqrt(u^2 + v^2) in body frame is the same as sqrt(vx_earth^2 + vy_earth^2)
        sog_mps = np.sqrt(u**2 + v**2)
        sog_kts = sog_mps * 1.94384  # m/s to knots conversion

        # --- HDG (Heading) ---
        hdg_deg = (np.degrees(psi)) % 360
        if hdg_deg < 0: # ensure 0-360 range
            hdg_deg += 360

        # --- COG (Course Over Ground) ---
        # This is the actual direction of the vessel's movement over the ground.
        # We need to transform body-velocities (u,v) to earth-frame velocities.
        vx_earth = u * np.cos(psi) - v * np.sin(psi)
        vy_earth = u * np.sin(psi) + v * np.cos(psi)
        
        # COG is undefined if speed is zero
        if np.isclose(sog_mps, 0):
            cog_deg = hdg_deg # If stationary, COG is usually reported as HDG
        else:
            cog_rad = np.arctan2(vy_earth, vx_earth)
            cog_deg = (np.degrees(cog_rad)) % 360
            if cog_deg < 0:
                cog_deg += 360

        # --- ROT (Rate of Turn) ---
        # Convert yaw rate from rad/s to deg/min
        rot_deg_min = np.degrees(r) * 60

        return {
            'pos_x': y, # Easting
            'pos_y': x, # Northing
            'sog_kts': sog_kts,
            'hdg_deg': hdg_deg,
            'cog_deg': cog_deg,
            'rot_deg_min': rot_deg_min,
        }

