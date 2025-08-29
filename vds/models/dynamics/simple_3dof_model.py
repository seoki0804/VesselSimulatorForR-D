# vds/models/dynamics/simple_3dof_model.py

import numpy as np
from .base_model import BaseDynamicsModel
from vds.models.vessels.base_vessel import VesselState

class Simple3DOFModel(BaseDynamicsModel):
    """
    A simplified 3-DOF (Surge, Sway, Yaw) dynamics model for vessel maneuvering.
    """
    def __init__(self, vessel_mass: float, inertia_z: float, vessel_draft: float):
        self.mass = vessel_mass
        self.Iz = inertia_z
        self.draft = vessel_draft
        # Simplified hydrodynamic derivatives (should be properly estimated for real ships)
        self.Xu = -50.0
        self.Yv = -200.0
        self.Nr = -10000.0

    def calculate_forces(self, state: VesselState, control: dict, depth: float = 1000.0) -> np.ndarray:
        """
        Calculates forces based on the current state and control inputs.
        Includes a simple model for shallow water effect.
        """
        u, v, _, _, _, r = state.nu
        rpm = control.get('rpm', 0)
        rudder_angle_deg = control.get('rudder_angle', 0)
        rudder_angle_rad = np.radians(rudder_angle_deg)

        # --- Shallow Water Effect ---
        h_T_ratio = depth / self.draft if self.draft > 0 else 100
        shallow_water_factor = 1.0
        if h_T_ratio < 3.0:
            shallow_water_factor = max(1.0, 4.0 - h_T_ratio)
        
        # --- Calculate Forces and Moment ---
        force_x = 5.0 * rpm
        damping_x = self.Xu * u * shallow_water_factor
        damping_y = self.Yv * v * shallow_water_factor
        damping_n = self.Nr * r * shallow_water_factor

        # --- Rudder Forces (simplified) ---
        rudder_lift = 2000 * (u**2) * np.sin(rudder_angle_rad)
        rudder_force_y = -rudder_lift
        # CORRECTED: The moment arm for the rudder force should result in the correct turning direction.
        # A positive rudder angle (starboard) should create a positive yaw moment (turn starboard).
        rudder_moment_n = rudder_lift * (50.0) # Changed from -50.0 to 50.0

        # --- Sum all forces and moments ---
        total_force_x = force_x + damping_x
        total_force_y = damping_y + rudder_force_y
        total_moment_n = damping_n + rudder_moment_n

        # --- Calculate accelerations (nu_dot) ---
        nu_dot = np.zeros(6)
        nu_dot[0] = total_force_x / self.mass
        nu_dot[1] = total_force_y / self.mass
        nu_dot[5] = total_moment_n / self.Iz
        
        return nu_dot

