# vds/models/dynamics/simple_3dof_model.py

import numpy as np
from .base_model import BaseDynamicsModel
from vds.models.vessels.base_vessel import VesselState

class Simple3DOFModel(BaseDynamicsModel):
    """
    A simplified 3-DOF (Surge, Sway, Yaw) dynamics model.
    This model includes a basic implementation of shallow water effects.
    """
    def __init__(self, vessel_mass: float, inertia_z: float, vessel_draft: float):
        self.M = np.array([
            [vessel_mass, 0, 0],
            [0, vessel_mass, 0],
            [0, 0, inertia_z]
        ])
        self.inv_M = np.linalg.inv(self.M)
        self.vessel_draft = vessel_draft

    def calculate_accelerations(self, current_state: VesselState, control_input: dict, water_depth: float) -> np.ndarray:
        """
        Calculates accelerations considering control inputs and shallow water effects.
        """
        u, v, r = current_state.nu[0], current_state.nu[1], current_state.nu[5]
        delta_rad = np.radians(control_input.get('delta_deg', 0.0))
        n_rpm = control_input.get('n_rpm', 0.0)

        # --- Shallow Water Effect Calculation ---
        h_over_T = water_depth / self.vessel_draft
        shallow_water_multiplier = 1.0
        if h_over_T < 3.0:
            # Simple model: resistance increases quadratically as h/T approaches 1
            shallow_water_multiplier = 1.0 + (3.0 - h_over_T)**2
        
        # --- Forces and Moments Calculation ---
        # 1. Propulsion (simplified)
        thrust = 10 * n_rpm

        # 2. Rudder Force (simplified)
        rudder_lift = -50000 * u**2 * delta_rad

        # 3. Damping / Resistance (Hydrodynamic forces)
        # These are affected by the shallow water multiplier
        damping_surge = -100 * u * abs(u) * shallow_water_multiplier
        damping_sway = -1000 * v * abs(v) * shallow_water_multiplier
        damping_yaw = -1e7 * r * abs(r) * shallow_water_multiplier

        # Total forces vector [X, Y, N]
        tau = np.array([
            thrust + damping_surge,
            rudder_lift + damping_sway,
            rudder_lift * -75 + damping_yaw # Rudder acts at a lever arm from CG
        ])

        # Calculate accelerations: nu_dot = M^-1 * tau
        nu_dot_3dof = self.inv_M @ tau
        
        # Return as a 6-DOF array
        nu_dot_6dof = np.zeros(6)
        nu_dot_6dof[0] = nu_dot_3dof[0]  # u_dot
        nu_dot_6dof[1] = nu_dot_3dof[1]  # v_dot
        nu_dot_6dof[5] = nu_dot_3dof[2]  # r_dot

        return nu_dot_6dof

