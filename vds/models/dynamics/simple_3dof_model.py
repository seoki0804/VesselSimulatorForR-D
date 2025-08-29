# vds/models/dynamics/simple_3dof_model.py

import numpy as np
from vds.models.dynamics.base_model import BaseDynamicsModel
from vds.models.vessels.base_vessel import VesselState

class Simple3DOFModel(BaseDynamicsModel):
    """
    A simplified 3-DOF (Surge, Sway, Yaw) dynamics model for vessel maneuvering.
    Heave, Roll, and Pitch are ignored in this model.
    """
    def __init__(self, vessel_mass: float, inertia_z: float):
        """
        Initializes the simplified dynamics model.

        Args:
            vessel_mass (float): The total mass of the vessel (in kg).
            inertia_z (float): The moment of inertia around the z-axis (for yaw).
        """
        # Mass matrix (M_RB) - simplified for 3-DOF
        self.M = np.diag([vessel_mass, vessel_mass, vessel_mass, 999, 999, inertia_z])
        # Placeholder for added mass (will be zero for now)
        self.M_A = np.zeros((6, 6))
        self.M_inv = np.linalg.inv(self.M + self.M_A)

        # Damping matrix (linear damping) - rough estimation
        self.D = np.diag([1e4, 5e4, 5e4, 1e5, 1e5, 1e6])
        
        print("Simple3DOFModel initialized.")

    def calculate_nu_dot(self, state: VesselState, control_input: dict) -> np.ndarray:
        """
        Calculates accelerations based on a simplified force model.

        Equation: M * nu_dot + D * nu = tau
        -> nu_dot = M_inv * (tau - D * nu)
        """
        nu = state.nu
        
        # Get control inputs
        rudder_angle_rad = np.deg2rad(control_input.get('delta_deg', 0.0))
        propeller_rpm = control_input.get('n_rpm', 0.0)

        # --- Simplified Force Calculation (tau) ---
        # 1. Propulsion force (Surge)
        # Proportional to the square of RPM
        force_x = 10.0 * (propeller_rpm ** 2)

        # 2. Rudder forces (Sway and Yaw)
        # Proportional to forward speed (u) squared and rudder angle
        u_sq = state.nu[0] ** 2
        force_y = -5e4 * u_sq * rudder_angle_rad
        torque_z = -1e6 * u_sq * rudder_angle_rad

        tau = np.array([force_x, force_y, 0, 0, 0, torque_z])
        
        # --- Damping forces ---
        damping_forces = self.D @ nu
        
        # --- Calculate accelerations ---
        net_forces = tau - damping_forces
        nu_dot = self.M_inv @ net_forces

        return nu_dot
