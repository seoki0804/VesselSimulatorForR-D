# vds/models/dynamics/base_model.py

import numpy as np
from abc import ABC, abstractmethod
from vds.models.vessels.base_vessel import VesselState

class BaseDynamicsModel(ABC):
    """
    Abstract base class for all vessel dynamics (physics) models.
    Defines the interface for calculating forces and accelerations.
    """
    @abstractmethod
    def calculate_accelerations(self, current_state: VesselState, control_input: dict, water_depth: float) -> np.ndarray:
        """
        Calculates the accelerations in the body-fixed frame (nu_dot).

        Args:
            current_state (VesselState): The current state of the vessel.
            control_input (dict): A dictionary containing control inputs like 
                                  propeller RPM ('n_rpm') and rudder angle ('delta_deg').
            water_depth (float): The water depth at the vessel's current position in meters.

        Returns:
            np.ndarray: A 6-element array representing the accelerations [u_dot, v_dot, w_dot, p_dot, q_dot, r_dot].
        """
        pass

