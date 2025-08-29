# vds/models/dynamics/base_model.py

import numpy as np
from abc import ABC, abstractmethod
from vds.models.vessels.base_vessel import VesselState

class BaseDynamicsModel(ABC):
    """
    Abstract base class for all vessel dynamics models.
    Defines the interface for calculating forces and moments.
    """

    @abstractmethod
    def calculate_forces(self, state: VesselState, control: dict, depth: float) -> np.ndarray:
        """
        Calculates the forces and moments acting on the vessel.

        Args:
            state (VesselState): The current state of the vessel.
            control (dict): The current control inputs (e.g., rpm, rudder_angle).
            depth (float): The water depth at the vessel's current position.

        Returns:
            np.ndarray: The resulting accelerations (nu_dot) in the body-fixed frame.
        """
        pass

