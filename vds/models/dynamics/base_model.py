# vds/models/dynamics/base_model.py

from abc import ABC, abstractmethod
import numpy as np
from vds.models.vessels.base_vessel import VesselState

class BaseDynamicsModel(ABC):
    """
    Abstract Base Class for all vessel dynamics (physics) models.
    It defines the interface that the simulator will use to get the vessel's accelerations.
    """

    @abstractmethod
    def calculate_nu_dot(self, state: VesselState, control_input: dict) -> np.ndarray:
        """
        Calculates the body-fixed accelerations (nu_dot) based on the current state and control inputs.
        This is the core of the physics simulation, solving the equations of motion.

        Args:
            state (VesselState): The current state of the vessel.
            control_input (dict): A dictionary containing control inputs like 
                                  rudder angle (delta) and propeller rpm (n).
                                  Example: {'delta': -0.1, 'n': 10.0}

        Returns:
            np.ndarray: A 6-element array representing the accelerations [u_dot, v_dot, w_dot, p_dot, q_dot, r_dot].
        """
        pass
