# vds/models/vessels/base_vessel.py

from dataclasses import dataclass, field
import numpy as np

@dataclass
class VesselSpecifications:
    """
    Data class for storing the vessel's immutable specifications.
    
    Attributes:
        length (float): Length of the vessel (meters).
        width (float): Width of the vessel (meters).
        draft (float): Draft of the vessel (meters).
        displacement (float): Displacement of the vessel (tons).
    """
    length: float
    width: float
    draft: float
    displacement: float

@dataclass
class VesselState:
    """
    Data class for storing the vessel's dynamic state in 6-DOF (Degrees of Freedom).
    It uses the SNAME (Society of Naval Architects and Marine Engineers) notation.

    Attributes:
        eta (np.ndarray): Position and orientation in the earth-fixed frame (NED).
                          [x, y, z, phi, theta, psi]
                          (North, East, Down, Roll, Pitch, Yaw)
        nu (np.ndarray): Linear and angular velocities in the body-fixed frame.
                         [u, v, w, p, q, r]
                         (Surge, Sway, Heave, Roll rate, Pitch rate, Yaw rate)
    """
    # Earth-fixed position and Euler angles [x, y, z, phi, theta, psi]
    eta: np.ndarray = field(default_factory=lambda: np.zeros(6))
    
    # Body-fixed velocities [u, v, w, p, q, r]
    nu: np.ndarray = field(default_factory=lambda: np.zeros(6))


class BaseVessel:
    """
    A simple container class for vessel properties and state.
    It's not abstract, allowing for direct instantiation if a complex vessel type is not needed.
    """
    def __init__(self, specs: VesselSpecifications, initial_state: VesselState = VesselState()):
        """
        Constructor for BaseVessel.

        Args:
            specs (VesselSpecifications): The vessel's unique specifications.
            initial_state (VesselState): The initial state of the vessel.
        """
        self.specs = specs
        self.state = initial_state
        
        print(f"Vessel with length {self.specs.length}m created (6-DOF enabled).")

    def get_state_summary(self) -> str:
        """
        Returns a string summarizing the current state of the vessel.

        Returns:
            str: A summary of the vessel's state.
        """
        pos = self.state.eta[:2]  # [x, y]
        heading_deg = np.rad2deg(self.state.eta[5]) # psi
        speed_mps = np.linalg.norm(self.state.nu[:3]) # sqrt(u^2 + v^2 + w^2)
        speed_knots = speed_mps * 1.94384

        return (
            f"Position: ({pos[0]:.2f}, {pos[1]:.2f}) m | "
            f"Speed: {speed_knots:.2f} knots | "
            f"Heading: {heading_deg:.2f}°"
        )

