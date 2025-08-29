# vds/core/kinematics.py

import numpy as np
from vds.models.vessels.base_vessel import VesselState

def transformation_matrix(phi: float, theta: float, psi: float) -> np.ndarray:
    """
    Computes the 6-DOF transformation matrix J(eta) that relates body-fixed velocities
    to the rates of change of position and orientation in the earth-fixed frame.
    
    eta_dot = J(eta) * nu

    Args:
        phi (float): Roll angle (radians).
        theta (float): Pitch angle (radians).
        psi (float): Yaw angle (radians).

    Returns:
        np.ndarray: The 6x6 transformation matrix.
    """
    c_phi, s_phi = np.cos(phi), np.sin(phi)
    c_theta, s_theta, t_theta = np.cos(theta), np.sin(theta), np.tan(theta)
    c_psi, s_psi = np.cos(psi), np.sin(psi)

    J1 = np.array([
        [c_psi * c_theta, -s_psi * c_phi + c_psi * s_theta * s_phi,  s_psi * s_phi + c_psi * c_phi * s_theta],
        [s_psi * c_theta,  c_psi * c_phi + s_phi * s_theta * s_psi, -c_psi * s_phi + s_theta * s_psi * c_phi],
        [-s_theta,         c_theta * s_phi,                          c_theta * c_phi]
    ])

    J2 = np.array([
        [1, s_phi * t_theta, c_phi * t_theta],
        [0, c_phi,          -s_phi],
        [0, s_phi / c_theta, c_phi / c_theta]
    ])
    
    J = np.zeros((6, 6))
    J[:3, :3] = J1
    J[3:, 3:] = J2
    
    return J

def update_kinematics_6dof(state: VesselState, dt: float) -> VesselState:
    """
    Updates the vessel's position and orientation (eta) for a time step dt,
    based on its current body-fixed velocities (nu).

    Args:
        state (VesselState): The current state of the vessel (eta and nu).
        dt (float): The time step for the simulation update (in seconds).

    Returns:
        VesselState: The updated state of the vessel.
    """
    phi, theta, psi = state.eta[3], state.eta[4], state.eta[5]

    # Get the transformation matrix for the current orientation
    J_matrix = transformation_matrix(phi, theta, psi)
    
    # Calculate the rate of change of eta in the earth-fixed frame
    eta_dot = J_matrix @ state.nu
    
    # Update position and orientation (eta) by integrating over the time step
    state.eta += eta_dot * dt
    
    # Normalize angles if necessary (e.g., keep yaw within [-pi, pi])
    state.eta[5] = (state.eta[5] + np.pi) % (2 * np.pi) - np.pi

    return state

