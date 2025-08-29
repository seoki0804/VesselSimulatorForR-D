import numpy as np
from vds.models.vessels.base_vessel import VesselState

def update_kinematics(state: VesselState, dt: float) -> VesselState:
    """
    Updates the vessel's position and heading based on its current velocity and rate of turn.
    This function handles the transformation from the body-fixed frame to the earth-fixed (NED) frame.

    Args:
        state (VesselState): The current state of the vessel.
        dt (float): The time step for the simulation update (in seconds).

    Returns:
        VesselState: The updated state of the vessel.
    """
    # 1. Update heading based on the rate of turn
    # The new heading is the old heading plus the angular distance turned in dt.
    state.heading += state.rate_of_turn * dt

    # Normalize heading angle to be within [-pi, pi]
    # This keeps the angle consistent and avoids large numbers.
    state.heading = (state.heading + np.pi) % (2 * np.pi) - np.pi

    # 2. Update position
    # The velocity (u, v) is in the vessel's body-fixed frame (surge, sway).
    # We need to convert it to the earth-fixed (NED) frame to update the global position.
    
    # Create the 2D rotation matrix for the current heading
    cos_h = np.cos(state.heading)
    sin_h = np.sin(state.heading)
    rotation_matrix = np.array([
        [cos_h, -sin_h],
        [sin_h,  cos_h]
    ])

    # Rotate the body-fixed velocity vector to get the earth-fixed velocity vector
    velocity_ned = rotation_matrix @ state.velocity

    # Calculate the change in position (delta_position = velocity * time)
    delta_position = velocity_ned * dt

    # Update the position
    state.position += delta_position

    return state
