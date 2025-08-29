# vds/core/simulator.py

import numpy as np
from .kinematics import update_kinematics_6dof
from vds.models.vessels.base_vessel import BaseVessel
from vds.models.dynamics.base_model import BaseDynamicsModel
from vds.environment.geography import Geography

class Simulator:
    """
    Manages the overall simulation loop, state updates, and interactions
    between the vessel and its environment.
    """
    def __init__(self, vessel: BaseVessel, dynamics_model: BaseDynamicsModel, geography: Geography = None):
        """
        Initializes the simulator.

        Args:
            vessel (BaseVessel): The vessel object to be simulated.
            dynamics_model (BaseDynamicsModel): The physics model for the vessel.
            geography (Geography, optional): The geographical data. Defaults to None.
        """
        self.vessel = vessel
        self.dynamics_model = dynamics_model
        self.geography = geography
        self.simulation_time = 0.0

    def step(self, dt: float, control_input: dict):
        """
        Advances the simulation by one time step.

        Args:
            dt (float): The time step duration in seconds.
            control_input (dict): Control inputs for the vessel.
        """
        # 1. Get water depth at current vessel position
        current_depth = 1000.0 # Default deep water
        if self.geography:
            x_north = self.vessel.state.eta[0]
            y_east = self.vessel.state.eta[1]
            # Depth values are negative, so we take the absolute value
            current_depth = abs(self.geography.get_depth_at(x_north, y_east))

        # 2. Calculate accelerations using the dynamics model
        nu_dot = self.dynamics_model.calculate_accelerations(
            self.vessel.state, 
            control_input,
            current_depth 
        )

        # 3. Integrate velocities (Euler integration)
        self.vessel.state.nu += nu_dot * dt

        # 4. Update position and orientation using kinematics
        self.vessel.state = update_kinematics_6dof(self.vessel.state, dt)

        # 5. Advance simulation time
        self.simulation_time += dt

