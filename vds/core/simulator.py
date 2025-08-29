# vds/core/simulator.py

import numpy as np
from .kinematics import update_kinematics_6dof
from vds.models.vessels.base_vessel import BaseVessel
from vds.models.dynamics.base_model import BaseDynamicsModel
from vds.environment.geography import Geography

class Simulator:
    """
    Manages the overall simulation loop, state updates, and interactions.
    """
    def __init__(self, vessel: BaseVessel, dynamics_model: BaseDynamicsModel, geography: Geography):
        """
        Initializes the simulator.

        Args:
            vessel (BaseVessel): The vessel object to be simulated.
            dynamics_model (BaseDynamicsModel): The physics model for the vessel.
            geography (Geography): The environment data.
        """
        self.vessel = vessel
        self.dynamics_model = dynamics_model
        self.geography = geography
        self.time = 0.0
        self.collision_detected = False

    def step(self, dt: float, control: dict):
        """
        Advances the simulation by one time step.

        Args:
            dt (float): The time step duration in seconds.
            control (dict): A dictionary containing control inputs (e.g., {'rpm': ..., 'rudder_angle': ...}).
        """
        if self.collision_detected:
            return

        # 1. Get current depth at vessel position
        current_depth = self.geography.get_depth_at(self.vessel.state.eta[0], self.vessel.state.eta[1])

        # 2. Calculate accelerations (nu_dot) from the dynamics model
        nu_dot = self.dynamics_model.calculate_forces(self.vessel.state, control, current_depth)
        
        # 3. Integrate velocities (nu) over the time step
        self.vessel.state.nu += nu_dot * dt
        
        # 4. Update vessel state (position and orientation) using kinematics
        # UPDATED: Now passes the entire state object
        self.vessel.state = update_kinematics_6dof(self.vessel.state, dt)

        # 5. Check for collisions with obstructions
        self.check_collisions()

        # 6. Update simulation time
        self.time += dt

    def check_collisions(self):
        """Checks for collisions between the vessel and any obstructions."""
        vessel_pos = self.vessel.state.eta[:2] # Get (x, y) position
        for obs in self.geography.obstructions:
            dist = np.linalg.norm(vessel_pos - obs.position)
            if dist < (self.vessel.specs.loa / 2 + obs.radius): # Simple circle collision
                self.collision_detected = True
                print(f"COLLISION DETECTED with obstruction at {obs.position}!")
                break
    
    def run(self, duration: float, dt: float, control: dict):
        """
        Runs the simulation for a given duration. (Mainly for non-GUI testing)
        
        Args:
            duration (float): Total time to simulate in seconds.
            dt (float): Time step in seconds.
            control (dict): Control inputs.
        """
        num_steps = int(duration / dt)
        for i in range(num_steps):
            self.step(dt, control)
            if (i % (10 / dt) == 0): # Print status every 10 seconds
                print(f"Time: {self.time:.1f}s | "
                      f"Position: ({self.vessel.state.eta[0]:.2f}, {self.vessel.state.eta[1]:.2f}) m | "
                      f"Speed: {self.vessel.sog:.2f} knots | "
                      f"Heading: {np.degrees(self.vessel.state.eta[5]):.2f}°")

