# vds/core/simulator.py

from vds.models.vessels.base_vessel import BaseVessel
from vds.models.dynamics.base_model import BaseDynamicsModel
from vds.core.kinematics import update_kinematics_6dof

class Simulator:
    """
    The main orchestrator for running the vessel dynamics simulation.
    It manages the simulation time and the update loop.
    """

    def __init__(self, vessel: BaseVessel, dynamics_model: BaseDynamicsModel):
        """
        Initializes the Simulator.

        Args:
            vessel (BaseVessel): The vessel instance to be simulated.
            dynamics_model (BaseDynamicsModel): The physics model to calculate the vessel's dynamics.
        """
        self.vessel = vessel
        self.dynamics_model = dynamics_model
        self.simulation_time = 0.0
        print("Simulator initialized.")

    def step(self, dt: float, control_input: dict):
        """
        Advances the simulation by a single time step, dt.

        Args:
            dt (float): The duration of the time step in seconds.
            control_input (dict): The control inputs for the vessel (e.g., rudder, propeller).
        """
        # 1. Calculate accelerations using the dynamics model
        nu_dot = self.dynamics_model.calculate_nu_dot(self.vessel.state, control_input)

        # 2. Integrate accelerations to update velocities (nu)
        #    (Euler forward integration)
        self.vessel.state.nu += nu_dot * dt

        # 3. Integrate velocities to update position and orientation (eta)
        #    This is handled by the kinematics module
        self.vessel.state = update_kinematics_6dof(self.vessel.state, dt)
        
        # 4. Advance simulation time
        self.simulation_time += dt

    def run(self, duration: float, dt: float = 0.1, initial_control: dict = {'delta': 0.0, 'n': 0.0}):
        """
        Runs the simulation for a given duration.

        Args:
            duration (float): The total time to run the simulation in seconds.
            dt (float, optional): The time step for each update. Defaults to 0.1.
            initial_control (dict, optional): The control inputs to be used for the simulation run.
        """
        print(f"\nRunning simulation for {duration} seconds...")
        
        num_steps = int(duration / dt)
        for i in range(num_steps):
            self.step(dt, initial_control)
            # Print status periodically, e.g., 10 times during the simulation
            if num_steps > 10 and i % (num_steps // 10) == 0:
                print(f"Time: {self.simulation_time:.1f}s | {self.vessel.get_state_summary()}")
        
        print("Simulation finished.")
        print(f"Final State -> Time: {self.simulation_time:.1f}s | {self.vessel.get_state_summary()}")

