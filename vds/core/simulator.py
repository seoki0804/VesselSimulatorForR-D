# vds/core/simulator.py

import numpy as np
import copy
from collections import deque
from .kinematics import update_kinematics_6dof
from vds.models.vessels.base_vessel import BaseVessel
from vds.models.dynamics.base_model import BaseDynamicsModel
from vds.environment.geography import Geography
from vds.data_handler.ais_parser import AISTarget

class Simulator:
    """
    Manages the overall simulation loop, state updates, and interactions.
    """
    def __init__(self, vessel: BaseVessel, dynamics_model: BaseDynamicsModel, geography: Geography, ais_targets: list[AISTarget] = []):
        self.vessel = vessel
        self.dynamics_model = dynamics_model
        self.geography = geography
        self.ais_targets = ais_targets

        self.initial_vessel_state = copy.deepcopy(vessel.state)
        self.time = 0.0
        self.collision_detected = False
        self.track_history = deque(maxlen=10000)

    def reset(self):
        """Resets the simulation to its initial state."""
        self.vessel.state = copy.deepcopy(self.initial_vessel_state)
        self.time = 0.0
        self.collision_detected = False
        self.track_history.clear()
        for target in self.ais_targets:
            target.update(0)
        print("\n--- Simulation Reset ---")

    def step(self, dt: float, control: dict):
        if self.collision_detected:
            return

        # --- Update Own Ship ---
        current_depth = self.geography.get_depth_at(self.vessel.state.eta[0], self.vessel.state.eta[1])
        nu_dot = self.dynamics_model.calculate_forces(self.vessel.state, control, current_depth)
        self.vessel.state.nu += nu_dot * dt
        self.vessel.state = update_kinematics_6dof(self.vessel.state, dt)
        
        # --- REVERTED TRACKING LOGIC ---
        # Add a point to the track history on every step.
        self.track_history.append(self.vessel.state.eta[:2].copy())

        self.check_collisions()
        
        # --- Update AIS Targets ---
        for target in self.ais_targets:
            target.update(self.time)

        self.time += dt

    def check_collisions(self):
        """Checks for collisions between the vessel and any obstructions."""
        vessel_pos = self.vessel.state.eta[:2]
        for obs in self.geography.obstructions:
            dist = np.linalg.norm(vessel_pos - obs.position)
            if dist < (self.vessel.specs.loa / 2 + obs.radius):
                self.collision_detected = True
                print(f"COLLISION DETECTED with obstruction at {obs.position}!")
                break
    
    def run(self, duration: float, dt: float, control: dict):
        """
        Runs the simulation for a given duration. (Mainly for non-GUI testing)
        """
        num_steps = int(duration / dt)
        for i in range(num_steps):
            self.step(dt, control)
            if (i % (10 / dt) == 0):
                print(f"Time: {self.time:.1f}s | "
                      f"Position: ({self.vessel.state.eta[0]:.2f}, {self.vessel.state.eta[1]:.2f}) m | "
                      f"Speed: {self.vessel.sog:.2f} knots | "
                      f"Heading: {np.degrees(self.vessel.state.eta[5]):.2f}°")

