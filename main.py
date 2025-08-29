# main.py

import pygame
import numpy as np
import sys
from collections import deque

from vds.core.simulator import Simulator
from vds.models.vessels.base_vessel import BaseVessel, VesselSpecifications, VesselState
from vds.models.dynamics.simple_3dof_model import Simple3DOFModel
from app.renderer import Renderer

def main():
    """
    Sets up and runs the main simulation and visualization loop.
    """
    # 1. Define Vessel Specifications
    specs = VesselSpecifications(
        length=150.0,
        width=25.0,
        draft=10.0,
        displacement=20000
    )

    # 2. Define Initial State
    initial_state = VesselState(
        eta=np.zeros(6),
        nu=np.array([5.0, 0, 0, 0, 0, 0]) # 5 m/s forward speed (~10 knots)
    )

    # 3. Create a Vessel Instance
    vessel = BaseVessel(specs, initial_state)

    # 4. Create a Dynamics Model Instance
    mass = specs.displacement * 1025
    Iz = 0.5 * mass * (specs.width**2)
    dynamics_model = Simple3DOFModel(vessel_mass=mass, inertia_z=Iz)

    # 5. Create the Simulator
    simulator = Simulator(vessel, dynamics_model)

    # 6. Create the Renderer
    renderer = Renderer(width=1280, height=720)

    # 7. Simulation Loop Variables
    running = True
    dt = 0.1  # Simulation time step
    clock = pygame.time.Clock()
    
    # Store vessel trajectory for rendering
    trail = deque(maxlen=1000) 

    # Define Control Inputs
    control = {
        'n_rpm': 80.0,
        'delta_deg': 15.0
    }
    
    print("\nStarting simulation with visualization...")

    while running:
        # --- Event Handling ---
        running = renderer.handle_events()
        if not running:
            break

        # --- Simulation Step ---
        simulator.step(dt, control)

        # --- Data Logging for Rendering ---
        current_pos = simulator.vessel.state.eta[:2]
        trail.append(current_pos)

        # --- Rendering ---
        renderer.render(simulator.vessel, list(trail), simulator.simulation_time)

        # --- Frame Rate Control ---
        clock.tick(60) # Limit to 60 frames per second

    print("Simulation finished.")
    renderer.quit()
    sys.exit()


if __name__ == "__main__":
    main()

