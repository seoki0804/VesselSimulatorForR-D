# main.py

import pygame
import numpy as np
import sys
from collections import deque

from vds.core.simulator import Simulator
from vds.models.vessels.base_vessel import BaseVessel, VesselSpecifications, VesselState
from vds.models.dynamics.simple_3dof_model import Simple3DOFModel
from vds.environment.geography import Geography
from app.renderer import Renderer

def main():
    """
    Sets up and runs the main simulation and visualization loop.
    """
    # 1. Load Environment Data
    geography = Geography.from_csv('data/bathymetry/sample_depth.csv', cell_size=20)
    print("Geography data loaded.")

    # 2. Define Vessel Specifications
    specs = VesselSpecifications(
        length=30.0, # Smaller vessel to better see map details
        width=5.0,
        draft=2.0,
        displacement=500
    )

    # 3. Define Initial State (start near the center of the map)
    initial_state = VesselState(
        eta=np.zeros(6),
        nu=np.array([2.0, 0, 0, 0, 0, 0]) # ~4 knots
    )

    # 4. Create a Vessel Instance
    vessel = BaseVessel(specs, initial_state)

    # 5. Create a Dynamics Model Instance
    mass = specs.displacement * 1025
    Iz = 0.5 * mass * (specs.width**2)
    # --- FIX: Added vessel_draft argument ---
    dynamics_model = Simple3DOFModel(vessel_mass=mass, inertia_z=Iz, vessel_draft=specs.draft)

    # 6. Create the Simulator
    simulator = Simulator(vessel, dynamics_model, geography)

    # 7. Create the Renderer
    renderer = Renderer(width=1280, height=720)

    # 8. Simulation Loop Variables
    running = True
    dt = 0.1
    clock = pygame.time.Clock()
    trail = deque(maxlen=1000) 

    control = {'n_rpm': 80.0, 'delta_deg': 15.0}
    
    print("\nStarting simulation with visualization...")

    while running:
        running = renderer.handle_events()
        if not running: break

        simulator.step(dt, control)
        
        current_pos = simulator.vessel.state.eta[:2]
        trail.append(current_pos)

        # Pass geography data to the renderer
        renderer.render(simulator.vessel, list(trail), simulator.simulation_time, geography)

        clock.tick(60)

    print("Simulation finished.")
    renderer.quit()
    sys.exit()

if __name__ == "__main__":
    main()

