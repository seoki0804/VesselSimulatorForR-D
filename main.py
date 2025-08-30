# main.py

import pygame
import sys
import numpy as np
import json # Import json to load hydro params

from app.renderer import Renderer
from vds.core.simulator import Simulator
from vds.models.vessels.base_vessel import BaseVessel, VesselSpecifications, VesselState
# from vds.models.dynamics.simple_3dof_model import Simple3DOFModel # No longer needed
from vds.models.dynamics.mmg_model import MMGModel # Import the new model
from vds.environment.geography import Geography
from vds.data_handler.ais_parser import load_ais_targets

def main():
    # --- Simulation Setup ---
    geography = Geography.from_csv('data/bathymetry/sample_depth.csv', cell_size=20)
    geography.add_obstacle(center_x=500, center_y=-400, radius=50)
    geography.add_obstacle(center_x=1000, center_y=-750, radius=80)
    
    ais_targets = load_ais_targets('data/ais/sample_ais_tracks.csv')
    print("Geography and data loaded.\n")

    # KCS Vessel Specifications
    specs = VesselSpecifications(
        loa=232.5,
        beam=32.2,
        draft=10.8,
        mass=5.2e7,       # Mass in kg for KCS
        inertia_z=2.17e10 # Yaw moment of inertia for KCS
    )
    initial_state = VesselState(
        nu=np.array([7.7, 0, 0, 0, 0, 0]) # Start with ~15 knots forward speed
    )
    vessel = BaseVessel(specs, initial_state)

    # --- Initialize NEW Dynamics Model ---
    hydro_params_path = 'data/vessel_params/kcs_hydrodynamics.json'
    dynamics_model = MMGModel(vessel.specs, hydro_params_path)

    simulator = Simulator(vessel, dynamics_model, geography, ais_targets)

    # --- Pygame Setup ---
    SCREEN_WIDTH = 1280
    SCREEN_HEIGHT = 720
    renderer = Renderer(SCREEN_WIDTH, SCREEN_HEIGHT)

    # --- Main Loop ---
    running = True
    clock = pygame.time.Clock()
    dt = 0.1

    # Control state (RPM and Rudder are now more sensitive)
    control = {'rpm': 80.0, 'rudder_angle': 0.0}
    RUDDER_INCREMENT = 1.0
    RUDDER_MAX = 35.0
    RPM_INCREMENT = 5.0 # Smaller increment
    RPM_MAX = 150.0

    print("Starting simulation... Use Arrow Keys to control the vessel.")
    print("UP/DOWN: RPM | LEFT/RIGHT: Rudder | '0' KEY: Center Rudder | 'R' KEY: Reset")

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                # ... (Control code is the same)
                if event.key == pygame.K_LEFT:
                    control['rudder_angle'] = max(-RUDDER_MAX, control['rudder_angle'] - RUDDER_INCREMENT)
                elif event.key == pygame.K_RIGHT:
                    control['rudder_angle'] = min(RUDDER_MAX, control['rudder_angle'] + RUDDER_INCREMENT)
                elif event.key == pygame.K_UP:
                    control['rpm'] = min(RPM_MAX, control['rpm'] + RPM_INCREMENT)
                elif event.key == pygame.K_DOWN:
                    control['rpm'] = max(0, control['rpm'] - RPM_INCREMENT)
                elif event.key == pygame.K_0 or event.key == pygame.K_KP0:
                    control['rudder_angle'] = 0.0
                elif event.key == pygame.K_r:
                    simulator.reset()
                    control = {'rpm': 80.0, 'rudder_angle': 0.0}

        if simulator.collision_detected:
            pass
        else:
            simulator.step(dt, control)

        renderer.render(simulator.vessel, geography, control, simulator.time, simulator.ais_targets)

        clock.tick(60)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()

