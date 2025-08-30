# main.py

import pygame
import sys
import numpy as np
import json

from app.renderer import Renderer
from vds.core.simulator import Simulator
from vds.models.vessels.base_vessel import BaseVessel, VesselSpecifications, VesselState
from vds.models.dynamics.mmg_model import MMGModel
from vds.environment.geography import Geography
from vds.data_handler.ais_parser import load_ais_targets

def main():
    # --- Simulation Setup ---
    geography = Geography.from_csv('data/bathymetry/sample_depth.csv', cell_size=20)
    
    # FIXED: Adjusted safe zone and obstacle count to match the map size.
    geography.add_random_obstacles(
        count=5,          # Reduced count for better spacing
        min_radius=15, 
        max_radius=30,    # Slightly smaller obstacles
        safe_zone_radius=200.0 # A reasonable safe zone for a 400x400m map
    )
    
    ais_targets = load_ais_targets('data/ais/sample_ais_tracks.csv')
    print("Geography and data loaded.\n")

    specs = VesselSpecifications(loa=232.5, beam=32.2, draft=10.8, mass=5.2e7, inertia_z=2.17e10)
    initial_state = VesselState(nu=np.array([7.7, 0, 0, 0, 0, 0]))
    vessel = BaseVessel(specs, initial_state)

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
    control = {'rpm': 80.0, 'rudder_angle': 0.0}
    RUDDER_INCREMENT = 1.0
    RUDDER_MAX = 35.0
    RPM_INCREMENT = 5.0
    RPM_MAX = 300.0
    RPM_MIN = -200.0

    print("Starting simulation... Use Arrow Keys to control the vessel.")
    print("Controls: '0' KEY: Center Rudder | 'R' KEY: Reset | 'O': Obstacles | 'W': Water Depth")

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    control['rudder_angle'] = max(-RUDDER_MAX, control['rudder_angle'] - RUDDER_INCREMENT)
                elif event.key == pygame.K_RIGHT:
                    control['rudder_angle'] = min(RUDDER_MAX, control['rudder_angle'] + RUDDER_INCREMENT)
                elif event.key == pygame.K_UP:
                    control['rpm'] = min(RPM_MAX, control['rpm'] + RPM_INCREMENT)
                elif event.key == pygame.K_DOWN:
                    control['rpm'] = max(RPM_MIN, control['rpm'] - RPM_INCREMENT)
                elif event.key == pygame.K_0 or event.key == pygame.K_KP0:
                    control['rudder_angle'] = 0.0
                elif event.key == pygame.K_r:
                    simulator.reset()
                    control = {'rpm': 80.0, 'rudder_angle': 0.0}
                elif event.key == pygame.K_o:
                    simulator.show_obstacles = not simulator.show_obstacles
                elif event.key == pygame.K_w: # ADDED: Water depth toggle
                    simulator.show_water_depth = not simulator.show_water_depth

        if simulator.collision_detected:
            pass
        else:
            simulator.step(dt, control)

        renderer.render(
            simulator.vessel, 
            geography, 
            control, 
            simulator.time, 
            simulator.ais_targets, 
            simulator.track_history,
            simulator.show_obstacles,
            simulator.show_water_depth # Pass toggle state to renderer
        )

        clock.tick(60)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()

