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
from vds.environment.wind import Wind
from vds.environment.current import Current
from vds.environment.waves import Waves

def settings_loop(renderer, clock):
    """Loop for the pre-simulation settings screen."""
    settings = {
        "wind_speed": "10.0", "wind_dir": "45",
        "current_speed": "1.0", "current_dir": "180",
        "waves_h": "0.5", "waves_dir": "90"
    }
    fields = list(settings.keys())
    active_field_index = 0
    
    while True:
        active_field = fields[active_field_index]
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return None # Exit application
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_RETURN:
                    # Parse settings and start simulation
                    try:
                        wind = Wind(speed=float(settings["wind_speed"]), direction=float(settings["wind_dir"]))
                        current = Current(speed=float(settings["current_speed"]), direction=float(settings["current_dir"]))
                        waves = Waves(significant_height=float(settings["waves_h"]), period=8.0, direction=float(settings["waves_dir"]))
                        return wind, current, waves
                    except ValueError:
                        print("Invalid number in settings. Please check.")
                        continue # Stay in settings loop
                elif event.key == pygame.K_TAB:
                    active_field_index = (active_field_index + 1) % len(fields)
                elif event.key == pygame.K_BACKSPACE:
                    settings[active_field] = settings[active_field][:-1]
                else:
                    settings[active_field] += event.unicode

        renderer.draw_settings_screen(settings, active_field)
        clock.tick(30)

def main():
    SCREEN_WIDTH, SCREEN_HEIGHT = 1280, 720
    renderer = Renderer(SCREEN_WIDTH, SCREEN_HEIGHT)
    clock = pygame.time.Clock()

    # --- Run Settings Loop First ---
    env_factors = settings_loop(renderer, clock)
    if env_factors is None:
        pygame.quit()
        sys.exit()
    wind, current, waves = env_factors

    # --- Simulation Setup ---
    geography = Geography.from_csv('data/bathymetry/sample_depth.csv', cell_size=20)
    geography.add_random_obstacles(count=5, min_radius=15, max_radius=30, safe_zone_radius=200.0)
    ais_targets = load_ais_targets('data/ais/sample_ais_tracks.csv')
    
    print(f"Starting with: Wind({wind.speed}kts, {wind.direction}°), Current({current.speed}kts, {current.direction}°), Waves(Hs={waves.significant_height}m, {waves.direction}°)")
    
    specs = VesselSpecifications(
        loa=232.5, beam=32.2, draft=10.8, mass=5.2e7, inertia_z=2.17e10,
        wind_area_longitudinal=800.0, wind_area_transverse=2500.0
    )
    initial_state = VesselState(nu=np.array([7.7, 0, 0, 0, 0, 0]))
    vessel = BaseVessel(specs, initial_state)
    hydro_params_path = 'data/vessel_params/kcs_hydrodynamics.json'
    dynamics_model = MMGModel(vessel.specs, hydro_params_path)
    simulator = Simulator(vessel, dynamics_model, geography, ais_targets, wind, current, waves)
    simulator.show_obstacles = False # Set initial state of obstacles to OFF

    # --- Main Simulation Loop ---
    running = True
    dt = 0.1
    control = {'rpm': 80.0, 'rudder_angle': 0.0}
    RUDDER_INCREMENT, RUDDER_MAX = 1.0, 35.0
    RPM_INCREMENT, RPM_MAX, RPM_MIN = 5.0, 300.0, -200.0

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT: control['rudder_angle'] = max(-RUDDER_MAX, control['rudder_angle'] - RUDDER_INCREMENT)
                elif event.key == pygame.K_RIGHT: control['rudder_angle'] = min(RUDDER_MAX, control['rudder_angle'] + RUDDER_INCREMENT)
                elif event.key == pygame.K_UP: control['rpm'] = min(RPM_MAX, control['rpm'] + RPM_INCREMENT)
                elif event.key == pygame.K_DOWN: control['rpm'] = max(RPM_MIN, control['rpm'] - RPM_INCREMENT)
                elif event.key == pygame.K_0 or event.key == pygame.K_KP0: control['rudder_angle'] = 0.0
                elif event.key == pygame.K_r: simulator.reset(); control = {'rpm': 80.0, 'rudder_angle': 0.0}
                elif event.key == pygame.K_o: simulator.show_obstacles = not simulator.show_obstacles
                elif event.key == pygame.K_w: simulator.show_water_depth = not simulator.show_water_depth

        if not simulator.collision_detected:
            simulator.step(dt, control)

        renderer.render(
            simulator.vessel, geography, control, simulator.time, 
            simulator.ais_targets, simulator.track_history,
            simulator.show_obstacles, simulator.show_water_depth,
            simulator.wind, simulator.current, simulator.waves
        )
        clock.tick(60)
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()

