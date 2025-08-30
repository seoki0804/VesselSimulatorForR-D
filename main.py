# main.py

import pygame
import sys
import numpy as np
from app.renderer import Renderer
from vds.core.simulator import Simulator
from vds.utils.logger import DataLogger
from scenarios.scenario_loader import load_scenario
from vds.environment.wind import Wind
from vds.environment.current import Current
from vds.environment.waves import Waves

def settings_loop(renderer, clock, default_env):
    """Loop for pre-simulation settings, with defaults from scenario."""
    settings = {
        "wind_speed": str(default_env['wind'].speed),
        "wind_dir": str(default_env['wind'].direction),
        "current_speed": str(default_env['current'].speed),
        "current_dir": str(default_env['current'].direction),
        "waves_h": str(default_env['waves'].significant_height),
        "waves_dir": str(default_env['waves'].direction)
    }
    fields = list(settings.keys())
    active_field_index = 0
    
    while True:
        active_field = fields[active_field_index]
        for event in pygame.event.get():
            if event.type == pygame.QUIT: return None
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_RETURN:
                    try:
                        wind = Wind(speed=float(settings["wind_speed"]), direction=float(settings["wind_dir"]))
                        current = Current(speed=float(settings["current_speed"]), direction=float(settings["current_dir"]))
                        waves = Waves(significant_height=float(settings["waves_h"]), period=8.0, direction=float(settings["waves_dir"]))
                        return wind, current, waves
                    except ValueError:
                        print("Invalid number in settings. Please check.")
                        continue
                elif event.key == pygame.K_TAB:
                    active_field_index = (active_field_index + 1) % len(fields)
                elif event.key == pygame.K_BACKSPACE:
                    settings[active_field] = settings[active_field][:-1]
                else:
                    if event.unicode.isdigit() or (event.unicode == '.' and '.' not in settings[active_field]):
                        settings[active_field] += event.unicode

        renderer.draw_settings_screen(settings, active_field)
        clock.tick(30)

def main(scenario_path='scenarios/turning_test_starboard.yaml'):
    # --- Load Scenario ---
    vessel, dynamics_model, geography, ais_targets, wind, current, waves, initial_control = load_scenario(scenario_path)
    
    # --- Pygame & UI Setup ---
    SCREEN_WIDTH, SCREEN_HEIGHT = 1280, 720
    renderer = Renderer(SCREEN_WIDTH, SCREEN_HEIGHT)
    clock = pygame.time.Clock()

    # --- Run Settings Loop (overrides scenario defaults) ---
    default_env = {'wind': wind, 'current': current, 'waves': waves}
    env_factors = settings_loop(renderer, clock, default_env)
    if env_factors is None:
        pygame.quit(); sys.exit()
    wind, current, waves = env_factors
    
    # --- Initialize Simulator with final settings ---
    simulator = Simulator(vessel, dynamics_model, geography, ais_targets, wind, current, waves)
    simulator.show_obstacles = False # Default to OFF

    logger = DataLogger()

    # --- Main Simulation Loop ---
    running = True
    dt = 0.1
    control = initial_control.copy()
    RUDDER_INCREMENT, RUDDER_MAX = 1.0, 35.0
    RPM_INCREMENT, RPM_MAX, RPM_MIN = 5.0, 300.0, -200.0
    print("Controls: '0': Center Rudder | 'R': Reset | 'O': Obstacles | 'W': Water Depth | 'P': Pause")

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT: control['rudder_angle'] = max(-RUDDER_MAX, control['rudder_angle'] - RUDDER_INCREMENT)
                elif event.key == pygame.K_RIGHT: control['rudder_angle'] = min(RUDDER_MAX, control['rudder_angle'] + RUDDER_INCREMENT)
                elif event.key == pygame.K_UP: control['rpm'] = min(RPM_MAX, control['rpm'] + RPM_INCREMENT)
                elif event.key == pygame.K_DOWN: control['rpm'] = max(RPM_MIN, control['rpm'] - RPM_INCREMENT)
                elif event.key == pygame.K_0 or event.key == pygame.K_KP0: control['rudder_angle'] = 0.0
                elif event.key == pygame.K_r: simulator.reset(); control = initial_control.copy()
                elif event.key == pygame.K_o: simulator.show_obstacles = not simulator.show_obstacles
                elif event.key == pygame.K_w: simulator.show_water_depth = not simulator.show_water_depth
                elif event.key == pygame.K_p: simulator.is_paused = not simulator.is_paused

        if not simulator.collision_detected:
            if not simulator.is_paused:
                logger.log(simulator, control)
            simulator.step(dt, control)

        renderer.render(
            simulator.vessel, geography, control, simulator.time, 
            simulator.ais_targets, simulator.track_history,
            simulator.show_obstacles, simulator.show_water_depth,
            simulator.wind, simulator.current, simulator.waves,
            simulator.is_paused
        )
        clock.tick(60)
        
    logger.save()
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()

