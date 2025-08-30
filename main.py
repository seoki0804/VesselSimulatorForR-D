# main.py

import pygame
import sys
import os
import numpy as np
import yaml
from app.renderer import Renderer
from vds.core.simulator import Simulator
from vds.utils.logger import DataLogger
from scenarios.scenario_loader import load_scenario
from vds.environment.wind import Wind
from vds.environment.current import Current
from vds.environment.waves import Waves
from vds.core.autopilot import Autopilot

def vessel_selection_loop(renderer, clock):
    """Loop for the initial scenario selection screen."""
    scenario_dir = 'scenarios'
    try:
        scenario_files = sorted([f for f in os.listdir(scenario_dir) if f.endswith('.yaml')])
        if not scenario_files:
            print(f"Error: No scenario files found in '{scenario_dir}/' directory.")
            return None
    except FileNotFoundError:
        print(f"Error: The '{scenario_dir}/' directory does not exist.")
        return None
        
    scenario_names = [s.replace('_', ' ').replace('.yaml', '').title() for s in scenario_files]
    selected_index = 0

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return None
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    selected_index = (selected_index - 1) % len(scenario_names)
                elif event.key == pygame.K_DOWN:
                    selected_index = (selected_index + 1) % len(scenario_names)
                elif event.key == pygame.K_RETURN:
                    return os.path.join(scenario_dir, scenario_files[selected_index])
        
        renderer.draw_vessel_selection_screen(scenario_names, selected_index)
        clock.tick(30)

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

def main():
    SCREEN_WIDTH, SCREEN_HEIGHT = 1280, 720
    renderer = Renderer(SCREEN_WIDTH, SCREEN_HEIGHT)
    clock = pygame.time.Clock()

    scenario_path = vessel_selection_loop(renderer, clock)
    if scenario_path is None:
        pygame.quit(); sys.exit()

    vessel, dynamics_model, geography, ais_targets, wind, current, waves, initial_control, waypoints = load_scenario(scenario_path)
    
    default_env = {'wind': wind, 'current': current, 'waves': waves}
    env_factors = settings_loop(renderer, clock, default_env)
    if env_factors is None:
        pygame.quit(); sys.exit()
    wind, current, waves = env_factors
    
    simulator = Simulator(vessel, dynamics_model, geography, ais_targets, wind, current, waves)
    simulator.waypoints = waypoints
    
    with open(scenario_path, 'r') as f:
        config = yaml.safe_load(f)
        simulator.show_obstacles = config.get('environment', {}).get('obstacles', {}).get('enabled', False)

    logger = DataLogger()
    autopilot = Autopilot()

    running = True
    dt = 0.1
    control = initial_control.copy()
    RUDDER_INCREMENT, RUDDER_MAX = 1.0, 35.0
    RPM_INCREMENT, RPM_MAX, RPM_MIN = 5.0, 300.0, -200.0
    
    camera_locked = True
    panning = False
    pan_start_pos = (0, 0)

    print("Controls: '0': Center Rudder | 'R': Reset | 'O': Obstacles | 'W': Water | 'P': Pause | 'C': Camera Lock | 'M': Minimap | 'A': Autopilot")

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            elif event.type == pygame.MOUSEWHEEL:
                camera_locked = False
                mouse_pos = pygame.mouse.get_pos()
                world_pos_before = (mouse_pos - renderer.offset) / renderer.zoom
                if event.y > 0: renderer.zoom *= 1.1
                else: renderer.zoom /= 1.1
                renderer.zoom = np.clip(renderer.zoom, 0.05, 5.0)
                renderer.offset = mouse_pos - world_pos_before * renderer.zoom
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:
                    panning, camera_locked = True, False
                    pan_start_pos = pygame.mouse.get_pos()
                    pan_start_offset = renderer.offset.copy()
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    panning = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_c: camera_locked = not camera_locked
                elif event.key == pygame.K_a:
                    simulator.autopilot_enabled = not simulator.autopilot_enabled
                    autopilot.reset()
                    print(f"Autopilot {'ENGAGED' if simulator.autopilot_enabled else 'DISENGAGED'}.")
                elif event.key == pygame.K_LEFT: control['rudder_angle'] = max(-RUDDER_MAX, control['rudder_angle'] - RUDDER_INCREMENT)
                elif event.key == pygame.K_RIGHT: control['rudder_angle'] = min(RUDDER_MAX, control['rudder_angle'] + RUDDER_INCREMENT)
                elif event.key == pygame.K_UP: control['rpm'] = min(RPM_MAX, control['rpm'] + RPM_INCREMENT)
                elif event.key == pygame.K_DOWN: control['rpm'] = max(RPM_MIN, control['rpm'] - RPM_INCREMENT)
                elif event.key == pygame.K_0 or event.key == pygame.K_KP0: control['rudder_angle'] = 0.0
                elif event.key == pygame.K_r: simulator.reset(); control = initial_control.copy(); autopilot.reset()
                elif event.key == pygame.K_o: simulator.show_obstacles = not simulator.show_obstacles
                elif event.key == pygame.K_w: simulator.show_water_depth = not simulator.show_water_depth
                elif event.key == pygame.K_p: simulator.is_paused = not simulator.is_paused
                elif event.key == pygame.K_m: simulator.show_minimap = not simulator.show_minimap

        if panning:
            mouse_delta = np.array(pygame.mouse.get_pos()) - np.array(pan_start_pos)
            renderer.offset = pan_start_offset + mouse_delta
        elif camera_locked:
            renderer.recenter(simulator.vessel.state.eta[:2])

        if simulator.autopilot_enabled and simulator.current_waypoint_index < len(simulator.waypoints):
            target_pos = np.array(simulator.waypoints[simulator.current_waypoint_index]['position'])
            control['rudder_angle'] = autopilot.calculate_rudder_angle(
                simulator.vessel.state.eta[:2], simulator.vessel.state.eta[5], target_pos, dt)

        if not simulator.collision_detected:
            if not simulator.is_paused:
                logger.log(simulator, control)
            simulator.step(dt, control)

        renderer.render(simulator, control)
        
        clock.tick(60)
        
    logger.save()
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()

