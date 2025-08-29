# main.py

import pygame
import sys
import numpy as np

from app.renderer import Renderer
from vds.core.simulator import Simulator
from vds.models.vessels.base_vessel import BaseVessel, VesselSpecifications, VesselState
from vds.models.dynamics.simple_3dof_model import Simple3DOFModel
from vds.environment.geography import Geography

def main():
    # --- Simulation Setup ---
    geography = Geography.from_csv('data/bathymetry/sample_depth.csv', cell_size=20)
    geography.add_obstacle(center_x=150, center_y=-100, radius=25)
    geography.add_obstacle(center_x=300, center_y=-250, radius=40)
    print("Geography and obstacle data loaded.\n")

    specs = VesselSpecifications(
        loa=150.0,
        beam=25.0,
        draft=10.0,
        mass=17000e3,
        inertia_z=1.77e9
    )
    initial_state = VesselState(
        nu=np.array([5.0, 0, 0, 0, 0, 0])
    )
    vessel = BaseVessel(specs, initial_state)

    mass = specs.mass + 6.0e6
    Iz = specs.inertia_z + 2.0e9
    dynamics_model = Simple3DOFModel(vessel_mass=mass, inertia_z=Iz, vessel_draft=specs.draft)

    simulator = Simulator(vessel, dynamics_model, geography)

    # --- Pygame Setup ---
    SCREEN_WIDTH = 1280
    SCREEN_HEIGHT = 720
    renderer = Renderer(SCREEN_WIDTH, SCREEN_HEIGHT)

    # --- Main Loop ---
    running = True
    clock = pygame.time.Clock()
    dt = 0.1

    control = {'rpm': 100.0, 'rudder_angle': 0.0}
    RUDDER_INCREMENT = 1.0
    RUDDER_MAX = 35.0
    RPM_INCREMENT = 10.0
    RPM_MAX = 200.0

    print("Starting simulation... Use Arrow Keys to control the vessel.")
    print("UP/DOWN: RPM | LEFT/RIGHT: Rudder | '0' KEY: Center Rudder | 'R' KEY: Reset")

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
                    control['rpm'] = max(0, control['rpm'] - RPM_INCREMENT)
                elif event.key == pygame.K_0 or event.key == pygame.K_KP0:
                    control['rudder_angle'] = 0.0
                elif event.key == pygame.K_r: # ADDED: Reset functionality
                    simulator.reset()
                    control = {'rpm': 100.0, 'rudder_angle': 0.0}

        if simulator.collision_detected:
            pass
        else:
            simulator.step(dt, control)

        renderer.render(simulator.vessel, geography, control, simulator.time)

        clock.tick(60)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()

