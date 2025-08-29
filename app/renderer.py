# app/renderer.py

import pygame
import numpy as np
from vds.models.vessels.base_vessel import BaseVessel

class Renderer:
    """
    Handles the 2D visualization of the simulation using Pygame.
    """
    def __init__(self, width: int, height: int):
        pygame.init()
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Vessel Dynamics Simulator")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("Arial", 18)

        # Colors
        self.COLOR_WATER = (25, 45, 75)
        self.COLOR_SHIP = (255, 100, 50)
        self.COLOR_TRAIL = (200, 200, 200)
        self.COLOR_TEXT = (255, 255, 255)

        # Camera settings
        self.zoom = 0.2  # pixels per meter
        self.camera_offset_x = 0
        self.camera_offset_y = 0

    def _world_to_screen(self, x_world, y_world):
        """Converts world coordinates (meters, NED) to screen coordinates (pixels)."""
        # In NED, x is North (down on screen), y is East (right on screen)
        # We map world_y to screen_x and world_x to screen_y
        x_screen = (y_world * self.zoom) + self.camera_offset_x
        y_screen = (x_world * self.zoom) + self.camera_offset_y
        return int(x_screen), int(y_screen)

    def _update_camera(self, vessel_x, vessel_y):
        """Centers the camera on the vessel."""
        self.camera_offset_x = self.width / 2 - (vessel_y * self.zoom)  # y is East
        self.camera_offset_y = self.height / 2 - (vessel_x * self.zoom) # x is North

    def _draw_ship(self, vessel: BaseVessel):
        """Draws the vessel as an elongated hexagon on the screen."""
        x, y, _, _, _, psi = vessel.state.eta
        length = vessel.specs.length
        width = vessel.specs.width

        # Define ship shape vertices in body-frame (origin at CG, bow points along positive x-axis)
        ship_points_body = [
            (length / 2, 0),             # 1. Bow tip
            (length * 0.4, width / 2),   # 2. Starboard bow shoulder
            (-length * 0.4, width / 2),  # 3. Starboard stern shoulder
            (-length / 2, 0),            # 4. Stern tip
            (-length * 0.4, -width / 2), # 5. Port stern shoulder
            (length * 0.4, -width / 2),  # 6. Port bow shoulder
        ]

        # Rotate and translate points to world-frame
        # Note: psi is rotation from North axis, clockwise. Pygame rotation is counter-clockwise.
        # We handle this by transforming points manually.
        cos_psi = np.cos(psi)
        sin_psi = np.sin(psi)
        
        rotated_points = []
        for p_x, p_y in ship_points_body:
            # Standard 2D rotation from body to world (NED)
            world_x = x + (p_x * cos_psi - p_y * sin_psi) # North component
            world_y = y + (p_x * sin_psi + p_y * cos_psi) # East component
            rotated_points.append(self._world_to_screen(world_x, world_y))

        pygame.draw.polygon(self.screen, self.COLOR_SHIP, rotated_points)
        pygame.draw.aalines(self.screen, self.COLOR_SHIP, True, rotated_points, 1)

    def _draw_trail(self, trail_points: list):
        """Draws the vessel's past trajectory."""
        if len(trail_points) > 1:
            # trail_points are (x_north, y_east)
            screen_points = [self._world_to_screen(x, y) for x, y in trail_points]
            pygame.draw.aalines(self.screen, self.COLOR_TRAIL, False, screen_points)

    def _draw_info(self, vessel: BaseVessel, sim_time: float):
        """Displays simulation info on the screen."""
        data = vessel.get_state_summary()
        
        info_lines = [
            f"Time: {sim_time:.1f} s",
            f"HDG: {data['hdg_deg']:.1f}°",
            f"COG: {data['cog_deg']:.1f}°",
            f"SOG: {data['sog_kts']:.3f} kts",
            f"ROT: {data['rot_deg_min']:.1f}°/min",
            f"Pos (N, E): ({data['pos_y']:.1f}, {data['pos_x']:.1f}) m"
        ]
        
        y_offset = 10
        for line in info_lines:
            text_surface = self.font.render(line, True, self.COLOR_TEXT)
            self.screen.blit(text_surface, (10, y_offset))
            y_offset += 22
        
    def render(self, vessel: BaseVessel, trail_points: list, sim_time: float):
        """The main rendering function to be called in the simulation loop."""
        self._update_camera(vessel.state.eta[0], vessel.state.eta[1])
        
        self.screen.fill(self.COLOR_WATER)
        self._draw_trail(trail_points)
        self._draw_ship(vessel)
        self._draw_info(vessel, sim_time)

        pygame.display.flip()

    def handle_events(self) -> bool:
        """Handles Pygame events like closing the window. Returns False if quit."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
        return True

    def quit(self):
        """Shuts down Pygame."""
        pygame.quit()

