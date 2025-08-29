# app/renderer.py

import pygame
import numpy as np
from vds.models.vessels.base_vessel import BaseVessel
from vds.environment.geography import Geography

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
        self.COLOR_WATER_DEEP = (25, 45, 75)
        self.COLOR_WATER_MID = (35, 65, 105)
        self.COLOR_WATER_SHALLOW = (55, 95, 145)
        self.COLOR_LAND = (118, 152, 93)
        self.COLOR_SHIP = (255, 100, 50)
        self.COLOR_TRAIL = (200, 200, 200)
        self.COLOR_TEXT = (255, 255, 255)

        # Camera settings
        self.zoom = 1.0  # pixels per meter
        self.camera_offset_x = 0
        self.camera_offset_y = 0

    def _world_to_screen(self, x_world, y_world):
        """Converts world coordinates (meters, NED) to screen coordinates (pixels)."""
        x_screen = (y_world * self.zoom) + self.camera_offset_x
        y_screen = (x_world * self.zoom) + self.camera_offset_y
        return int(x_screen), int(y_screen)

    def _update_camera(self, vessel_x, vessel_y):
        """Centers the camera on the vessel."""
        self.camera_offset_x = self.width / 2 - (vessel_y * self.zoom)
        self.camera_offset_y = self.height / 2 - (vessel_x * self.zoom)

    def _draw_geography(self, geography: Geography):
        """Draws the bathymetry grid."""
        cell_size_px = geography.cell_size * self.zoom
        if cell_size_px < 1:
            return # Don't draw if cells are too small to see

        for r in range(geography.height):
            for c in range(geography.width):
                depth = geography.grid_data[r, c]
                
                # Determine color based on depth
                if depth >= 0:
                    color = self.COLOR_LAND
                elif -20 < depth < 0:
                    color = self.COLOR_WATER_SHALLOW
                elif -40 < depth <= -20:
                    color = self.COLOR_WATER_MID
                else:
                    color = self.COLOR_WATER_DEEP

                world_x = geography.origin_x + r * geography.cell_size
                world_y = geography.origin_y + c * geography.cell_size
                
                screen_pos = self._world_to_screen(world_x, world_y)
                
                # Optimization: only draw if on screen
                if -cell_size_px < screen_pos[0] < self.width and \
                   -cell_size_px < screen_pos[1] < self.height:
                    pygame.draw.rect(self.screen, color, 
                                     (screen_pos[0], screen_pos[1], 
                                      int(cell_size_px) + 1, int(cell_size_px) + 1))

    def _draw_ship(self, vessel: BaseVessel):
        """Draws the vessel as an elongated hexagon on the screen."""
        x, y, _, _, _, psi = vessel.state.eta
        length = vessel.specs.length
        width = vessel.specs.width

        ship_points_body = [
            (length / 2, 0), (length * 0.4, width / 2),
            (-length * 0.4, width / 2), (-length / 2, 0),
            (-length * 0.4, -width / 2), (length * 0.4, -width / 2),
        ]

        cos_psi = np.cos(psi)
        sin_psi = np.sin(psi)
        
        rotated_points = []
        for p_x, p_y in ship_points_body:
            world_x = x + (p_x * cos_psi - p_y * sin_psi)
            world_y = y + (p_x * sin_psi + p_y * cos_psi)
            rotated_points.append(self._world_to_screen(world_x, world_y))

        pygame.draw.polygon(self.screen, self.COLOR_SHIP, rotated_points)
        
    def _draw_trail(self, trail_points: list):
        """Draws the vessel's past trajectory."""
        if len(trail_points) > 1:
            screen_points = [self._world_to_screen(x, y) for x, y in trail_points]
            pygame.draw.aalines(self.screen, self.COLOR_TRAIL, False, screen_points)

    def _draw_info(self, vessel: BaseVessel, sim_time: float):
        """Displays simulation info on the screen."""
        data = vessel.get_state_summary()
        info_lines = [
            f"Time: {sim_time:.1f} s",
            f"HDG: {data['hdg_deg']:.1f}°", f"COG: {data['cog_deg']:.1f}°",
            f"SOG: {data['sog_kts']:.3f} kts", f"ROT: {data['rot_deg_min']:.1f}°/min",
            f"Pos (N, E): ({data['pos_y']:.1f}, {data['pos_x']:.1f}) m"
        ]
        y_offset = 10
        for line in info_lines:
            text_surface = self.font.render(line, True, self.COLOR_TEXT)
            self.screen.blit(text_surface, (10, y_offset))
            y_offset += 22
        
    def render(self, vessel: BaseVessel, trail_points: list, sim_time: float, geography: Geography):
        """The main rendering function to be called in the simulation loop."""
        self._update_camera(vessel.state.eta[0], vessel.state.eta[1])
        
        self.screen.fill(self.COLOR_WATER_DEEP)
        self._draw_geography(geography)
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

