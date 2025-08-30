# app/renderer.py

import pygame
import numpy as np
from collections import deque
from vds.models.vessels.base_vessel import BaseVessel
from vds.environment.geography import Geography
from vds.data_handler.ais_parser import AISTarget

class Renderer:
    def __init__(self, width: int, height: int):
        pygame.init()
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Vessel Dynamics Simulator")
        self.font = pygame.font.Font(None, 28)
        self.zoom = 0.5
        self.offset = np.array([width / 2, height / 2], dtype=float)

    def _world_to_screen(self, pos: np.ndarray) -> tuple[int, int]:
        transformed_pos = np.array([pos[1], -pos[0]])
        screen_pos = (transformed_pos * self.zoom) + self.offset
        return int(screen_pos[0]), int(screen_pos[1])

    def render(self, vessel: BaseVessel, geography: Geography, control: dict, time: float, ais_targets: list[AISTarget], track_history: deque, show_obstacles: bool, show_water: bool):
        self.screen.fill((22, 44, 77))
        transformed_vessel_pos = np.array([vessel.state.eta[1], -vessel.state.eta[0]])
        self.offset = np.array([self.width / 2, self.height / 2], dtype=float) - transformed_vessel_pos * self.zoom

        # Pass the vessel object to _draw_geography to access draft info
        self._draw_geography(geography, show_obstacles, show_water, vessel)
        self._draw_track(track_history)
        self._draw_ais_targets(ais_targets)
        self._draw_vessel(vessel)
        self._draw_hud(vessel, control, time, show_obstacles, show_water)

        pygame.display.flip()

    def _draw_track(self, track_history: deque):
        if len(track_history) < 2: return
        screen_points = [self._world_to_screen(p) for p in track_history]
        pygame.draw.aalines(self.screen, (200, 200, 255), False, screen_points, 1)

    def _draw_geography(self, geography: Geography, show_obstacles: bool, show_water: bool, vessel: BaseVessel):
        if show_water:
            cell_size_screen = int(geography.cell_size * self.zoom)
            if cell_size_screen >= 2:
                for i in range(geography.grid_height):
                    for j in range(geography.grid_width):
                        depth = geography.depth_data[i, j]
                        # Pass vessel's draft to determine the water color
                        color = self._get_depth_color(depth, vessel.specs.draft)
                        world_pos = np.array([j * geography.cell_size, i * geography.cell_size])
                        screen_pos = self._world_to_screen(world_pos)
                        pygame.draw.rect(self.screen, color, (screen_pos[0], screen_pos[1], cell_size_screen, cell_size_screen))

        if show_obstacles:
            for obs in geography.obstructions:
                screen_pos = self._world_to_screen(obs.position)
                screen_radius = int(obs.radius * self.zoom)
                if screen_radius > 1:
                    pygame.draw.circle(self.screen, (139, 69, 19), screen_pos, screen_radius)
    
    def _draw_ais_targets(self, ais_targets: list[AISTarget]):
        for target in ais_targets:
            pos = np.array([target.state.x, target.state.y])
            cog_rad = target.state.cog_rad - np.pi / 2
            target_size = 15 * self.zoom
            if target_size < 2: continue
            points = [(target_size, 0),(-target_size / 2, -target_size / 3),(-target_size / 2, target_size / 3),]
            rot_matrix = np.array([[np.cos(cog_rad), -np.sin(cog_rad)], [np.sin(cog_rad), np.cos(cog_rad)]])
            rotated_points = [rot_matrix @ p for p in points]
            screen_pos = self._world_to_screen(pos)
            screen_points = [(p[0] + screen_pos[0], p[1] + screen_pos[1]) for p in rotated_points]
            pygame.draw.polygon(self.screen, (150, 150, 150), screen_points)

    def _draw_vessel(self, vessel: BaseVessel):
        pos = vessel.state.eta[:2]
        heading_rad = vessel.state.eta[5] - np.pi / 2
        vessel_len = vessel.specs.loa * self.zoom
        vessel_width = vessel.specs.beam * self.zoom
        points = [(vessel_len / 2, 0),(vessel_len / 4, vessel_width / 2),(-vessel_len / 2, vessel_width / 2),(-vessel_len / 2, -vessel_width / 2),(vessel_len / 4, -vessel_width / 2),]
        rot_matrix = np.array([[np.cos(heading_rad), -np.sin(heading_rad)],[np.sin(heading_rad), np.cos(heading_rad)]])
        rotated_points = [rot_matrix @ p for p in points]
        screen_pos = self._world_to_screen(pos)
        screen_points = [(p[0] + screen_pos[0], p[1] + screen_pos[1]) for p in rotated_points]
        pygame.draw.polygon(self.screen, (255, 165, 0), screen_points)
        pygame.draw.polygon(self.screen, (255, 255, 255), screen_points, 1)

    def _draw_hud(self, vessel: BaseVessel, control: dict, time: float, show_obstacles: bool, show_water: bool):
        rudder_angle = control.get('rudder_angle', 0.0)
        if rudder_angle < -0.1: rudder_color = (255, 100, 100)
        elif rudder_angle > 0.1: rudder_color = (100, 255, 100)
        else: rudder_color = (255, 255, 255)
        info_texts = [f"Time: {time:.1f}s",f"HDG: {vessel.heading:.1f}°",f"COG: {vessel.cog:.1f}°",f"SOG: {vessel.sog:.2f} kts",f"ROT: {vessel.rot:.1f}°/min",f"Pos: ({vessel.state.eta[0]:.1f}, {vessel.state.eta[1]:.1f}) m"]
        for i, text in enumerate(info_texts):
            surface = self.font.render(text, True, (255, 255, 255))
            self.screen.blit(surface, (10, 10 + i * 25))
        control_texts = [f"RPM: {control.get('rpm', 0.0):.0f}",f"Rudder: {rudder_angle:.1f}°"]
        for i, text in enumerate(control_texts):
            color = (255, 255, 255) if i == 0 else rudder_color
            surface = self.font.render(text, True, color)
            self.screen.blit(surface, (self.width - surface.get_width() - 10, 10 + i * 25))
        
        obs_text = f"[O] Obstacles: {'ON' if show_obstacles else 'OFF'}"
        water_text = f"[W] Water Depth: {'ON' if show_water else 'OFF'}"
        obs_surface = self.font.render(obs_text, True, (255, 255, 0))
        water_surface = self.font.render(water_text, True, (255, 255, 0))
        self.screen.blit(obs_surface, (self.width - obs_surface.get_width() - 10, self.height - obs_surface.get_height() - 40))
        self.screen.blit(water_surface, (self.width - water_surface.get_width() - 10, self.height - water_surface.get_height() - 10))

    def _get_depth_color(self, depth: float, vessel_draft: float):
        """Returns a color based on water depth relative to the vessel's draft."""
        depth = abs(depth)
        
        shallow_limit = vessel_draft * 1.2
        deep_limit = vessel_draft * 2.0

        if depth < shallow_limit:
            return (217, 102, 79)  # Reddish for Shallow Water (Danger)
        elif depth > deep_limit:
            return (22, 85, 142)    # Dark Blue for Deep Water (Safe)
        else:
            return (71, 161, 201)   # Normal Blue for Transitional Water

