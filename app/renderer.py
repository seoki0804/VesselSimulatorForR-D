# app/renderer.py

import pygame
import numpy as np
from collections import deque
from vds.models.vessels.base_vessel import BaseVessel
from vds.environment.geography import Geography
from vds.data_handler.ais_parser import AISTarget
from vds.environment.wind import Wind
from vds.environment.current import Current
from vds.environment.waves import Waves

class Renderer:
    def __init__(self, width: int, height: int):
        pygame.init()
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Vessel Dynamics Simulator")
        self.font = pygame.font.Font(None, 24)
        self.title_font = pygame.font.Font(None, 50)
        self.pause_font = pygame.font.Font(None, 74)
        self.zoom = 0.5
        self.offset = np.array([width / 2, height / 2], dtype=float)

    def recenter(self, vessel_pos_world: np.ndarray):
        """Recenter the camera on the vessel."""
        transformed_vessel_pos = np.array([vessel_pos_world[1], -vessel_pos_world[0]])
        self.offset = np.array([self.width / 2, self.height / 2], dtype=float) - transformed_vessel_pos * self.zoom

    def draw_vessel_selection_screen(self, scenarios: list, selected_index: int):
        self.screen.fill((22, 44, 77))
        title_surf = self.title_font.render("Select Scenario", True, (255, 255, 255))
        self.screen.blit(title_surf, (self.width / 2 - title_surf.get_width() / 2, 100))
        for i, scenario_name in enumerate(scenarios):
            color = (255, 255, 0) if i == selected_index else (200, 200, 200)
            text_surf = self.font.render(scenario_name, True, color)
            self.screen.blit(text_surf, (self.width / 2 - text_surf.get_width() / 2, 200 + i * 40))
        inst_surf = self.font.render("Use UP/DOWN arrows to select, ENTER to continue", True, (150, 150, 150))
        self.screen.blit(inst_surf, (self.width / 2 - inst_surf.get_width() / 2, 500))
        pygame.display.flip()

    def draw_settings_screen(self, settings: dict, active_field: str):
        self.screen.fill((22, 44, 77))
        title_surf = self.title_font.render("Environment Settings", True, (255, 255, 255))
        self.screen.blit(title_surf, (self.width / 2 - title_surf.get_width() / 2, 100))
        fields = ["wind_speed", "wind_dir", "current_speed", "current_dir", "waves_h", "waves_dir"]
        labels = ["Wind Speed (kts):", "Wind Dir (deg):", "Current Speed (kts):", "Current Dir (deg):", "Waves Hs (m):", "Waves Dir (deg):"]
        for i, (field, label) in enumerate(zip(fields, labels)):
            label_surf = self.font.render(label, True, (200, 200, 200))
            self.screen.blit(label_surf, (400, 200 + i * 40))
            input_rect = pygame.Rect(600, 200 + i * 40, 140, 32)
            color = (255, 255, 0) if active_field == field else (255, 255, 255)
            pygame.draw.rect(self.screen, color, input_rect, 2)
            text_surf = self.font.render(settings[field], True, (255, 255, 255))
            self.screen.blit(text_surf, (input_rect.x + 5, input_rect.y + 5))
        inst_surf = self.font.render("Use TAB to switch fields, ENTER to start simulation", True, (150, 150, 150))
        self.screen.blit(inst_surf, (self.width / 2 - inst_surf.get_width() / 2, 500))
        pygame.display.flip()

    def _world_to_screen(self, pos: np.ndarray) -> tuple[int, int]:
        transformed_pos = np.array([pos[1], -pos[0]])
        screen_pos = (transformed_pos * self.zoom) + self.offset
        return int(screen_pos[0]), int(screen_pos[1])

    def render(self, simulator, control: dict):
        """Main rendering function. Now takes the simulator object directly."""
        self.screen.fill((22, 44, 77))
        vessel = simulator.vessel

        self._draw_geography(simulator.geography, simulator.show_obstacles, simulator.show_water_depth, vessel)
        # FIXED: Pass the current waypoint index from the simulator
        self._draw_waypoints(getattr(simulator, 'waypoints', []), simulator.current_waypoint_index)
        self._draw_track(simulator.track_history)
        self._draw_ais_targets(simulator.ais_targets)
        self._draw_vessel(vessel)
        self._draw_hud(vessel, control, simulator)
        self._draw_6dof_indicator(vessel)
        
        if simulator.show_minimap:
            self._draw_minimap(vessel, simulator.geography, simulator.ais_targets)
        if simulator.is_paused:
            self._draw_pause_overlay()
        pygame.display.flip()

    def _draw_minimap(self, vessel, geography, ais_targets):
        map_w, map_h = 250, 200
        map_x, map_y = self.width - map_w - 10, 10
        map_rect = pygame.Rect(map_x, map_y, map_w, map_h)
        pygame.draw.rect(self.screen, (0, 0, 0, 150), map_rect)
        pygame.draw.rect(self.screen, (200, 200, 200), map_rect, 1)
        if geography.map_width == 0 or geography.map_height == 0: return
        scale = min(map_w / geography.map_width, map_h / geography.map_height)
        def world_to_mini(pos):
            mini_pos_x = map_x + pos[1] * scale
            mini_pos_y = map_y + pos[0] * scale
            return int(mini_pos_x), int(mini_pos_y)
        vessel_pos = world_to_mini(vessel.state.eta[:2])
        pygame.draw.circle(self.screen, (255, 165, 0), vessel_pos, 4)
        for target in ais_targets:
            target_pos = world_to_mini([target.state.x, target.state.y])
            pygame.draw.circle(self.screen, (150, 150, 150), target_pos, 3)

    def _draw_6dof_indicator(self, vessel):
        ind_x, ind_y, ind_w, ind_h = self.width - 260, self.height - 110, 250, 100
        pygame.draw.rect(self.screen, (0, 0, 0, 150), (ind_x, ind_y, ind_w, ind_h))
        pygame.draw.rect(self.screen, (200, 200, 200), (ind_x, ind_y, ind_w, ind_h), 1)
        v, p, q = vessel.state.nu[1], vessel.state.nu[3], vessel.state.nu[4]
        def draw_bar(label, value, y_pos, max_val):
            label_surf = self.font.render(label, True, (200, 200, 200))
            self.screen.blit(label_surf, (ind_x + 10, y_pos + 2))
            bar_bg_rect = pygame.Rect(ind_x + 70, y_pos, 170, 20)
            pygame.draw.rect(self.screen, (50, 50, 50), bar_bg_rect)
            bar_width = np.clip(abs(value) / max_val, 0, 1) * 85
            bar_color = (255, 100, 100) if value < 0 else (100, 255, 100)
            if value > 0: pygame.draw.rect(self.screen, bar_color, (bar_bg_rect.centerx, y_pos, bar_width, 20))
            else: pygame.draw.rect(self.screen, bar_color, (bar_bg_rect.centerx - bar_width, y_pos, bar_width, 20))
            pygame.draw.line(self.screen, (255, 255, 255), (bar_bg_rect.centerx, y_pos), (bar_bg_rect.centerx, y_pos + 20))
        draw_bar("Sway", v, ind_y + 10, 2.0)
        draw_bar("Roll", np.degrees(p), ind_y + 40, 5.0)
        draw_bar("Pitch", np.degrees(q), ind_y + 70, 5.0)

    def _draw_hud(self, vessel, control, simulator):
        rudder_angle = control.get('rudder_angle', 0.0)
        if rudder_angle < -0.1: rudder_color = (255, 100, 100)
        elif rudder_angle > 0.1: rudder_color = (100, 255, 100)
        else: rudder_color = (255, 255, 255)
        
        info_texts = [
            f"Time: {simulator.time:.1f}s", f"HDG: {vessel.heading:.1f}° | COG: {vessel.cog:.1f}°",
            f"SOG: {vessel.sog:.2f} kts", f"ROT: {vessel.rot:.1f}°/min",
            f"Pos N: {vessel.state.eta[0]:.1f} E: {vessel.state.eta[1]:.1f}", f"Draft: {vessel.specs.draft:.1f} m"
        ]
        
        wind, current, waves = simulator.wind, simulator.current, simulator.waves
        if wind and wind.speed > 0: info_texts.append(f"Wind: {wind.speed:.1f} kts @ {wind.direction}°")
        if current and current.speed > 0: info_texts.append(f"Current: {current.speed:.1f} kts @ {current.direction}°")
        if waves and waves.significant_height > 0: info_texts.append(f"Waves: Hs={waves.significant_height:.1f}m @ {waves.direction}°")

        for i, text in enumerate(info_texts):
            surface = self.font.render(text, True, (255, 255, 255))
            self.screen.blit(surface, (10, 10 + i * 25))
            
        rpm_text = f"RPM: {control.get('rpm', 0.0):.0f}"
        rudder_text = f"Rudder: {rudder_angle:.1f}°"
        rpm_surf = self.font.render(rpm_text, True, (255, 255, 255))
        rudder_surf = self.font.render(rudder_text, True, rudder_color)
        padding = 40
        total_width = rpm_surf.get_width() + padding + rudder_surf.get_width()
        start_x = self.width / 2 - total_width / 2
        y_pos = self.height - rpm_surf.get_height() - 10
        self.screen.blit(rpm_surf, (start_x, y_pos))
        self.screen.blit(rudder_surf, (start_x + rpm_surf.get_width() + padding, y_pos))
        
        toggle_texts = [
            f"[A] Autopilot: {'ON' if simulator.autopilot_enabled else 'OFF'}",
            f"[M] Minimap: {'ON' if simulator.show_minimap else 'OFF'}",
            f"[O] Obstacles: {'ON' if simulator.show_obstacles else 'OFF'}",
            f"[W] Water Depth: {'ON' if simulator.show_water_depth else 'OFF'}"
        ]
        for i, text in enumerate(toggle_texts):
            surface = self.font.render(text, True, (255, 255, 0))
            y_pos = self.height - (len(toggle_texts) - i) * 30
            self.screen.blit(surface, (10, y_pos))

    def _draw_waypoints(self, waypoints: list, current_index: int):
        for i, wp in enumerate(waypoints):
            pos = np.array(wp['position'])
            name = wp['name']
            screen_pos = self._world_to_screen(pos)
            color = (0, 255, 0) if i == current_index else (255, 0, 255)
            pygame.draw.circle(self.screen, color, screen_pos, 12, 2)
            text_surf = self.font.render(name, True, color)
            self.screen.blit(text_surf, (screen_pos[0] + 15, screen_pos[1] - 10))

    def _draw_pause_overlay(self):
        overlay = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 128))
        self.screen.blit(overlay, (0, 0))
        pause_text = self.pause_font.render("PAUSED", True, (255, 255, 0))
        text_rect = pause_text.get_rect(center=(self.width / 2, self.height / 2))
        self.screen.blit(pause_text, text_rect)

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

    def _get_depth_color(self, depth: float, vessel_draft: float):
        depth = abs(depth)
        shallow_limit = vessel_draft * 1.2
        deep_limit = vessel_draft * 2.0
        if depth < shallow_limit: return (217, 102, 79)
        elif depth > deep_limit: return (22, 85, 142)
        else: return (71, 161, 201)

