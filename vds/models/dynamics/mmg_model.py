# vds/models/dynamics/mmg_model.py

import numpy as np
import json
from .base_model import BaseDynamicsModel
from vds.models.vessels.base_vessel import VesselState
from vds.environment.wind import Wind
from vds.environment.current import Current
from vds.environment.waves import Waves

class MMGModel(BaseDynamicsModel):
    def __init__(self, vessel_spec, hydro_params_path):
        with open(hydro_params_path, 'r') as f:
            self.p = json.load(f)
        self.spec = vessel_spec
        self.mass = float(self.spec.mass) # Ensure mass is a float
        self.Iz = float(self.spec.inertia_z) # Ensure inertia is a float
        self.L = self.p['Lpp']
        self.d = self.p['d']
        self.rho = self.p['rho']
        self.rho_air = 1.225

    def calculate_forces(self, state: VesselState, control: dict, depth: float = 1000.0, wind: Wind = None, current: Current = None, waves: Waves = None) -> np.ndarray:
        u_abs, v_abs, _, _, _, r = state.nu
        psi = state.eta[5]

        u_c, v_c = 0, 0
        if current:
            current_speed_ms = current.speed * 0.514444
            current_dir_rad = np.radians(current.direction)
            u_c = current_speed_ms * np.cos(current_dir_rad - psi)
            v_c = current_speed_ms * np.sin(current_dir_rad - psi)
        
        u = u_abs - u_c
        v = v_abs - v_c

        rpm = control.get('rpm', 0)
        n_rps = rpm / 60.0
        rudder_angle_rad = np.radians(control.get('rudder_angle', 0))

        if abs(u) < 0.1 and abs(rpm) < 1.0: return np.zeros(6)
        U = np.sqrt(u**2 + v**2)
        v_prime = v / U if U > 0 else 0
        r_prime = r * self.L / U if U > 0 else 0
        w_p = self.p['w_P0']
        if n_rps >= 0:
            J = u * (1 - w_p) / (n_rps * self.p['D_P']) if n_rps > 0 else 0
            Kt = self.p['k_0'] + self.p['k_1'] * J + self.p['k_2'] * J**2
            T = self.rho * (n_rps**2) * (self.p['D_P']**4) * Kt
        else:
            Kt_astern = -0.4 * self.p['k_0'] 
            T = self.rho * (n_rps**2) * (self.p['D_P']**4) * Kt_astern
        if n_rps > 0:
            C1 = self.p['kappa'] * (2 * Kt) / (J**2) if J > 0 else self.p['kappa'] * 2 * self.p['k_0']
            u_r_factor = np.sqrt(1 + C1)
        else: u_r_factor = 1.0
        u_r = self.p['epsilon'] * u * (1-w_p) * u_r_factor
        v_r = v + r * self.p['x_R_prime'] * self.L
        U_R_sq = u_r**2 + v_r**2
        alpha_R = rudder_angle_rad - np.arctan2(v_r, u_r)
        f_alpha = (8.0 * self.p['Lambda']) / (self.p['Lambda'] + 2.25)
        F_N = 0.5 * self.rho * self.p['A_R'] * U_R_sq * f_alpha * np.sin(alpha_R)
        N_r_prime_damped = self.p['N_r_prime'] * (1.0 + 3.0 * abs(r_prime))
        X_H_prime = self.p['R_0_prime'] + self.p['X_vv_prime'] * v_prime**2
        Y_H_prime = self.p['Y_v_prime'] * v_prime + self.p['Y_r_prime'] * r_prime
        N_H_prime = self.p['N_v_prime'] * v_prime + N_r_prime_damped * r_prime
        X_P = T
        X_R = -(1 - self.p['a_H']) * F_N * np.sin(rudder_angle_rad)
        Y_R = -(1 + self.p['a_H']) * F_N * np.cos(rudder_angle_rad)
        N_R = -(self.p['x_R_prime'] + self.p['a_H'] * self.p['x_H_prime']) * self.L * F_N * np.cos(rudder_angle_rad)
        hull_resistance_X = X_H_prime * 0.5 * self.rho * self.L * self.d * U**2
        hull_sway_Y = Y_H_prime * 0.5 * self.rho * self.L * self.d * U**2
        hull_yaw_N = N_H_prime * 0.5 * self.rho * self.L**2 * self.d * U**2

        X_W, Y_W, N_W = 0, 0, 0
        if wind:
            wind_speed = wind.speed * 0.514444
            wind_dir_rad = np.radians(wind.direction) + np.pi
            u_w = wind_speed * np.cos(wind_dir_rad - psi) - u_abs
            v_w = wind_speed * np.sin(wind_dir_rad - psi) - v_abs
            V_wr = np.sqrt(u_w**2 + v_w**2)
            alpha_wr = np.arctan2(-v_w, -u_w)
            C_X = -0.6 * np.cos(alpha_wr)
            C_Y = 0.9 * np.sin(alpha_wr)
            C_N = 0.15 * np.sin(2 * alpha_wr)
            X_W = 0.5 * self.rho_air * V_wr**2 * self.spec.wind_area_transverse * C_X
            Y_W = 0.5 * self.rho_air * V_wr**2 * self.spec.wind_area_longitudinal * C_Y
            N_W = 0.5 * self.rho_air * V_wr**2 * self.spec.wind_area_longitudinal * self.L * C_N
            
        X_WV, Y_WV, N_WV = 0, 0, 0
        if waves:
            wave_dir_rad = np.radians(waves.direction) + np.pi
            relative_wave_angle = (wave_dir_rad - psi + np.pi) % (2*np.pi) - np.pi
            C_X_WV = -0.05 * waves.significant_height**2 * (1 - np.cos(relative_wave_angle))
            C_Y_WV = 0.2 * waves.significant_height**2 * np.sin(2 * relative_wave_angle)
            C_N_WV = 0.03 * waves.significant_height**2 * np.sin(relative_wave_angle)
            X_WV = C_X_WV * self.rho * self.L * 9.81
            Y_WV = C_Y_WV * self.rho * self.L * 9.81
            N_WV = C_N_WV * self.rho * self.L**2 * 9.81
            
        X = -np.sign(u) * hull_resistance_X + X_P + X_R + X_W + X_WV
        Y = hull_sway_Y + Y_R + Y_W + Y_WV
        N = hull_yaw_N + N_R + N_W + N_WV
        
        nu_dot = np.zeros(6)
        added_mass_x = 0.15 * self.mass
        added_mass_y = 0.8 * self.mass
        added_inertia_z = 0.1 * self.Iz
        nu_dot[0] = X / (self.mass + added_mass_x)
        nu_dot[1] = Y / (self.mass + added_mass_y)
        nu_dot[5] = N / (self.Iz + added_inertia_z)
        return nu_dot

