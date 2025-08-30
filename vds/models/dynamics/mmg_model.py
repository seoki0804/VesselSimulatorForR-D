# vds/models/dynamics/mmg_model.py

import numpy as np
import json
from .base_model import BaseDynamicsModel
from vds.models.vessels.base_vessel import VesselState

class MMGModel(BaseDynamicsModel):
    """
    A more realistic 3-DOF dynamics model based on the MMG standard model.
    This model separates forces for Hull, Propeller, and Rudder.
    """
    def __init__(self, vessel_spec, hydro_params_path):
        with open(hydro_params_path, 'r') as f:
            self.p = json.load(f)

        self.spec = vessel_spec
        self.mass = self.spec.mass
        self.Iz = self.spec.inertia_z
        self.L = self.p['Lpp']
        self.d = self.p['d']
        self.rho = self.p['rho']

    def calculate_forces(self, state: VesselState, control: dict, depth: float = 1000.0) -> np.ndarray:
        u, v, _, _, _, r = state.nu
        rpm = control.get('rpm', 0)
        n_rps = rpm / 60.0
        rudder_angle_rad = np.radians(control.get('rudder_angle', 0))

        if abs(u) < 0.1 and abs(rpm) < 1.0:
            return np.zeros(6)

        U = np.sqrt(u**2 + v**2)
        v_prime = v / U if U > 0 else 0
        r_prime = r * self.L / U if U > 0 else 0
        
        # Propeller calculations
        w_p = self.p['w_P0']
        
        if n_rps >= 0:
            J = u * (1 - w_p) / (n_rps * self.p['D_P']) if n_rps > 0 else 0
            Kt = self.p['k_0'] + self.p['k_1'] * J + self.p['k_2'] * J**2
            T = self.rho * (n_rps**2) * (self.p['D_P']**4) * Kt
        else:
            Kt_astern = -0.4 * self.p['k_0'] 
            T = self.rho * (n_rps**2) * (self.p['D_P']**4) * Kt_astern

        # Rudder inflow velocity
        if n_rps > 0:
            C1 = self.p['kappa'] * (2 * Kt) / (J**2) if J > 0 else self.p['kappa'] * 2 * self.p['k_0']
            u_r_factor = np.sqrt(1 + C1)
        else:
            u_r_factor = 1.0
        
        u_r = self.p['epsilon'] * u * (1-w_p) * u_r_factor
        v_r = v + r * self.p['x_R_prime'] * self.L
        U_R_sq = u_r**2 + v_r**2
        alpha_R = rudder_angle_rad - np.arctan2(v_r, u_r)
        
        f_alpha = (8.0 * self.p['Lambda']) / (self.p['Lambda'] + 2.25)
        F_N = 0.5 * self.rho * self.p['A_R'] * U_R_sq * f_alpha * np.sin(alpha_R)

        # Hull forces (non-dimensional)
        N_r_prime_damped = self.p['N_r_prime'] * (1.0 + 3.0 * abs(r_prime))
        X_H_prime = self.p['R_0_prime'] + self.p['X_vv_prime'] * v_prime**2 + \
                    self.p['X_vr_prime'] * v_prime * r_prime + self.p['X_rr_prime'] * r_prime**2
        Y_H_prime = self.p['Y_v_prime'] * v_prime + self.p['Y_r_prime'] * r_prime
        N_H_prime = self.p['N_v_prime'] * v_prime + N_r_prime_damped * r_prime

        # Propeller forces (dimensional)
        X_P = T
        
        # Rudder forces (dimensional)
        X_R = -(1 - self.p['a_H']) * F_N * np.sin(rudder_angle_rad)
        Y_R = -(1 + self.p['a_H']) * F_N * np.cos(rudder_angle_rad)
        N_R = -(self.p['x_R_prime'] + self.p['a_H'] * self.p['x_H_prime']) * self.L * F_N * np.cos(rudder_angle_rad)

        # Total forces (dimensional)
        # --- FIXED: Hull resistance now correctly opposes the direction of motion ---
        hull_resistance_X = X_H_prime * 0.5 * self.rho * self.L * self.d * U**2
        hull_sway_Y = Y_H_prime * 0.5 * self.rho * self.L * self.d * U**2
        hull_yaw_N = N_H_prime * 0.5 * self.rho * self.L**2 * self.d * U**2

        X = -np.sign(u) * hull_resistance_X + X_P + X_R
        Y = hull_sway_Y + Y_R
        N = hull_yaw_N + N_R
        
        # Calculate accelerations
        nu_dot = np.zeros(6)
        added_mass_x = 0.15 * self.mass
        added_mass_y = 0.8 * self.mass
        added_inertia_z = 0.1 * self.Iz

        nu_dot[0] = X / (self.mass + added_mass_x)
        nu_dot[1] = Y / (self.mass + added_mass_y)
        nu_dot[5] = N / (self.Iz + added_inertia_z)

        return nu_dot

