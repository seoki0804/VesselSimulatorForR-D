# scenarios/scenario_loader.py

import yaml
import numpy as np
from vds.models.vessels.base_vessel import BaseVessel, VesselSpecifications, VesselState
from vds.environment.geography import Geography
from vds.environment.wind import Wind
from vds.environment.current import Current
from vds.environment.waves import Waves
from vds.data_handler.ais_parser import load_ais_targets
from vds.models.dynamics.mmg_model import MMGModel

def load_scenario(filepath: str):
    """Loads all simulation components from a YAML scenario file."""
    with open(filepath, 'r') as f:
        config = yaml.safe_load(f)

    # Load Vessel
    vessel_conf = config['vessel']
    specs = VesselSpecifications(**vessel_conf['specs'])
    
    initial_speed_ms = vessel_conf['initial_state']['speed_kts'] * 0.514444
    initial_pos = vessel_conf['initial_state'].get('position', [0, 0])
    
    # FIXED: Explicitly create the eta array with dtype=float
    initial_eta = np.array([initial_pos[0], initial_pos[1], 0, 0, 0, 0], dtype=float)
    initial_nu = np.array([initial_speed_ms, 0, 0, 0, 0, 0], dtype=float)
    
    initial_state = VesselState(eta=initial_eta, nu=initial_nu)
    vessel = BaseVessel(specs, initial_state)

    # Load Dynamics Model
    hydro_params_path = vessel_conf['hydro_params']
    dynamics_model = MMGModel(vessel.specs, hydro_params_path)
    
    # Load Environment
    env_conf = config['environment']
    geography = Geography.from_csv(env_conf['geography_data'], env_conf['cell_size'])
    
    if env_conf.get('obstacles', {}).get('enabled', False):
        for obs_data in env_conf['obstacles'].get('locations', []):
            geography.add_obstacle(center_x=obs_data['position'][0], center_y=obs_data['position'][1], radius=obs_data['radius'])

    ais_targets = []
    if env_conf.get('ais_targets', {}).get('enabled', False):
        ais_targets = load_ais_targets('data/ais/sample_ais_tracks.csv')

    waypoints = env_conf.get('waypoints', [])

    wind = Wind(speed=env_conf['wind']['speed_kts'], direction=env_conf['wind']['direction_deg'])
    current = Current(speed=env_conf['current']['speed_kts'], direction=env_conf['current']['direction_deg'])
    waves = Waves(significant_height=env_conf['waves']['hs_m'], period=env_conf['waves']['period_s'], direction=env_conf['waves']['direction_deg'])
    
    initial_control = config['initial_control']

    print(f"Loaded scenario: {config['scenario_name']}")
    
    return vessel, dynamics_model, geography, ais_targets, wind, current, waves, initial_control, waypoints

