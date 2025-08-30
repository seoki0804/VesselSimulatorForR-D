# vds/utils/logger.py

import pandas as pd
from datetime import datetime
import os

class DataLogger:
    """
    Logs simulation data and saves it to a CSV file upon completion.
    """
    def __init__(self):
        self.log_data = []

    def log(self, simulator, control):
        """
        Records a snapshot of the current simulation state.
        """
        state_summary = {
            'timestamp': simulator.time,
            'pos_x': simulator.vessel.state.eta[0],
            'pos_y': simulator.vessel.state.eta[1],
            'heading_deg': simulator.vessel.heading,
            'sog_kts': simulator.vessel.sog,
            'rot_deg_min': simulator.vessel.rot,
            'u_mps': simulator.vessel.state.nu[0],
            'v_mps': simulator.vessel.state.nu[1],
            'r_rad_s': simulator.vessel.state.nu[5],
            'control_rpm': control.get('rpm', 0),
            'control_rudder_deg': control.get('rudder_angle', 0)
        }
        self.log_data.append(state_summary)

    def save(self, output_dir="output"):
        """
        Saves the logged data to a timestamped CSV file.
        """
        if not self.log_data:
            print("No data to save.")
            return

        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(output_dir, f"simulation_log_{timestamp}.csv")
        
        df = pd.DataFrame(self.log_data)
        df.to_csv(filename, index=False)
        print(f"Simulation log saved to {filename}")
