# visualize_log.py

import pandas as pd
import matplotlib.pyplot as plt
import os
import argparse

def find_latest_log(log_dir="output"):
    """
    Finds the most recent log file in the specified directory.
    지정된 디렉토리에서 가장 최신 로그 파일을 찾습니다.
    """
    if not os.path.exists(log_dir):
        return None
    
    log_files = [f for f in os.listdir(log_dir) if f.startswith("simulation_log_") and f.endswith(".csv")]
    
    if not log_files:
        return None
        
    # Sort files by name to get the latest one (based on timestamp)
    latest_file = sorted(log_files, reverse=True)[0]
    return os.path.join(log_dir, latest_file)

def plot_simulation_data(log_filepath):
    """
    Reads a simulation log CSV and generates plots for analysis.
    시뮬레이션 로그 CSV를 읽고 분석용 플롯을 생성합니다.
    """
    print(f"Loading log file: {log_filepath}")
    df = pd.read_csv(log_filepath)

    fig, axs = plt.subplots(2, 2, figsize=(16, 10))
    fig.suptitle('Simulation Analysis', fontsize=20)

    # 1. Trajectory Plot (항적 플롯)
    axs[0, 0].plot(df['pos_y'], df['pos_x']) # E-N plot
    axs[0, 0].set_title('Vessel Trajectory')
    axs[0, 0].set_xlabel('East position (m)')
    axs[0, 0].set_ylabel('North position (m)')
    axs[0, 0].grid(True)
    axs[0, 0].set_aspect('equal', adjustable='box')

    # 2. Speed over Time (시간에 따른 속력)
    axs[0, 1].plot(df['timestamp'], df['sog_kts'])
    axs[0, 1].set_title('Speed Over Ground (SOG) vs. Time')
    axs[0, 1].set_xlabel('Time (s)')
    axs[0, 1].set_ylabel('Speed (knots)')
    axs[0, 1].grid(True)
    
    # 3. Heading over Time (시간에 따른 선수각)
    axs[1, 0].plot(df['timestamp'], df['heading_deg'])
    axs[1, 0].set_title('Heading vs. Time')
    axs[1, 0].set_xlabel('Time (s)')
    axs[1, 0].set_ylabel('Heading (degrees)')
    axs[1, 0].grid(True)
    
    # 4. Rudder Angle over Time (시간에 따른 타각)
    axs[1, 1].plot(df['timestamp'], df['control_rudder_deg'], color='red')
    axs[1, 1].set_title('Rudder Angle vs. Time')
    axs[1, 1].set_xlabel('Time (s)')
    axs[1, 1].set_ylabel('Rudder Angle (degrees)')
    axs[1, 1].grid(True)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    # Save the plot
    plot_filename = log_filepath.replace('.csv', '.png')
    plt.savefig(plot_filename)
    print(f"Analysis plot saved to: {plot_filename}")

    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize vessel simulation log data.")
    parser.add_argument('filepath', type=str, nargs='?', default=None,
                        help="Path to the simulation log CSV file. If not provided, the latest log in the 'output' directory will be used.")
    args = parser.parse_args()

    log_file_to_visualize = args.filepath
    if log_file_to_visualize is None:
        log_file_to_visualize = find_latest_log()

    if log_file_to_visualize and os.path.exists(log_file_to_visualize):
        plot_simulation_data(log_file_to_visualize)
    else:
        print("Error: Log file not found. Please run a simulation first or provide a valid file path.")
