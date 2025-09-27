import pandas as pd
import subprocess
import os
import re

"""
optimizer.py (Final Robust Version for C++ Integration)

This script implements the upper-level optimization loop for a bilevel traffic
tolling model. It iteratively calls the C++ LivingCity simulator, analyzes
the output, and updates road toll fees to optimize a weighted objective of
revenue and congestion.
"""

# --- Configuration Constants ---
MAX_ITERATIONS = 20
THETA1_REVENUE_WEIGHT = 1.0
THETA2_CONGESTION_WEIGHT = 0.005
LEARNING_RATE = 0.01

TOLL_MIN = 0.0
TOLL_MAX = 15.0
MAX_TOLL_CHANGE = 0.5

# --- File and Program Names ---
EDGES_INPUT_FILE = "edges.csv"
CONFIG_FILE_NAME = "command_line_options.ini"
SIM_OUTPUT_FILE_NAME = "results.csv"
SIMULATOR_EXECUTABLE = "./LivingCity"
SIMULATOR_WORKING_DIR = "LivingCity"


def update_ini_file_safely(ini_path: str, toll_filepath: str):
    """
    Safely reads an .ini file, updates or adds the TOLL_FILE_PATH,
    and writes it back without altering the original structure.
    """
    if not os.path.exists(ini_path):
        print(f"Error: Cannot find configuration file at '{ini_path}'")
        exit()

    with open(ini_path, 'r') as f:
        lines = f.readlines()

    key_to_update = "toll_file_path"
    new_line = f"{key_to_update} = {toll_filepath}\n"
    found = False

    for i, line in enumerate(lines):
        # Using regex to find the key, ignoring case and leading/trailing whitespace
        if re.match(fr'^\s*{key_to_update}\s*=', line, re.IGNORECASE):
            lines[i] = new_line
            found = True
            break

    if not found:
        # If key was not found, append it. Assumes a [General] or similar section exists.
        lines.append(new_line)

    with open(ini_path, 'w') as f:
        f.writelines(lines)
    print(f"Safely updated '{ini_path}' with TOLL_FILE_PATH = {toll_filepath}")


def run_simulation(config_filepath_for_sim: str):
    """
    Executes the C++ simulator in the correct working directory.
    """
    print(f"\nExecuting command: '{SIMULATOR_EXECUTABLE}' inside directory '{SIMULATOR_WORKING_DIR}'")
    try:
        result = subprocess.run(
            [SIMULATOR_EXECUTABLE],
            check=True,
            capture_output=True,
            text=True,
            timeout=1800,
            cwd=SIMULATOR_WORKING_DIR
        )
        print("C++ Simulator executed successfully.")
    except FileNotFoundError:
        print(f"Error: Simulator program not found at '{os.path.join(SIMULATOR_WORKING_DIR, SIMULATOR_EXECUTABLE)}'.")
        exit()
    except subprocess.CalledProcessError as e:
        print(f"Error: C++ Simulator failed with exit code {e.returncode}.")
        print(f"--- C++ Stdout ---\n{e.stdout}")
        print(f"--- C++ Stderr ---\n{e.stderr}")
        exit()
    except subprocess.TimeoutExpired:
        print("Error: C++ Simulator process timed out.")
        exit()


def update_tolls(iter_df: pd.DataFrame) -> pd.Series:
    congestion_delay = iter_df['final_travel_time'] - iter_df['free_flow_time']
    toll_change = LEARNING_RATE * congestion_delay
    constrained_toll_change = toll_change.clip(lower=-MAX_TOLL_CHANGE, upper=MAX_TOLL_CHANGE)
    new_toll = iter_df['toll_fee'] + constrained_toll_change
    final_new_toll = new_toll.clip(lower=TOLL_MIN, upper=TOLL_MAX)
    print("Toll fees updated for the next iteration.")
    return final_new_toll


def main():
    if not os.path.exists(EDGES_INPUT_FILE):
        print(f"Error: Main input file '{EDGES_INPUT_FILE}' not found.")
        return

    edges_df = pd.read_csv(EDGES_INPUT_FILE)
    edges_df['free_flow_time'] = edges_df['length'] / (edges_df['speed_mph'] * 0.44704)

    current_tolls = pd.Series(0.0, index=edges_df['uniqueid'])
    print("Initialized tolls to zero and calculated free-flow times.")

    for i in range(MAX_ITERATIONS):
        print("-" * 50)
        print(f"Starting Iteration {i + 1}/{MAX_ITERATIONS}")

        toll_update_df = pd.DataFrame({
            'edge_id': current_tolls.index,
            'toll_fee': current_tolls.values
        })

        # NOTE: C++ program will look for this file relative to its working directory
        temp_toll_filename = f"tolls_for_iter_{i}.csv"
        temp_toll_filepath_for_sim = os.path.join(SIMULATOR_WORKING_DIR, temp_toll_filename)
        toll_update_df.to_csv(temp_toll_filepath_for_sim, index=False)

        # Update the .ini file that lives inside the simulator's directory
        config_path_in_sim_dir = os.path.join(SIMULATOR_WORKING_DIR, CONFIG_FILE_NAME)
        update_ini_file_safely(config_path_in_sim_dir, temp_toll_filename)

        # Execute the C++ simulator
        run_simulation(config_path_in_sim_dir)

        # Read the results file, which was created inside the simulator's directory
        sim_output_filepath = os.path.join(SIMULATOR_WORKING_DIR, SIM_OUTPUT_FILE_NAME)
        if not os.path.exists(sim_output_filepath):
            print(f"Error: Simulator did not produce output file '{sim_output_filepath}'.")
            break
        results_df = pd.read_csv(sim_output_filepath)

        iter_df = edges_df[['uniqueid', 'free_flow_time']].copy()
        iter_df['toll_fee'] = current_tolls.values
        iter_df = pd.merge(iter_df, results_df, left_on='uniqueid', right_on='edge_id')

        if iter_df['final_flow'].isnull().any():
            print("Warning: Some edges missing from results. Filling with 0.")
            iter_df.fillna({'final_flow': 0, 'final_travel_time': 0}, inplace=True)

        revenue = (iter_df['toll_fee'] * iter_df['final_flow']).sum()
        congestion = (iter_df['final_travel_time'] * iter_df['final_flow']).sum()
        objective = (THETA1_REVENUE_WEIGHT * revenue) - (THETA2_CONGESTION_WEIGHT * congestion)

        print(f"  - Objective: {objective:,.2f}")
        print(f"  - Total Revenue: ${revenue:,.2f}")
        print(f"  - Total System Travel Time (Congestion): {congestion:,.0f} vehicle-seconds")

        current_tolls = update_tolls(iter_df)
        current_tolls.index = edges_df['uniqueid']

    print("-" * 50)
    print("Optimization complete.")
    final_tolls_df = pd.DataFrame({'uniqueid': current_tolls.index, 'toll_fee': current_tolls.values})
    print("Final Toll Fees:")
    print(final_tolls_df[final_tolls_df['toll_fee'] > 0].round(4))


if __name__ == "__main__":
    main()