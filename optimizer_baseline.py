import pandas as pd
import subprocess
import os
import re

# --- Configuration Constants ---
# These are now defaults, can be overridden by the main function call
THETA1_REVENUE_WEIGHT = 1.0
THETA2_CONGESTION_WEIGHT = 0.005
TOLL_MIN = 0.0
TOLL_MAX = 15.0
MAX_TOLL_CHANGE = 0.5

# --- File and Program Names ---
EDGES_INPUT_FILE = "edges.csv"
SIMULATOR_WORKING_DIR = "LivingCity"
CONFIG_FILE_NAME = "command_line_options.ini"
SIM_OUTPUT_FILE_NAME = "results.csv"
SIMULATOR_EXECUTABLE = "./LivingCity"

def update_ini_file_safely(ini_path: str, toll_filepath: str):
    if not os.path.exists(ini_path):
        print(f"Error: Cannot find configuration file at '{ini_path}'")
        exit()
    with open(ini_path, 'r') as f:
        lines = f.readlines()
    key_to_update = "TOLL_FILE_PATH"
    new_line = f"{key_to_update} = {toll_filepath}\n"
    found = False
    for i, line in enumerate(lines):
        if re.match(fr'^\s*{key_to_update}\s*=', line, re.IGNORECASE):
            lines[i], found = new_line, True
            break
    if not found:
        lines.append(new_line)
    with open(ini_path, 'w') as f:
        f.writelines(lines)
    print(f"Safely updated '{ini_path}' with {key_to_update} = {toll_filepath}")


def run_simulation():
    print(f"\nExecuting command: '{SIMULATOR_EXECUTABLE}' inside directory '{SIMULATOR_WORKING_DIR}'")
    try:
        result = subprocess.run([SIMULATOR_EXECUTABLE], check=True, capture_output=True, text=True, timeout=1800,
                                cwd=SIMULATOR_WORKING_DIR)
        print("C++ Simulator executed successfully.")
    except FileNotFoundError:
        print(f"Error: Simulator program not found at '{os.path.join(SIMULATOR_WORKING_DIR, SIMULATOR_EXECUTABLE)}'.")
        exit()
    except subprocess.CalledProcessError as e:
        print(
            f"Error: C++ Simulator failed with exit code {e.returncode}.\n--- C++ Stdout ---\n{e.stdout}\n--- C++ Stderr ---\n{e.stderr}")
        exit()
    except subprocess.TimeoutExpired:
        print("Error: C++ Simulator process timed out.")
        exit()


def update_tolls(iter_df: pd.DataFrame, learning_rate: float) -> pd.Series:  ### <<< CHANGE: pass learning rate
    congestion_delay = iter_df['final_travel_time'] - iter_df['free_flow_time']
    toll_change = learning_rate * congestion_delay
    constrained_toll_change = toll_change.clip(lower=-MAX_TOLL_CHANGE, upper=MAX_TOLL_CHANGE)
    new_toll = iter_df['toll_fee'] + constrained_toll_change
    final_new_toll = new_toll.clip(lower=TOLL_MIN, upper=TOLL_MAX)
    final_new_toll.index = iter_df['uniqueid']
    print("Toll fees updated for the next iteration.")
    return final_new_toll


def main(results_dir: str, max_iterations: int, learning_rate: float):
    """
    The main optimization workflow.
    """
    if not os.path.exists(EDGES_INPUT_FILE):
        print(f"Error: Main input file '{EDGES_INPUT_FILE}' not found.")
        return

    os.makedirs(results_dir, exist_ok=True)

    edges_df = pd.read_csv(EDGES_INPUT_FILE)
    edges_df['free_flow_time'] = edges_df['length'] / (edges_df['speed_mph'] * 0.44704)

    current_tolls = pd.Series(0.0, index=edges_df['uniqueid'])
    print("Initialized tolls to zero and calculated free-flow times.")

    summary_data = []

    # --- Optimization Loop ---
    for i in range(max_iterations):  ### <<< CHANGE
        print("-" * 50)
        print(f"Starting Iteration {i + 1}/{max_iterations}")

        toll_update_df = pd.DataFrame({
            'edge_id': current_tolls.index,
            'toll_fee': current_tolls.values
        })

        temp_toll_filename = f"tolls_for_iter_{i}.csv"
        temp_toll_filepath_for_sim = os.path.join(SIMULATOR_WORKING_DIR, temp_toll_filename)
        toll_update_df.to_csv(temp_toll_filepath_for_sim, index=False)
        config_path_in_sim_dir = os.path.join(SIMULATOR_WORKING_DIR, CONFIG_FILE_NAME)
        update_ini_file_safely(config_path_in_sim_dir, temp_toll_filename)
        run_simulation()
        sim_output_filepath = os.path.join(SIMULATOR_WORKING_DIR, SIM_OUTPUT_FILE_NAME)
        if not os.path.exists(sim_output_filepath):
            print(f"Error: Simulator did not produce output file '{sim_output_filepath}'.")
            break
        results_df = pd.read_csv(sim_output_filepath)
        iter_df = edges_df[['uniqueid', 'free_flow_time']].copy()
        iter_df['toll_fee'] = current_tolls.loc[iter_df['uniqueid']].values
        iter_df = pd.merge(iter_df, results_df, left_on='uniqueid', right_on='edge_id', how='left')
        iter_df['final_flow'].fillna(0, inplace=True)
        iter_df['final_travel_time'].fillna(iter_df['free_flow_time'], inplace=True)
        revenue = (iter_df['toll_fee'] * iter_df['final_flow']).sum()
        congestion = (iter_df['final_travel_time'] * iter_df['final_flow']).sum()
        objective = (THETA1_REVENUE_WEIGHT * revenue) - (THETA2_CONGESTION_WEIGHT * congestion)
        print(f"  - Objective: {objective:,.2f}")
        print(f"  - Total Revenue: ${revenue:,.2f}")
        print(f"  - Total System Travel Time (Congestion): {congestion:,.0f} vehicle-seconds")

        iteration_filename = os.path.join(results_dir, f"iteration_{i + 1}_details.csv")  ### <<< CHANGE
        iter_df.to_csv(iteration_filename, index=False, float_format='%.4f')
        print(f"  - Detailed results saved to '{iteration_filename}'")

        summary_data.append({
            'iteration': i + 1,
            'objective': objective,
            'revenue': revenue,
            'congestion': congestion
        })

        newly_calculated_tolls = update_tolls(iter_df.dropna(subset=['edge_id']), learning_rate)
        current_tolls.update(newly_calculated_tolls)

    print("-" * 50)
    print("Optimization complete.")

    if summary_data:
        summary_df = pd.DataFrame(summary_data)
        summary_filename = os.path.join(results_dir, "optimization_summary.csv")  ### <<< CHANGE
        summary_df.to_csv(summary_filename, index=False, float_format='%.2f')
        print(f"Optimization summary saved to '{summary_filename}'")


if __name__ == "__main__":
    ### <<< CHANGE: This section now controls which experiment you run

    # --- Run 1: Baseline Scenario ---
    print("===== RUNNING BASELINE SCENARIO =====")
    main(
        results_dir="baseline_results",
        max_iterations=1,
        learning_rate=0.0
    )

    # --- Run 2: Full Optimization Scenario ---
    """print("\n\n===== RUNNING FULL OPTIMIZATION SCENARIO =====")
    main(
        results_dir="optimized_results",
        max_iterations=20,
        learning_rate=0.01
    )"""