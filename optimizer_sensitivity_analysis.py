import pandas as pd
import subprocess
import os
import re
import shutil

"""
optimizer.py (Sensitivity Analysis Version)

This script runs multiple optimization scenarios by iterating through a predefined
list of parameter configurations. For each scenario, it creates a unique output
directory to store all results, preventing data from being overwritten.
"""

SCENARIOS = [
    #{
        #"name": "Default_Params",
        #"THETA1_REVENUE_WEIGHT": 1.0,
        #"THETA2_CONGESTION_WEIGHT": 0.005,
        #"LEARNING_RATE": 0.01,
        #"MAX_TOLL_CHANGE": 0.5
    #},
    {
        "name": "More_Aggressive_Learning",
        "THETA1_REVENUE_WEIGHT": 1.0,
        "THETA2_CONGESTION_WEIGHT": 0.005,
        "LEARNING_RATE": 0.5,  # Update tolls much more aggressively
        "MAX_TOLL_CHANGE": 10.0  # Allow larger toll changes
    },
    {
        "name": "High_Revenue_Focus",
        "THETA1_REVENUE_WEIGHT": 2.0,       # Emphasize revenue more
        "THETA2_CONGESTION_WEIGHT": 0.005,
        "LEARNING_RATE": 0.01,
        "MAX_TOLL_CHANGE": 0.5
    },
    {
        "name": "High_Congestion_Focus",
        "THETA1_REVENUE_WEIGHT": 1.0,
        "THETA2_CONGESTION_WEIGHT": 0.01,   # Penalize congestion more
        "LEARNING_RATE": 0.01,
        "MAX_TOLL_CHANGE": 0.5
    },
    {
        "name": "Aggressive_Learning",
        "THETA1_REVENUE_WEIGHT": 1.0,
        "THETA2_CONGESTION_WEIGHT": 0.005,
        "LEARNING_RATE": 0.05,              # Update tolls more aggressively
        "MAX_TOLL_CHANGE": 1.0              # Allow larger toll changes
    },
]

# --- General Configuration ---
MAX_ITERATIONS = 20
TOLL_MIN = 0.0
TOLL_MAX = 15.0
EDGES_INPUT_FILE = "edges.csv"
SIMULATOR_WORKING_DIR = "LivingCity"
CONFIG_FILE_NAME = "command_line_options.ini"
SIM_OUTPUT_FILE_NAME = "results.csv"
SIMULATOR_EXECUTABLE = "./LivingCity"

def update_ini_file_safely(ini_path: str, toll_filepath: str):
    # ... (This function remains unchanged)
    if not os.path.exists(ini_path):
        print(f"Error: Cannot find config file at '{ini_path}'")
        exit()
    with open(ini_path, 'r') as f: lines = f.readlines()
    key_to_update = "TOLL_FILE_PATH"
    new_line = f"{key_to_update} = {toll_filepath}\n"
    found = False
    for i, line in enumerate(lines):
        if re.match(fr'^\s*{key_to_update}\s*=', line, re.IGNORECASE):
            lines[i] = new_line
            found = True
            break
    if not found: lines.append(new_line)
    with open(ini_path, 'w') as f: f.writelines(lines)
    # print(f"Safely updated '{ini_path}' with {key_to_update} = {toll_filepath}")

def run_simulation():
    # ... (This function remains unchanged)
    print(f"\nExecuting command: '{SIMULATOR_EXECUTABLE}' inside directory '{SIMULATOR_WORKING_DIR}'")
    try:
        result = subprocess.run(
            [SIMULATOR_EXECUTABLE], check=True, capture_output=True,
            text=True, timeout=1800, cwd=SIMULATOR_WORKING_DIR
        )
        print("C++ Simulator executed successfully.")
    except Exception as e:
        print(f"An error occurred during C++ simulation: {e}")
        exit()

def update_tolls(iter_df: pd.DataFrame, learning_rate: float, max_toll_change: float) -> pd.Series:
    """Updates tolls, now accepting LR and max_change as parameters."""
    congestion_delay = iter_df['final_travel_time'] - iter_df['free_flow_time']
    toll_change = learning_rate * congestion_delay
    constrained_toll_change = toll_change.clip(lower=-max_toll_change, upper=max_toll_change)
    new_toll = iter_df['toll_fee'] + constrained_toll_change
    final_new_toll = new_toll.clip(lower=TOLL_MIN, upper=TOLL_MAX)
    final_new_toll.index = iter_df['uniqueid']
    print("Toll fees updated for the next iteration.")
    return final_new_toll

def run_single_experiment(scenario_name: str, params: dict):
    """
    Runs one full optimization experiment for a given scenario.
    """
    # --- Create a unique directory for this scenario's results ---
    output_dir = f"results_{scenario_name}"
    os.makedirs(output_dir, exist_ok=True)
    print(f"Results for this scenario will be saved in: '{output_dir}'")
    
    edges_df = pd.read_csv(EDGES_INPUT_FILE)
    edges_df['free_flow_time'] = edges_df['length'] / (edges_df['speed_mph'] * 0.44704)
    current_tolls = pd.Series(0.0, index=edges_df['uniqueid'])
    
    summary_data = []

    for i in range(MAX_ITERATIONS):
        print("-" * 50)
        print(f"Running Iteration {i + 1}/{MAX_ITERATIONS} for scenario '{scenario_name}'")

        toll_update_df = pd.DataFrame({'edge_id': current_tolls.index, 'toll_fee': current_tolls.values})
        temp_toll_filename = f"tolls_for_iter_{i}.csv"
        temp_toll_filepath_for_sim = os.path.join(SIMULATOR_WORKING_DIR, temp_toll_filename)
        toll_update_df.to_csv(temp_toll_filepath_for_sim, index=False)
        
        config_path_in_sim_dir = os.path.join(SIMULATOR_WORKING_DIR, CONFIG_FILE_NAME)
        update_ini_file_safely(config_path_in_sim_dir, temp_toll_filename)

        run_simulation()

        sim_output_filepath = os.path.join(SIMULATOR_WORKING_DIR, SIM_OUTPUT_FILE_NAME)
        results_df = pd.read_csv(sim_output_filepath)

        iter_df = pd.merge(edges_df, results_df, left_on='uniqueid', right_on='edge_id', how='left')
        iter_df['toll_fee'] = current_tolls.loc[iter_df['uniqueid']].values
        iter_df['final_flow'].fillna(0, inplace=True)
        iter_df['final_travel_time'].fillna(iter_df['free_flow_time'], inplace=True)
        
        # Use parameters from the current scenario
        revenue = (iter_df['toll_fee'] * iter_df['final_flow']).sum()
        congestion = (iter_df['final_travel_time'] * iter_df['final_flow']).sum()
        objective = (params["THETA1_REVENUE_WEIGHT"] * revenue) - (params["THETA2_CONGESTION_WEIGHT"] * congestion)

        print(f"  - Objective: {objective:,.2f}")

        # Save detailed results for this iteration into the scenario's directory
        iteration_filename = os.path.join(output_dir, f"iteration_{i+1}_details.csv")
        iter_df.to_csv(iteration_filename, index=False, float_format='%.4f')
        print(f"  - Detailed results saved to '{iteration_filename}'")
        
        summary_data.append({'iteration': i + 1, 'objective': objective, 'revenue': revenue, 'congestion': congestion})
        
        newly_calculated_tolls = update_tolls(iter_df.dropna(subset=['edge_id']), params["LEARNING_RATE"], params["MAX_TOLL_CHANGE"])
        current_tolls.update(newly_calculated_tolls)
        
    # Save the summary file for this specific scenario
    summary_df = pd.DataFrame(summary_data)
    summary_filename = os.path.join(output_dir, "optimization_summary.csv")
    summary_df.to_csv(summary_filename, index=False, float_format='%.2f')
    print(f"\nOptimization summary for '{scenario_name}' saved to '{summary_filename}'")

def main():
    """
    Main function to loop through all scenarios and run the experiments.
    """
    # First, run a baseline simulation with zero tolls for comparison
    print(f"\n\n{'='*60}")
    print("RUNNING SCENARIO: Baseline (Zero Tolls)")
    print(f"{'='*60}")
    #baseline_params = SCENARIOS[0].copy() # Use first scenario structure but with no learning
    #baseline_params['LEARNING_RATE'] = 0.0
    #baseline_params['MAX_TOLL_CHANGE'] = 0.0
    #run_single_experiment(scenario_name="baseline", params=baseline_params)

    print("RUNNING SCENARIO: Others")
    # Then, run all defined scenarios
    for scenario in SCENARIOS:
        print(f"\n\n{'='*60}")
        print(f"RUNNING SCENARIO: {scenario['name']}")
        print(f"{'='*60}")
        run_single_experiment(scenario_name=scenario['name'], params=scenario)
    
    print("\n\nAll scenarios complete.")
    print("You can now use the analysis.py script to compare the results folders:")
    # List the created folders
    folder_names = ["results_baseline"] + [f"results_{s['name']}" for s in SCENARIOS]
    print(" - " + "\n - ".join(folder_names))

if __name__ == "__main__":
    main()