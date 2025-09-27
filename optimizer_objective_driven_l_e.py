import pandas as pd
import subprocess
import os
import re
import numpy as np

"""
optimizer.py (Final Optimized Version)

This script runs multiple optimization scenarios, including a single-iteration
baseline run for scientific comparison. It features an objective-driven update
rule and an early-stopping mechanism for efficiency.
"""

# ==============================================================================
# --- 1. DEFINE YOUR EXPERIMENTS HERE ---
# ==============================================================================

SCENARIOS = [
    {
        "name": "Best_Params",
        "THETA1_REVENUE_WEIGHT": 1.0,
        "THETA2_CONGESTION_WEIGHT": 0.4,
        "LEARNING_RATE": 1.6084171086874908,
        "MAX_TOLL_CHANGE": 3.734753854301818
    }
]

"""SCENARIOS = [
    #{
        #"name": "Default_Params",
        #"THETA1_REVENUE_WEIGHT": 1.0,
        #"THETA2_CONGESTION_WEIGHT": 0.005,
        #"LEARNING_RATE": 0.01,
        #"MAX_TOLL_CHANGE": 0.5
    #},
    {
        "name": "More_Aggressive_Learning_2",
        "THETA1_REVENUE_WEIGHT": 1.0,
        "THETA2_CONGESTION_WEIGHT": 0.005,
        "LEARNING_RATE": 1,  # Update tolls much more aggressively
        "MAX_TOLL_CHANGE": 20.0  # Allow larger toll changes
    },
    {
        "name": "High_Revenue_Focus_2",
        "THETA1_REVENUE_WEIGHT": 2.0,       # Emphasize revenue more
        "THETA2_CONGESTION_WEIGHT": 0.005,
        "LEARNING_RATE": 0.1,
        "MAX_TOLL_CHANGE": 5
    },
    {
        "name": "High_Congestion_Focus_2",
        "THETA1_REVENUE_WEIGHT": 1.0,
        "THETA2_CONGESTION_WEIGHT": 0.01,   # Penalize congestion more
        "LEARNING_RATE": 0.1,
        "MAX_TOLL_CHANGE": 5
    },
    {
        "name": "Aggressive_Learning_2",
        "THETA1_REVENUE_WEIGHT": 1.0,
        "THETA2_CONGESTION_WEIGHT": 0.005,
        "LEARNING_RATE": 0.5,              # Update tolls more aggressively
        "MAX_TOLL_CHANGE": 10              # Allow larger toll changes
    },
]"""

# --- General Configuration ---
MAX_ITERATIONS = 50
TOLL_MIN = 0.0
TOLL_MAX = 20.0
EDGES_INPUT_FILE = "edges.csv"
SIMULATOR_WORKING_DIR = "LivingCity"
CONFIG_FILE_NAME = "command_line_options.ini"
SIM_OUTPUT_FILE_NAME = "results.csv"
SIMULATOR_EXECUTABLE = "./LivingCity"

# --- Convergence Configuration ---
CONVERGENCE_PATIENCE = 5
CONVERGENCE_TOLERANCE = 5.0


def update_ini_file_safely(ini_path: str, toll_filepath: str):
    # This function remains the same
    if not os.path.exists(ini_path):
        print(f"Error: Cannot find config file at '{ini_path}'", flush=True)
        exit()
    with open(ini_path, 'r') as f:
        lines = f.readlines()
    key_to_update = "TOLL_FILE_PATH"
    new_line = f"{key_to_update} = {toll_filepath}\n"
    found = False
    for i, line in enumerate(lines):
        if re.match(fr'^\s*{key_to_update}\s*=', line, re.IGNORECASE):
            lines[i] = new_line
            found = True
            break
    if not found: lines.append(new_line)
    with open(ini_path, 'w') as f:
        f.writelines(lines)


def run_simulation():
    # This function remains the same
    print(f"Executing C++ simulator: '{SIMULATOR_EXECUTABLE}'...", flush=True)
    print("This may take several minutes. Please wait...", flush=True)
    try:
        subprocess.run(
            [SIMULATOR_EXECUTABLE], check=True, capture_output=True,
            text=True, timeout=3600, cwd=SIMULATOR_WORKING_DIR
        )
        print("C++ Simulator executed successfully.", flush=True)
    except Exception as e:
        print(f"An error occurred during C++ simulation: {e}", flush=True)
        exit()


def update_tolls_objective_driven(iter_df: pd.DataFrame, params: dict) -> pd.Series:
    """
    Updates tolls based on the marginal contribution of each edge to the
    objective function.
    """
    # Extract parameters for clarity
    theta1 = params["THETA1_REVENUE_WEIGHT"]
    theta2 = params["THETA2_CONGESTION_WEIGHT"]
    learning_rate = params["LEARNING_RATE"]
    max_toll_change = params["MAX_TOLL_CHANGE"]

    # Calculate the "marginal cost" or "damage" caused by one unit of flow on an edge
    # This is the congestion delay (T_e - T_e_ff) weighted by the congestion penalty theta2
    marginal_congestion_cost = theta2 * (iter_df['final_travel_time'] - iter_df['free_flow_time'])

    # The "target toll" should ideally be equal to the marginal congestion cost it's trying to mitigate.
    # This is a core principle from transportation economics.
    target_toll = marginal_congestion_cost

    # The update is based on the gap between the current toll and the target toll.
    # We also include theta1 to scale the update by the importance of revenue.
    toll_change = learning_rate * theta1 * (target_toll - iter_df['toll_fee'])

    constrained_toll_change = toll_change.clip(-max_toll_change, max_toll_change)
    new_toll = iter_df['toll_fee'] + constrained_toll_change
    final_new_toll = new_toll.clip(lower=TOLL_MIN, upper=TOLL_MAX)

    final_new_toll.index = iter_df['uniqueid']
    print("Toll fees updated based on objective function's marginal costs.", flush=True)
    return final_new_toll


def run_single_experiment(scenario_name: str, params: dict, max_iterations: int):
    """
    Runs one full optimization experiment with convergence checking.
    """
    output_dir = f"results_{scenario_name}"
    os.makedirs(output_dir, exist_ok=True)
    print(f"Results for this scenario will be saved in: '{output_dir}'", flush=True)

    edges_df = pd.read_csv(EDGES_INPUT_FILE)
    edges_df.drop_duplicates(subset=['uniqueid'], keep='first', inplace=True)
    edges_df['free_flow_time'] = edges_df['length'] / (edges_df['speed_mph'] * 0.44704)
    current_tolls = pd.Series(0.0, index=edges_df['uniqueid'])

    summary_data = []

    best_objective = -np.inf
    patience_counter = 0

    for i in range(max_iterations):
        print("-" * 50, flush=True)
        print(f"Running Iteration {i + 1}/{max_iterations} for scenario '{scenario_name}'", flush=True)

        toll_update_df = pd.DataFrame({'edge_id': current_tolls.index, 'toll_fee': current_tolls.values})
        temp_toll_filename = f"tolls_for_iter_{i}.csv"
        temp_toll_filepath_for_sim = os.path.join(SIMULATOR_WORKING_DIR, temp_toll_filename)
        toll_update_df.to_csv(temp_toll_filepath_for_sim, index=False)

        config_path_in_sim_dir = os.path.join(SIMULATOR_WORKING_DIR, CONFIG_FILE_NAME)
        update_ini_file_safely(config_path_in_sim_dir, temp_toll_filename)

        run_simulation()

        print("Processing simulation results...", flush=True)
        sim_output_filepath = os.path.join(SIMULATOR_WORKING_DIR, SIM_OUTPUT_FILE_NAME)
        results_df = pd.read_csv(sim_output_filepath)

        iter_df = pd.merge(edges_df, results_df, left_on='uniqueid', right_on='edge_id', how='left')
        iter_df['toll_fee'] = current_tolls.loc[iter_df['uniqueid']].values

        # --- FIX: Replaced inplace=True with direct assignment to remove FutureWarnings ---
        iter_df['final_flow'] = iter_df['final_flow'].fillna(0)
        iter_df['final_travel_time'] = iter_df['final_travel_time'].fillna(iter_df['free_flow_time'])
        # --- END FIX ---

        revenue = (iter_df['toll_fee'] * iter_df['final_flow']).sum()

        per_edge_delay = iter_df['final_travel_time'] - iter_df['free_flow_time']

        objective = (params["THETA1_REVENUE_WEIGHT"] * revenue) - (params["THETA2_CONGESTION_WEIGHT"] * total_delay)

        print(f"  - Objective: {objective:,.2f}", flush=True)

        iteration_filename = os.path.join(output_dir, f"iteration_{i + 1}_details.csv")
        iter_df.to_csv(iteration_filename, index=False, float_format='%.4f')
        print(f"  - Detailed results saved to '{iteration_filename}'", flush=True)

        summary_data.append({'iteration': i + 1, 'objective': objective, 'revenue': revenue, 'congestion': total_delay})

        simulated_iter_df = iter_df.dropna(subset=['edge_id']).copy()
        newly_calculated_tolls = update_tolls_objective_driven(
            simulated_iter_df,
            params
        )
        current_tolls.update(newly_calculated_tolls)

        if max_iterations > 1:
            if objective > best_objective + CONVERGENCE_TOLERANCE:
                best_objective = objective
                patience_counter = 0
                print(f"  - CONVERGENCE: New best objective found. Resetting patience.", flush=True)
            else:
                patience_counter += 1
                print(
                    f"  - CONVERGENCE: No significant improvement. Patience: {patience_counter}/{CONVERGENCE_PATIENCE}",
                    flush=True)

            if patience_counter >= CONVERGENCE_PATIENCE:
                print(f"\nConvergence reached at iteration {i + 1}. Halting scenario.", flush=True)
                break

    summary_df = pd.DataFrame(summary_data)
    summary_filename = os.path.join(output_dir, "optimization_summary.csv")
    summary_df.to_csv(summary_filename, index=False, float_format='%.2f')
    print(f"\nOptimization summary for '{scenario_name}' saved to '{summary_filename}'", flush=True)

# !!! --- KEY CHANGE: 'main' function is now smarter --- !!!
def main():
    """
    Main function to run the baseline and then loop through all optimization scenarios.
    """
    print(f"\n\n{'=' * 60}", flush=True)
    print("RUNNING SCENARIO: Baseline (Zero Tolls)", flush=True)
    print(f"{'=' * 60}", flush=True)

    # Define baseline parameters: use the first scenario's weights but with zero learning.
    baseline_params = SCENARIOS[0].copy()
    baseline_params['LEARNING_RATE'] = 0.0
    baseline_params['MAX_TOLL_CHANGE'] = 0.0
    # Run the baseline for ONLY ONE iteration.
    run_single_experiment(scenario_name="baseline", params=baseline_params, max_iterations=1)

    # Now, run all defined optimization scenarios for the full number of iterations.
    for scenario in SCENARIOS:
        print(f"\n\n{'=' * 60}", flush=True)
        print(f"RUNNING SCENARIO: {scenario['name']}", flush=True)
        print(f"{'=' * 60}", flush=True)
        run_single_experiment(scenario_name=scenario['name'], params=scenario, max_iterations=MAX_ITERATIONS)

    print("\n\nAll scenarios complete.", flush=True)


if __name__ == "__main__":
    main()