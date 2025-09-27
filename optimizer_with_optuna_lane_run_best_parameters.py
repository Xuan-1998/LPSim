import pandas as pd
import subprocess
import os
import re
import numpy as np

"""
run_best_parameters.py (with Detailed Output Generation)

This script runs a single, full simulation experiment using a pre-defined set of
'best' hyperparameters. It has been modified to also run a baseline (no-toll)
scenario and to save detailed per-edge results for each iteration. This ensures
that the downstream analysis script has all the necessary input files.
"""

# ==============================================================================
# --- 1. CONFIGURATION ---
# ==============================================================================
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
CONVERGENCE_TOLERANCE = 10.0

# --- Output Directories ---
# The analysis script looks for a directory with this exact name for baseline data.
BASELINE_RESULTS_DIR = "baseline_results"
# The directory for the main optimized run.
OPTIMIZED_RUN_DIR = "final_best_run"

# ==============================================================================
# --- 2. PARAMETER SETS ---
# ==============================================================================
# Best hyperparameters found from your Optuna study.
BEST_PARAMS = {
    "name": "best_params_run",
    "THETA1_REVENUE_WEIGHT": 1.0,
    "THETA2_CONGESTION_WEIGHT": 0.0010002980814772604,
    "LEARNING_RATE": 0.15900879015941038,
    "MAX_TOLL_CHANGE": 6.915591953709335
}

# Parameters for the baseline (no-toll) scenario.
BASELINE_PARAMS = {
    "name": "baseline",
    "THETA1_REVENUE_WEIGHT": 1.0,
    "THETA2_CONGESTION_WEIGHT": 0.0,
    "LEARNING_RATE": 0.0,
    "MAX_TOLL_CHANGE": 0.0
}


# ==============================================================================
# --- 3. CORE & EXPERIMENT FUNCTIONS ---
# ==============================================================================
def update_ini_file_safely(ini_path: str, toll_filepath: str):
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
        raise e


def update_tolls_objective_driven(iter_df: pd.DataFrame, params: dict) -> pd.Series:
    theta1 = params["THETA1_REVENUE_WEIGHT"]
    theta2 = params["THETA2_CONGESTION_WEIGHT"]
    learning_rate = params["LEARNING_RATE"]
    max_toll_change = params["MAX_TOLL_CHANGE"]
    marginal_congestion_cost = theta2 * (iter_df['final_travel_time'] - iter_df['free_flow_time'])
    target_toll = marginal_congestion_cost
    toll_change = learning_rate * theta1 * (target_toll - iter_df['toll_fee'])
    constrained_toll_change = toll_change.clip(-max_toll_change, max_toll_change)
    new_toll = iter_df['toll_fee'] + constrained_toll_change
    final_new_toll = new_toll.clip(lower=TOLL_MIN, upper=TOLL_MAX)
    final_new_toll.index = iter_df['uniqueid']
    print("Toll fees updated based on objective function's marginal costs.", flush=True)
    return final_new_toll


def run_single_experiment(scenario_name: str, params: dict, max_iterations: int, output_base_dir: str):
    """
    Runs one full optimization experiment and saves both summary and detailed results.
    """
    # For the baseline, we don't need the extra 'results_' prefix to match the analysis script.
    if scenario_name == "baseline":
        output_dir = output_base_dir
    else:
        output_dir = os.path.join(output_base_dir, f"results_{scenario_name}")

    os.makedirs(output_dir, exist_ok=True)
    print(f"Results for this run will be saved in: '{output_dir}'")

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
        temp_toll_filename = f"tolls_iter_{i + 1}.csv"
        temp_toll_filepath_for_sim = os.path.join(SIMULATOR_WORKING_DIR, temp_toll_filename)
        toll_update_df.to_csv(temp_toll_filepath_for_sim, index=False)

        config_path_in_sim_dir = os.path.join(SIMULATOR_WORKING_DIR, CONFIG_FILE_NAME)
        update_ini_file_safely(config_path_in_sim_dir, temp_toll_filename)

        run_simulation()

        sim_output_filepath = os.path.join(SIMULATOR_WORKING_DIR, SIM_OUTPUT_FILE_NAME)
        results_df = pd.read_csv(sim_output_filepath)
        iter_df = pd.merge(edges_df, results_df, left_on='uniqueid', right_on='edge_id', how='left')
        iter_df['toll_fee'] = current_tolls.loc[iter_df['uniqueid']].values
        iter_df['final_flow'] = iter_df['final_flow'].fillna(0)
        iter_df['final_travel_time'] = iter_df['final_travel_time'].fillna(iter_df['free_flow_time'])

        # --- NEW: SAVE DETAILED ITERATION DATA ---
        # This is the file required by the analysis script for spatial plots.
        details_filename = f"iteration_{i + 1}_details.csv"
        details_filepath = os.path.join(output_dir, details_filename)
        iter_df.to_csv(details_filepath, index=False, float_format='%.4f')
        print(f"  > Detailed results for iteration {i + 1} saved to '{details_filepath}'")
        # --- END NEW ---

        revenue = (iter_df['toll_fee'] * iter_df['final_flow']).sum()
        per_edge_delay = iter_df['final_travel_time'] - iter_df['free_flow_time']
        lane_weighted_congestion = (iter_df['lanes'] * per_edge_delay).sum()
        objective = (params["THETA1_REVENUE_WEIGHT"] * revenue) - \
                    (params["THETA2_CONGESTION_WEIGHT"] * lane_weighted_congestion)

        print(f"  - Revenue: {revenue:,.2f}", flush=True)
        print(f"  - Lane-Weighted Congestion: {lane_weighted_congestion:,.2f}", flush=True)
        print(f"  - Objective Value: {objective:,.2f}", flush=True)

        summary_data.append(
            {'iteration': i + 1, 'objective': objective, 'revenue': revenue, 'congestion': lane_weighted_congestion})

        simulated_iter_df = iter_df.dropna(subset=['edge_id']).copy()
        newly_calculated_tolls = update_tolls_objective_driven(simulated_iter_df, params)
        current_tolls.update(newly_calculated_tolls)

        if objective > best_objective + CONVERGENCE_TOLERANCE:
            best_objective = objective
            patience_counter = 0
        else:
            patience_counter += 1
        if patience_counter >= CONVERGENCE_PATIENCE:
            print(f"\nConvergence reached at iteration {i + 1}. Halting run.", flush=True)
            break

    summary_df = pd.DataFrame(summary_data)
    summary_filename = os.path.join(output_dir, "optimization_summary.csv")
    summary_df.to_csv(summary_filename, index=False, float_format='%.2f')
    print(f"\nFinal optimization summary saved to '{summary_filename}'", flush=True)


# ==============================================================================
# --- 4. MAIN EXECUTION BLOCK ---
# ==============================================================================
def main():
    """
    Main function to run both the baseline and the optimized experiment.
    """
    # --- Step 1: Run Baseline Scenario ---
    print("=" * 60)
    print("STEP 1: RUNNING BASELINE (NO-TOLL) SCENARIO")
    print("=" * 60)
    try:
        run_single_experiment(
            scenario_name=BASELINE_PARAMS["name"],
            params=BASELINE_PARAMS,
            max_iterations=1,  # Baseline only needs one iteration
            output_base_dir=BASELINE_RESULTS_DIR
        )
        print("\nBaseline run finished successfully!")
    except Exception as e:
        print(f"\nAn error occurred during the baseline run: {e}")
        return  # Stop if baseline fails

    # --- Step 2: Run Optimized Scenario ---
    print("\n" + "=" * 60)
    print("STEP 2: RUNNING EXPERIMENT WITH BEST HYPERPARAMETERS")
    print("=" * 60)
    print("Using Parameters:")
    for key, value in BEST_PARAMS.items():
        print(f"  - {key}: {value}")
    print("-" * 60)

    try:
        run_single_experiment(
            scenario_name=BEST_PARAMS["name"],
            params=BEST_PARAMS,
            max_iterations=MAX_ITERATIONS,
            output_base_dir=OPTIMIZED_RUN_DIR
        )
        print("\nOptimized experiment finished successfully!")
    except Exception as e:
        print(f"\nAn error occurred during the optimized experiment: {e}")

    print("\n" + "=" * 60)
    print("ALL RUNS COMPLETE")
    print("=" * 60)


if __name__ == "__main__":
    main()