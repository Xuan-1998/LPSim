import pandas as pd
import subprocess
import os
import re
import numpy as np

"""
optimizer_MSA_S3.py (Master Experiment Runner)

This script automates the entire sensitivity analysis process. It iterates through
a predefined matrix of experiments, configures and runs the C++ simulator for each,
and saves all results in a structured, non-overlapping directory format.
"""

# ==============================================================================
# --- 1. DEFINE YOUR EXPERIMENT MATRIX HERE ---
# ==============================================================================

# These are the HYPERPARAMETERS found from your previous Optuna run.
# We will hold these constant for the comparative analysis.
BEST_PARAMS = {
    "THETA1_REVENUE_WEIGHT": 1.0,
    "THETA2_CONGESTION_WEIGHT": 0.0055,
    "LEARNING_RATE": 0.05,
    "MAX_TOLL_CHANGE": 11.0,
    "INERTIA": 0.54
}

# Define the different levels for your sensitivity analysis
AV_PENETRATION_RATES = [0.0, 0.1, 0.3, 0.5, 0.7, 1.0]

# This is the master list of all experiments to run.
EXPERIMENTS = []

# --- SCENARIO 1 RUNS ---
# C++ will use SCENARIO_MODE = 1 (routing ignores tolls)
for eta in AV_PENETRATION_RATES:
    EXPERIMENTS.append({
        "name": f"S1_Competition_eta_{eta:.1f}",
        "SCENARIO_MODE": 1,
        "AV_PENETRATION_RATE": eta,
        "AV_TOLL_DISCOUNT": 1.0,  # No discount
        "objective_type": "lane_weighted",
        **BEST_PARAMS  # Use the best hyperparameters
    })

# --- SCENARIO 2 RUNS ---
# C++ will use SCENARIO_MODE = 2 (routing considers tolls)
for eta in AV_PENETRATION_RATES:
    EXPERIMENTS.append({
        "name": f"S2_PartialCollab_eta_{eta:.1f}",
        "SCENARIO_MODE": 2,
        "AV_PENETRATION_RATE": eta,
        "AV_TOLL_DISCOUNT": 1.0,  # No discount
        "objective_type": "lane_weighted",
        **BEST_PARAMS
    })

# --- SCENARIO 3 RUNS ---
# C++ will use SCENARIO_MODE = 2 (routing considers tolls)
# The difference is the objective function calculated in Python.
for eta in AV_PENETRATION_RATES:
    EXPERIMENTS.append({
        "name": f"S3_FullCollab_eta_{eta:.1f}",
        "SCENARIO_MODE": 2,
        "AV_PENETRATION_RATE": eta,
        "AV_TOLL_DISCOUNT": 0.8,  # AVs get a 20% discount
        "objective_type": "flow_weighted_S3",
        **BEST_PARAMS
    })

# --- General Configuration ---
MAX_ITERATIONS = 50
TOLL_MIN = 0.0
TOLL_MAX = 20.0
EDGES_INPUT_FILE = "edges.csv"
SIMULATOR_WORKING_DIR = "LivingCity"
CONFIG_FILE_NAME = "command_line_options.ini"
SIM_OUTPUT_FILE_NAME = "results.csv"
SIMULATOR_EXECUTABLE = "./LivingCity"
CONVERGENCE_PATIENCE = 10
CONVERGENCE_TOLERANCE = 10.0
MAIN_RESULTS_DIR = "sensitivity_analysis_results"


def update_ini_file_safely(ini_path: str, params_to_update: dict):
    if not os.path.exists(ini_path):
        print(f"Error: Cannot find config file at '{ini_path}'", flush=True)
        exit()
    with open(ini_path, 'r') as f:
        lines = f.readlines()
    found_keys = {key.upper(): False for key in params_to_update}
    for i, line in enumerate(lines):
        for key, value in params_to_update.items():
            if re.match(fr'^\s*{re.escape(key)}\s*=', line, re.IGNORECASE):
                lines[i] = f"{key.upper()} = {value}\n"
                found_keys[key.upper()] = True
                break
    for key, value in params_to_update.items():
        if not found_keys[key.upper()]:
            lines.append(f"{key.upper()} = {value}\n")
    with open(ini_path, 'w') as f:
        f.writelines(lines)
    # print(f"Safely updated '{ini_path}' with new parameters.", flush=True)


def run_simulation():
    print(f"Executing C++ simulator...", flush=True)
    try:
        subprocess.run(
            [SIMULATOR_EXECUTABLE], check=True, capture_output=True,
            text=True, timeout=3600, cwd=SIMULATOR_WORKING_DIR
        )
        print("C++ Simulator executed successfully.", flush=True)
    except Exception as e:
        print(f"An error occurred during C++ simulation: {e}", flush=True)
        exit()


def update_tolls_tunable_MSA(iter_df: pd.DataFrame, params: dict) -> pd.Series:
    theta1 = params["THETA1_REVENUE_WEIGHT"]
    theta2 = params["THETA2_CONGESTION_WEIGHT"]
    learning_rate = params["LEARNING_RATE"]
    inertia = params["INERTIA"]

    total_flow = iter_df['final_flow_hv'] + iter_df['final_flow_av']
    weighted_time_numerator = (iter_df['final_travel_time_hv'] * iter_df['final_flow_hv'] +
                               iter_df['final_travel_time_av'] * iter_df['final_flow_av'])
    avg_travel_time = np.divide(weighted_time_numerator, total_flow,
                                out=np.zeros_like(weighted_time_numerator, dtype=float),
                                where=(total_flow != 0))

    marginal_congestion_cost = theta2 * (avg_travel_time - iter_df['free_flow_time'])
    target_toll = marginal_congestion_cost
    gradient_step = learning_rate * theta1 * (target_toll - iter_df['toll_fee'])

    # We apply max_toll_change here, which was missing in some previous versions
    max_toll_change = params["MAX_TOLL_CHANGE"]
    constrained_gradient_step = gradient_step.clip(-max_toll_change, max_toll_change)

    old_toll = iter_df['toll_fee']
    new_toll_proposal = old_toll + constrained_gradient_step
    new_toll = (1 - inertia) * old_toll + inertia * new_toll_proposal

    final_new_toll = new_toll.clip(lower=TOLL_MIN, upper=TOLL_MAX)
    final_new_toll.index = iter_df['uniqueid']
    return final_new_toll


def run_single_experiment(exp_params: dict, MAX_ITERATIONS: int):
    output_dir = os.path.join(MAIN_RESULTS_DIR, exp_params["name"])
    os.makedirs(output_dir, exist_ok=True)
    print(f"Results for this experiment will be saved in: '{output_dir}'")

    edges_df = pd.read_csv(EDGES_INPUT_FILE)
    edges_df.drop_duplicates(subset=['uniqueid'], keep='first', inplace=True)
    edges_df['free_flow_time'] = edges_df['length'] / (edges_df['speed_mph'] * 0.44704)
    current_tolls = pd.Series(0.0, index=edges_df['uniqueid'])

    summary_data = []
    best_objective = -np.inf
    patience_counter = 0

    for i in range(MAX_ITERATIONS):
        print(f"\n--- Iteration {i + 1}/{MAX_ITERATIONS} ---", flush=True)

        params_for_ini = {
            "TOLL_FILE_PATH": f"tolls_for_iter_{i}.csv",
            "AV_PENETRATION_RATE": exp_params.get("AV_PENETRATION_RATE", 0.0),
            "AV_TOLL_DISCOUNT": exp_params.get("AV_TOLL_DISCOUNT", 1.0),
            "SCENARIO_MODE": exp_params.get("SCENARIO_MODE", 2)  # Default to S2/S3 logic
        }

        toll_update_df = pd.DataFrame({'edge_id': current_tolls.index, 'toll_fee': current_tolls.values})
        temp_toll_filepath = os.path.join(SIMULATOR_WORKING_DIR, params_for_ini["TOLL_FILE_PATH"])
        toll_update_df.to_csv(temp_toll_filepath, index=False)

        config_path_in_sim_dir = os.path.join(SIMULATOR_WORKING_DIR, CONFIG_FILE_NAME)
        update_ini_file_safely(config_path_in_sim_dir, params_for_ini)

        run_simulation()

        sim_output_filepath = os.path.join(SIMULATOR_WORKING_DIR, SIM_OUTPUT_FILE_NAME)
        results_df = pd.read_csv(sim_output_filepath)

        iter_df = pd.merge(edges_df, results_df, left_on='uniqueid', right_on='edge_id', how='left')
        iter_df['toll_fee'] = current_tolls.loc[iter_df['uniqueid']].values

        for col in ['final_flow_hv', 'final_flow_av', 'final_travel_time_hv', 'final_travel_time_av']:
            iter_df[col] = iter_df[col].fillna(0)

        # --- Dynamic objective function calculation ---
        flow_hv = iter_df['final_flow_hv']
        flow_av = iter_df['final_flow_av']
        gamma = exp_params["AV_TOLL_DISCOUNT"]
        revenue = (iter_df['toll_fee'] * flow_hv + gamma * iter_df['toll_fee'] * flow_av).sum()
        total_delay = (iter_df['final_flow_hv'] * (iter_df['final_travel_time_hv'] - iter_df['free_flow_time']) +
                       iter_df['final_flow_av'] * (
                               iter_df['final_travel_time_av'] - iter_df['free_flow_time'])).sum()
        objective_name = "S3 Flow-Weighted"
        objective = (exp_params["THETA1_REVENUE_WEIGHT"] * revenue) - (exp_params["THETA2_CONGESTION_WEIGHT"] * total_delay)

        print(f"  - Objective: {objective:,.2f}", flush=True)

        iteration_filename = os.path.join(output_dir, f"iteration_{i + 1}_details.csv")
        iter_df.to_csv(iteration_filename, index=False, float_format='%.4f')

        summary_data.append({'iteration': i + 1, 'objective': objective, 'revenue': revenue, 'congestion': total_delay})

        simulated_iter_df = iter_df.dropna(subset=['edge_id']).copy()
        newly_calculated_tolls = update_tolls_tunable_MSA(simulated_iter_df, exp_params)
        current_tolls.update(newly_calculated_tolls)

        if objective > best_objective + CONVERGENCE_TOLERANCE:
            best_objective = objective;
            patience_counter = 0
        else:
            patience_counter += 1
        if patience_counter >= CONVERGENCE_PATIENCE:
            print(f"\nConvergence reached at iteration {i + 1}.", flush=True)
            break

    summary_df = pd.DataFrame(summary_data)
    summary_filename = os.path.join(output_dir, "optimization_summary.csv")
    summary_df.to_csv(summary_filename, index=False, float_format='%.2f')
    print(f"\nOptimization summary for '{exp_params['name']}' saved to '{summary_filename}'")


def main():
    """
    Main function to run the baseline and then loop through all defined experiments.
    """
    os.makedirs(MAIN_RESULTS_DIR, exist_ok=True)

    # Run a single baseline for comparison
    print(f"\n\n{'=' * 60}\nRUNNING SCENARIO: Baseline (Zero Tolls)\n{'=' * 60}")
    baseline_params = {**EXPERIMENTS[0], "LEARNING_RATE": 0.0, "MAX_TOLL_CHANGE": 0.0}
    run_single_experiment(exp_params={"name": "Baseline", **baseline_params}, MAX_ITERATIONS=1)

    # Run all defined experiments
    for experiment_params in EXPERIMENTS:
        print(f"\n\n{'=' * 60}\nRUNNING SCENARIO: {experiment_params['name']}\n{'=' * 60}")
        run_single_experiment(exp_params=experiment_params, MAX_ITERATIONS=MAX_ITERATIONS)

    print("\n\nAll scenarios complete.")


if __name__ == "__main__":
    main()
