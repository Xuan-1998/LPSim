import pandas as pd
import subprocess
import os
import re
import numpy as np

"""
optimizer.py (S3 Model Version)

This script runs multiple optimization scenarios, including the advanced S3 model
with AV penetration rates and toll discounts. It passes these parameters to the
C++ simulator and calculates the appropriate objective function based on the
disaggregated AV/HV results.
"""

# ==============================================================================
# --- 1. DEFINE YOUR EXPERIMENTS HERE ---
# ==============================================================================
# Define your three scenarios based on our discussion.
SCENARIOS = [

    {
        "name": "Scenario2_Partial_Collab",
        "objective_type": "lane_weighted",
        "AV_PENETRATION_RATE": 0.3,  # Example: 30% AVs
        "AV_TOLL_DISCOUNT": 1.0,  # No discount in this model
        "THETA1_REVENUE_WEIGHT": 1.0,
        "THETA2_CONGESTION_WEIGHT": 0.0055,
        "LEARNING_RATE": 0.2,
        "MAX_TOLL_CHANGE": 2.0,
        "INERTIA": 0.7
        # NOTE: For this scenario, the cost-based lane change in C++ must be enabled.
    },
    {
        "name": "Scenario3_Full_Collab_Discount",
        "objective_type": "flow_weighted_S3",  # A new type for the S3 revenue formula
        "AV_PENETRATION_RATE": 0.3,  # Example: 30% AVs
        "AV_TOLL_DISCOUNT": 0.8,  # AVs get a 20% discount
        "THETA1_REVENUE_WEIGHT": 1.0,
        "THETA2_CONGESTION_WEIGHT": 0.0055,
        "LEARNING_RATE": 0.2,
        "MAX_TOLL_CHANGE": 2.0,
        "INERTIA": 0.7
        # NOTE: For this scenario, the cost-based lane change in C++ must be enabled.
    },
]

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
CONVERGENCE_TOLERANCE = 5.0


# --- NEW: More flexible INI updater ---
def update_ini_file_safely(ini_path: str, params_to_update: dict):
    """
    Safely reads an .ini file, updates/adds multiple key-value pairs,
    and writes it back without altering the original structure.
    """
    if not os.path.exists(ini_path):
        print(f"Error: Cannot find config file at '{ini_path}'", flush=True)
        exit()
    with open(ini_path, 'r') as f:
        lines = f.readlines()

    # Create a dictionary of keys we've found to manage updates
    found_keys = {key.upper(): False for key in params_to_update}

    for i, line in enumerate(lines):
        for key, value in params_to_update.items():
            # Check if the line starts with the key (case-insensitive)
            if re.match(fr'^\s*{re.escape(key)}\s*=', line, re.IGNORECASE):
                lines[i] = f"{key.upper()} = {value}\n"
                found_keys[key.upper()] = True
                break  # Move to the next line

    # Add any keys that were not found in the file
    for key, value in params_to_update.items():
        if not found_keys[key.upper()]:
            lines.append(f"{key.upper()} = {value}\n")

    with open(ini_path, 'w') as f:
        f.writelines(lines)
    print(f"Safely updated '{ini_path}' with new parameters.", flush=True)


def run_simulation():
    # This function remains unchanged
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


def update_tolls_tunable_MSA(iter_df: pd.DataFrame, params: dict) -> pd.Series:
    # Extract all relevant parameters from the dictionary
    theta1 = params["THETA1_REVENUE_WEIGHT"]
    theta2 = params["THETA2_CONGESTION_WEIGHT"]
    learning_rate = params["LEARNING_RATE"]
    max_toll_change = params["MAX_TOLL_CHANGE"]
    inertia = params["INERTIA"]

    total_flow = iter_df['final_flow_hv'] + iter_df['final_flow_av']
    weighted_time_numerator = (iter_df['final_travel_time_hv'] * iter_df['final_flow_hv'] +
                               iter_df['final_travel_time_av'] * iter_df['final_flow_av'])

    iter_df['final_travel_time'] = np.divide(weighted_time_numerator,
                                             total_flow,
                                             out=np.zeros_like(weighted_time_numerator, dtype=float),
                                             where=(total_flow != 0))

    marginal_congestion_cost = theta2 * (iter_df['final_travel_time'] - iter_df['free_flow_time'])
    target_toll = marginal_congestion_cost
    gradient_step = learning_rate * theta1 * (target_toll - iter_df['toll_fee'])
    constrained_gradient_step = gradient_step.clip(-max_toll_change, max_toll_change)

    old_toll = iter_df['toll_fee']
    new_toll_proposal = old_toll + constrained_gradient_step
    new_toll = (1 - inertia) * old_toll + inertia * new_toll_proposal

    final_new_toll = new_toll.clip(lower=TOLL_MIN, upper=TOLL_MAX)
    final_new_toll.index = iter_df['uniqueid']

    print(f"Toll fees updated using Tunable MSA (inertia={inertia:.3f}).", flush=True)
    return final_new_toll

def run_single_experiment(scenario_name: str, params: dict, max_iterations: int):
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

        # --- KEY CHANGE: Prepare all dynamic parameters for the .ini file ---
        params_for_ini = {
            "TOLL_FILE_PATH": f"tolls_for_iter_{i}.csv",
            "AV_PENETRATION_RATE": params.get("AV_PENETRATION_RATE", 0.0),
            "AV_TOLL_DISCOUNT": params.get("AV_TOLL_DISCOUNT", 1.0)
        }

        toll_update_df = pd.DataFrame({'edge_id': current_tolls.index, 'toll_fee': current_tolls.values})
        temp_toll_filepath = os.path.join(SIMULATOR_WORKING_DIR, params_for_ini["TOLL_FILE_PATH"])
        toll_update_df.to_csv(temp_toll_filepath, index=False)

        config_path_in_sim_dir = os.path.join(SIMULATOR_WORKING_DIR, CONFIG_FILE_NAME)
        update_ini_file_safely(config_path_in_sim_dir, params_for_ini)

        run_simulation()

        print("Processing simulation results...", flush=True)
        sim_output_filepath = os.path.join(SIMULATOR_WORKING_DIR, SIM_OUTPUT_FILE_NAME)
        results_df = pd.read_csv(sim_output_filepath)

        iter_df = pd.merge(edges_df, results_df, left_on='uniqueid', right_on='edge_id', how='left')
        iter_df['toll_fee'] = current_tolls.loc[iter_df['uniqueid']].values

        # Fill NaNs for new disaggregated columns
        for col in ['final_flow_hv', 'final_flow_av', 'final_travel_time_hv', 'final_travel_time_av']:
            iter_df[col] = iter_df[col].fillna(0)

        # --- KEY CHANGE: Dynamic objective function calculation ---

        flow_hv = iter_df['final_flow_hv']
        flow_av = iter_df['final_flow_av']
        gamma = params["AV_TOLL_DISCOUNT"]
        revenue = (iter_df['toll_fee'] * flow_hv + gamma * iter_df['toll_fee'] * flow_av).sum()
        total_delay = (iter_df['final_flow_hv'] * (iter_df['final_travel_time_hv'] - iter_df['free_flow_time']) +
                       iter_df['final_flow_av'] * (
                               iter_df['final_travel_time_av'] - iter_df['free_flow_time'])).sum()
        objective_name = "S3 Flow-Weighted"

        """objective_type = params.get("objective_type", "flow_weighted_S3")
        if objective_type == "flow_weighted_S3":
            flow_hv = iter_df['final_flow_hv']
            flow_av = iter_df['final_flow_av']
            gamma = params["AV_TOLL_DISCOUNT"]
            revenue = (iter_df['toll_fee'] * flow_hv + gamma * iter_df['toll_fee'] * flow_av).sum()
            total_delay = (iter_df['final_flow_hv'] * (iter_df['final_travel_time_hv'] - iter_df['free_flow_time']) +
                           iter_df['final_flow_av'] * (
                                       iter_df['final_travel_time_av'] - iter_df['free_flow_time'])).sum()
            objective_name = "S3 Flow-Weighted"
        else:  
            revenue = (iter_df['toll_fee'] * (iter_df['final_flow_hv'] + iter_df['final_flow_av'])).sum()
            total_delay = (iter_df['lanes'] * (iter_df['final_travel_time_hv'] - iter_df['free_flow_time'])).sum()
            objective_name = "Lane-Weighted"
        """

        objective = (params["THETA1_REVENUE_WEIGHT"] * revenue) - (params["THETA2_CONGESTION_WEIGHT"] * total_delay)

        print(f"  - Objective ({objective_name}): {objective:,.2f}", flush=True)

        iteration_filename = os.path.join(output_dir, f"iteration_{i + 1}_details.csv")
        iter_df.to_csv(iteration_filename, index=False, float_format='%.4f')
        print(f"  - Detailed results saved to '{iteration_filename}'", flush=True)

        summary_data.append({'iteration': i + 1, 'objective': objective, 'revenue': revenue, 'congestion': total_delay})

        simulated_iter_df = iter_df.dropna(subset=['edge_id']).copy()
        newly_calculated_tolls = update_tolls_tunable_MSA(
            simulated_iter_df,
            params
        )
        current_tolls.update(newly_calculated_tolls)

        if max_iterations > 1:
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
    print(f"\nOptimization summary for '{scenario_name}' saved to '{summary_filename}'", flush=True)


def main():
    baseline_params = SCENARIOS[0].copy()
    baseline_params['LEARNING_RATE'] = 0.0
    baseline_params['MAX_TOLL_CHANGE'] = 0.0
    run_single_experiment(scenario_name="baseline", params=baseline_params, max_iterations=1)

    for scenario in SCENARIOS:
        print(f"\n\n{'=' * 60}", flush=True)
        print(f"RUNNING SCENARIO: {scenario['name']}", flush=True)
        print(f"{'=' * 60}", flush=True)
        run_single_experiment(scenario_name=scenario['name'], params=scenario, max_iterations=MAX_ITERATIONS)

    print("\n\nAll scenarios complete.", flush=True)


if __name__ == "__main__":
    main()