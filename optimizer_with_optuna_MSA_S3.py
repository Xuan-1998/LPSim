import pandas as pd
import subprocess
import os
import re
import numpy as np
import optuna

"""
optimizer_with_optuna.py

This script uses the Optuna framework to automatically find the best
hyperparameters for a traffic toll optimization problem. It evaluates each
parameter set based on a score that rewards both high performance and
stable convergence.
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
CONVERGENCE_TOLERANCE = 5.0

# --- Optuna Configuration ---
N_TRIALS = 50
OPTUNA_RESULTS_DIR = "optuna_study_results_congestion_MSA_S3"


# ==============================================================================
# --- 2. CORE FUNCTIONS (No changes here) ---
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

def update_tolls_tunable_MSA(iter_df: pd.DataFrame, params: dict) -> pd.Series:
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

# ==============================================================================
# --- 3. EXPERIMENT RUNNER (MODIFIED FOR OPTUNA) ---
# ==============================================================================
def run_single_experiment(scenario_name: str, params: dict, max_iterations: int, study_dir: str) -> float:
    output_dir = os.path.join(study_dir, f"results_{scenario_name}")
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
    if summary_df.empty:
        return -np.inf

    summary_filename = os.path.join(output_dir, "optimization_summary.csv")
    summary_df.to_csv(summary_filename, index=False, float_format='%.2f')
    print(f"\nOptimization summary for '{scenario_name}' saved to '{summary_filename}'", flush=True)

    converged_early = (len(summary_df) < max_iterations)
    stable_iterations = min(len(summary_df), CONVERGENCE_PATIENCE)
    last_objectives = summary_df['objective'].tail(stable_iterations)

    mean_objective = last_objectives.mean()
    std_dev = last_objectives.std()
    if pd.isna(std_dev):
        std_dev = 0.0

    STABILITY_PENALTY_FACTOR = 2
    final_score = mean_objective - STABILITY_PENALTY_FACTOR * std_dev

    if converged_early:
        final_score *= 1.05

    print(f"  - Final score for Optuna: {final_score:,.2f} (Mean Obj: {mean_objective:,.2f}, StdDev: {std_dev:,.2f})")

    return final_score

# ==============================================================================
# --- 4. OPTUNA OBJECTIVE FUNCTION ---
# ==============================================================================
def objective_for_optuna(trial: optuna.Trial, study_dir: str) -> float:
    """
    This is the objective function that Optuna will minimize/maximize.
    It suggests hyperparameters, runs the experiment, and returns the final score.
    """
    params = {
        "name": f"trial_{trial.number}",
        "THETA1_REVENUE_WEIGHT": 1,
        "THETA2_CONGESTION_WEIGHT": 0.0055,
        "AV_PENETRATION_RATE": 0.5,  # Example: 50% AVs
        "AV_TOLL_DISCOUNT": 1.0,  # No discount in this model
        "LEARNING_RATE": trial.suggest_float("LEARNING_RATE", 1e-2, 2.0, log=True),
        "MAX_TOLL_CHANGE": trial.suggest_float("MAX_TOLL_CHANGE", 0.1, 20.0),
        "INERTIA": trial.suggest_float("INERTIA", 0.05, 0.95)
    }

    print(f"\n\n{'=' * 60}", flush=True)
    print(f"STARTING OPTUNA TRIAL {trial.number}", flush=True)
    print(f"PARAMETERS: {params}", flush=True)
    print(f"{'=' * 60}", flush=True)

    try:
        score = run_single_experiment(
            scenario_name=params["name"],
            params=params,
            max_iterations=MAX_ITERATIONS,
            study_dir=study_dir
        )
        return score
    except Exception as e:
        print(f"TRIAL {trial.number} FAILED due to an error: {e}", flush=True)
        raise optuna.exceptions.TrialPruned()


# ==============================================================================
# --- 5. MAIN EXECUTION BLOCK ---
# ==============================================================================
def main():
    """
    Main function to run the baseline and then use Optuna for hyperparameter optimization.
    """
    os.makedirs(OPTUNA_RESULTS_DIR, exist_ok=True)
    print(f"All optimization results will be saved in '{OPTUNA_RESULTS_DIR}' folder.")

    # 1. Run the baseline scenario for comparison
    print(f"\n\n{'=' * 60}", flush=True) # <-- THIS IS THE CORRECTED LINE
    print("RUNNING SCENARIO: Baseline (Zero Tolls)", flush=True)
    print(f"{'=' * 60}", flush=True)
    baseline_params = {
        "THETA1_REVENUE_WEIGHT": 1.0, "THETA2_CONGESTION_WEIGHT": 0.0055,
        'LEARNING_RATE': 0.0, 'MAX_TOLL_CHANGE': 0.0, 'INERTIA': 1.0,
        "AV_PENETRATION_RATE": 0.5,  # Example: 50% AVs
        "AV_TOLL_DISCOUNT": 1.0}
    run_single_experiment(
        scenario_name="baseline",
        params=baseline_params,
        max_iterations=1,
        study_dir=OPTUNA_RESULTS_DIR
    )

    # 2. Set up and run the Optuna study
    study = optuna.create_study(direction="maximize")
    try:
        study.optimize(
            lambda trial: objective_for_optuna(trial, study_dir=OPTUNA_RESULTS_DIR),
            n_trials=N_TRIALS
        )
    except KeyboardInterrupt:
        print("Optimization stopped by user.")
    except Exception as e:
        print(f"An unexpected error occurred during optimization: {e}")

    # 3. Print the results
    print("\n\n--- OPTIMIZATION FINISHED ---")
    print(f"Number of finished trials: {len(study.trials)}")

    pruned_trials = study.get_trials(deepcopy=False, states=[optuna.trial.TrialState.PRUNED])
    complete_trials = study.get_trials(deepcopy=False, states=[optuna.trial.TrialState.COMPLETE])

    print(f"  - Pruned (failed) trials: {len(pruned_trials)}")
    print(f"  - Complete trials: {len(complete_trials)}")

    if complete_trials:
        print("\nBest trial:")
        trial = study.best_trial
        print(f"  Value (Final Score): {trial.value:,.4f}")
        print("  Best Parameters: ")
        for key, value in trial.params.items():
            print(f"    {key}: {value}")
    else:
        print("No trials completed successfully.")


if __name__ == "__main__":
    main()