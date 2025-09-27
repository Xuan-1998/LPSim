import pandas as pd
import subprocess
import os
import re
import numpy as np
import optuna
from datetime import datetime

# Import the optimizer from SciPy
from scipy.optimize import minimize

# ==============================================================================
# --- 1. CONFIGURATION ---
# ==============================================================================
# --- General Configuration ---
# MAX_ITERATIONS is now for the SciPy optimizer, representing the max
# number of times it will call the simulator.
MAX_ITERATIONS = 50
TOLL_MIN = 0.0
TOLL_MAX = 20.0
EDGES_INPUT_FILE = "edges.csv"
SIMULATOR_WORKING_DIR = "LivingCity"
CONFIG_FILE_NAME = "command_line_options.ini"
SIM_OUTPUT_FILE_NAME = "results.csv"
SIMULATOR_EXECUTABLE = "./LivingCity"

# --- Optuna Configuration ---
N_TRIALS = 20 # Can be reduced as each trial is now more computationally expensive.

# ==============================================================================
# --- 2. CORE FUNCTIONS ---
# ==============================================================================
def update_ini_file_safely(ini_path: str, toll_filepath: str):
    if not os.path.exists(ini_path):
        print(f"Error: Cannot find config file at '{ini_path}'", flush=True)
        # In an optimizer context, it's better to raise an exception.
        raise FileNotFoundError(f"Config file not found at {ini_path}")
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
    print(f"Executing C++ simulator...", flush=True)
    try:
        subprocess.run(
            [SIMULATOR_EXECUTABLE], check=True, capture_output=True,
            text=True, timeout=3600, cwd=SIMULATOR_WORKING_DIR
        )
    except Exception as e:
        print(f"An error occurred during C++ simulation: {e}", flush=True)
        raise e

# The update_tolls_objective_driven function is no longer needed.
# SciPy handles the update logic.

# ==============================================================================
# --- 3. SCIPY OBJECTIVE FUNCTION ---
# ==============================================================================
def objective_for_scipy(tolls_array: np.ndarray, edges_df: pd.DataFrame, params: dict, scenario_name: str) -> float:
    """
    This function is called by SciPy's optimizer.
    It takes an array of tolls, runs the simulation, and returns a scalar value to be minimized.
    """
    # 1. Prepare the toll file
    # Convert the 1D numpy array from SciPy into a DataFrame
    toll_update_df = pd.DataFrame({
        'edge_id': edges_df['uniqueid'],
        'toll_fee': tolls_array
    })

    # Use a unique temporary filename to prevent conflicts during parallel runs
    temp_toll_filename = f"tolls_{scenario_name}.csv"
    temp_toll_filepath_for_sim = os.path.join(SIMULATOR_WORKING_DIR, temp_toll_filename)
    toll_update_df.to_csv(temp_toll_filepath_for_sim, index=False)

    config_path_in_sim_dir = os.path.join(SIMULATOR_WORKING_DIR, CONFIG_FILE_NAME)
    update_ini_file_safely(config_path_in_sim_dir, temp_toll_filename)

    # 2. Run the simulation
    run_simulation()

    # 3. Calculate the objective value
    sim_output_filepath = os.path.join(SIMULATOR_WORKING_DIR, SIM_OUTPUT_FILE_NAME)
    results_df = pd.read_csv(sim_output_filepath)
    iter_df = pd.merge(edges_df, results_df, left_on='uniqueid', right_on='edge_id', how='left')

    # Use the passed tolls_array to populate the toll_fee column, as this is what the simulator used
    iter_df['toll_fee'] = tolls_array

    iter_df['final_flow'] = iter_df['final_flow'].fillna(0)
    iter_df['final_travel_time'] = iter_df['final_travel_time'].fillna(iter_df['free_flow_time'])

    revenue = (iter_df['toll_fee'] * iter_df['final_flow']).sum()
    total_delay = (iter_df['final_flow'] * (iter_df['final_travel_time'] - iter_df['free_flow_time'])).sum()

    # Our goal is to maximize this objective function
    objective = (params["THETA1_REVENUE_WEIGHT"] * revenue) - (params["THETA2_CONGESTION_WEIGHT"] * total_delay)

    print(f"  - Tolls evaluated. Objective = {objective:,.2f}", flush=True)

    # (KEY!) SciPy is a minimizer, so we must return the negative of our objective
    return -objective


# ==============================================================================
# --- 4. EXPERIMENT RUNNER (MODIFIED FOR SCIPY) ---
# ==============================================================================
def run_single_experiment(scenario_name: str, params: dict, max_iterations: int, main_output_dir: str) -> float:
    """
    Runs one full optimization experiment by setting up and calling the SciPy optimizer.
    Returns the final score for Optuna.
    """
    # Create a subdirectory for this specific trial's results
    trial_output_dir = os.path.join(main_output_dir, scenario_name)
    os.makedirs(trial_output_dir, exist_ok=True)

    edges_df = pd.read_csv(EDGES_INPUT_FILE)
    edges_df.drop_duplicates(subset=['uniqueid'], keep='first', inplace=True)
    edges_df['free_flow_time'] = edges_df['length'] / (edges_df['speed_mph'] * 0.44704)

    num_toll_edges = len(edges_df)

    # 1. Define the initial guess and bounds for the optimization
    initial_tolls = np.zeros(num_toll_edges) # Start from zero tolls
    bounds = [(TOLL_MIN, TOLL_MAX)] * num_toll_edges

    print("-" * 50, flush=True)
    print(f"Starting SciPy optimization for scenario '{scenario_name}'...", flush=True)
    print(f"Optimizer: Nelder-Mead, Max Iterations: {max_iterations}", flush=True)

    # 2. Call SciPy's minimize function
    # We use a lambda function to pass the extra arguments (edges_df, params, etc.) to objective_for_scipy
    result = minimize(
        fun=lambda tolls: objective_for_scipy(tolls, edges_df, params, scenario_name),
        x0=initial_tolls,
        method='Nelder-Mead',  # Or change to 'COBYLA'
        bounds=bounds,         # Nelder-Mead doesn't strictly enforce bounds, but SciPy handles this. COBYLA uses them as constraints.
        options={'maxiter': max_iterations, 'disp': True} # 'disp': True prints detailed optimizer output
    )

    # 3. Process and save the results
    if result.success:
        print("\nSciPy optimization finished successfully.", flush=True)
        # Negate the result again to get our original maximized objective value
        final_objective = -result.fun
        best_tolls = result.x
        print(f"  - Final Objective: {final_objective:,.2f}", flush=True)
    else:
        print("\nSciPy optimization did not converge.", flush=True)
        print(f"  - Reason: {result.message}", flush=True)
        # Return a very poor score to Optuna if the optimization failed
        return -np.inf

    # Save the optimal toll configuration for this trial
    final_tolls_df = pd.DataFrame({'edge_id': edges_df['uniqueid'], 'optimized_toll': best_tolls})
    tolls_filename = os.path.join(trial_output_dir, "optimized_tolls.csv")
    final_tolls_df.to_csv(tolls_filename, index=False, float_format='%.4f')
    print(f"Optimized tolls for this trial saved to '{tolls_filename}'", flush=True)

    # For Optuna, we directly return the best objective value found by the optimizer
    return final_objective

# ==============================================================================
# --- 5. OPTUNA OBJECTIVE FUNCTION ---
# ==============================================================================
def objective_for_optuna(trial: optuna.Trial, main_output_dir: str) -> float:
    """
    The objective function for Optuna. It now suggests problem "weights"
    instead of optimization algorithm parameters.
    """
    params = {
        "name": f"trial_{trial.number}",
        # Fix the revenue weight, or let Optuna explore it as well
        "THETA1_REVENUE_WEIGHT": 1.0,
        # Let Optuna find the best congestion penalty weight
        "THETA2_CONGESTION_WEIGHT": trial.suggest_float("THETA2_CONGESTION_WEIGHT", 0.01, 1.0),
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
            main_output_dir=main_output_dir # Pass the main output directory
        )
        return score
    except Exception as e:
        print(f"TRIAL {trial.number} FAILED due to an error: {e}", flush=True)
        raise optuna.exceptions.TrialPruned()

# ==============================================================================
# --- 6. MAIN EXECUTION BLOCK ---
# ==============================================================================
def main():
    """
    Main function to run the baseline and then use Optuna for hyperparameter optimization.
    """
    # Set the main results directory name based on the optimizer package
    optimizer_name = "scipy"
    main_results_dir = f"optimization_run_{optimizer_name}"
    os.makedirs(main_results_dir, exist_ok=True)
    print(f"All results for this run will be saved in: '{main_results_dir}'")

    # 1. Run the baseline scenario with zero tolls
    print(f"\n\n{'=' * 60}", flush=True)
    print("RUNNING SCENARIO: Baseline (Zero Tolls)", flush=True)
    print(f"{'=' * 60}", flush=True)

    # Create a temporary zero-toll file for the baseline simulation
    edges_df = pd.read_csv(EDGES_INPUT_FILE)
    zero_tolls = pd.DataFrame({'edge_id': edges_df['uniqueid'], 'toll_fee': 0.0})
    zero_toll_path = os.path.join(SIMULATOR_WORKING_DIR, "tolls_baseline.csv")
    zero_tolls.to_csv(zero_toll_path, index=False)
    config_path = os.path.join(SIMULATOR_WORKING_DIR, CONFIG_FILE_NAME)
    update_ini_file_safely(config_path, "tolls_baseline.csv")
    run_simulation()

    # Copy baseline results to the main results directory
    baseline_output_dir = os.path.join(main_results_dir, "baseline")
    os.makedirs(baseline_output_dir, exist_ok=True)
    baseline_sim_results = os.path.join(SIMULATOR_WORKING_DIR, SIM_OUTPUT_FILE_NAME)
    if os.path.exists(baseline_sim_results):
        # Using os.rename to move the file
        os.rename(baseline_sim_results, os.path.join(baseline_output_dir, SIM_OUTPUT_FILE_NAME))
    print("Baseline simulation complete. Results copied to baseline subfolder.")


    # 2. Set up and run the Optuna study
    study = optuna.create_study(direction="maximize")
    try:
        # Use a lambda to pass the main_results_dir to the objective function
        study.optimize(lambda trial: objective_for_optuna(trial, main_results_dir), n_trials=N_TRIALS)
    except KeyboardInterrupt:
        print("Optimization stopped by user.")
    except Exception as e:
        print(f"An unexpected error occurred during optimization: {e}")

    # 3. Print and save the final results
    print("\n\n--- OPTIMIZATION FINISHED ---")
    pruned_trials = study.get_trials(deepcopy=False, states=[optuna.trial.TrialState.PRUNED])
    complete_trials = study.get_trials(deepcopy=False, states=[optuna.trial.TrialState.COMPLETE])

    print(f"Number of finished trials: {len(study.trials)}")
    print(f"  - Pruned (failed) trials: {len(pruned_trials)}")
    print(f"  - Complete trials: {len(complete_trials)}")

    if complete_trials:
        # Save the study summary dataframe to a CSV file
        summary_df = study.trials_dataframe()
        summary_df.to_csv(os.path.join(main_results_dir, "study_summary.csv"), index=False)
        print(f"\nFull study summary saved to '{os.path.join(main_results_dir, 'study_summary.csv')}'")

        print("\nBest trial:")
        trial = study.best_trial
        print(f"  Value (Final Objective): {trial.value:,.4f}")
        print("  Best Parameters: ")
        for key, value in trial.params.items():
            print(f"    {key}: {value}")

        # Save the best trial's information to a text file
        with open(os.path.join(main_results_dir, "best_trial_summary.txt"), "w") as f:
            f.write("--- Best Trial Summary ---\n")
            f.write(f"Value (Final Objective): {trial.value:,.4f}\n")
            f.write("Best Parameters:\n")
            for key, value in trial.params.items():
                f.write(f"  {key}: {value}\n")
        print(f"Best trial summary saved to '{os.path.join(main_results_dir, 'best_trial_summary.txt')}'")
    else:
        print("No trials completed successfully.")


if __name__ == "__main__":
    main()