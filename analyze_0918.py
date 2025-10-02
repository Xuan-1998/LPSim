# analyze_log.py (Final version, saves all iteration data + summary + plots)

import re
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


def parse_log_file(log_file_path):
    """
    Parses the output log file from optimizer_MSA_S3.py,
    even if all experiments were incorrectly named 'Best_Params'.
    """
    print(f"Attempting to read and parse log file: {log_file_path}")
    try:
        with open(log_file_path, 'r') as f:
            lines = f.readlines()
    except FileNotFoundError:
        print(f"ERROR: File not found at '{log_file_path}'.")
        print("Please make sure this script and the log file are in the same directory.")
        return None

    # Based on your original script, we know the true execution order of the experiments
    AV_PENETRATION_RATES = [0.0, 0.1, 0.3, 0.5, 0.7, 1.0]
    true_experiment_names = ["Baseline"]

    # Build the list of experiment names for S1, S2, S3
    for scenario_num in [1, 2, 3]:
        if scenario_num == 1:
            prefix = "S1_Competition"
        elif scenario_num == 2:
            prefix = "S2_PartialCollab"
        else:
            prefix = "S3_FullCollab"

        for eta in AV_PENETRATION_RATES:
            true_experiment_names.append(f"{prefix}_eta_{eta:.1f}")

    all_iterations_data = []
    experiment_counter = -1
    current_iteration = 0
    objective_regex = re.compile(r"Objective: ([-\d.,]+)")

    for line in lines:
        if "RUNNING SCENARIO:" in line:
            experiment_counter += 1
            current_iteration = 0
            if experiment_counter >= len(true_experiment_names):
                print(
                    f"Warning: More experiments found in log than expected. Stopping at experiment {experiment_counter}.")
                break
            continue

        match = objective_regex.search(line)
        if match and experiment_counter >= 0:
            current_iteration += 1
            objective_str = match.group(1).replace(',', '')
            objective_val = float(objective_str)
            exp_name = true_experiment_names[experiment_counter]
            all_iterations_data.append({
                "experiment_name": exp_name,
                "iteration": current_iteration,
                "objective": objective_val
            })

    print("Log file parsed successfully!")
    return pd.DataFrame(all_iterations_data)


def main():
    # --- 1. Parse the Log File ---
    log_file = 'output_optimizer_MSA_S3_new_2.txt'
    df_iterations = parse_log_file(log_file)

    if df_iterations is None or df_iterations.empty:
        print("Stopping program because no data could be parsed.")
        return

    # ==============================================================================
    # 新增功能: 保存所有实验的【完整迭代过程】
    # ==============================================================================
    iterations_filename = 'rescued_all_iterations_data.csv'
    df_iterations.to_csv(iterations_filename, index=False, float_format='%.2f')
    print(f"\n所有实验的【完整迭代数据】已成功保存至: '{iterations_filename}'")

    # --- 2. Extract Final Results for summary table and plot ---
    df_final = df_iterations.loc[df_iterations.groupby('experiment_name')['iteration'].idxmax()]

    # Process non-Baseline experiments
    df_main_processed = df_final[df_final['experiment_name'] != 'Baseline'].copy()
    split_data = df_main_processed['experiment_name'].str.extract(r'(S\d_[a-zA-Z]+)_eta_([\d\.]+)')
    split_data.columns = ['scenario', 'eta']

    df_main_processed['scenario'] = split_data['scenario']
    df_main_processed['eta'] = pd.to_numeric(split_data['eta'])

    # Process the Baseline experiment
    baseline_row = df_final[df_final['experiment_name'] == 'Baseline'].copy()
    if not baseline_row.empty:
        baseline_row['scenario'] = 'Baseline'
        baseline_row['eta'] = 0.0

    # Combine Baseline and main data into the final DataFrame
    df_final_processed = pd.concat([baseline_row, df_main_processed], ignore_index=True)

    # Clean up unnecessary columns and sort the data
    df_final_processed = df_final_processed[['scenario', 'eta', 'objective']].sort_values(by=['scenario', 'eta'])

    print("\n--- Rescued Final Experiment Results from Log File (Summary) ---")
    print(df_final_processed.to_string(index=False))

    # Save the summary table to a CSV file
    summary_filename = 'rescued_summary_results.csv'
    df_final_processed.to_csv(summary_filename, index=False, float_format='%.2f')
    print(f"\n【摘要表格】已成功保存至: '{summary_filename}'")

    # --- 3. Data Visualization ---
    plt.style.use('seaborn-v0_8-whitegrid')

    # Plot 1: Plotting ALL convergence curves using subplots
    scenarios_for_plot1 = ['S1_Competition', 'S2_PartialCollab', 'S3_FullCollab']
    fig1, axes = plt.subplots(3, 1, figsize=(14, 20), sharex=True)
    fig1.suptitle('All Experiment Convergence Curves by Scenario', fontsize=20, y=0.95)

    for i, scenario_name in enumerate(scenarios_for_plot1):
        ax = axes[i]
        scenario_df = df_iterations[df_iterations['experiment_name'].str.startswith(scenario_name)]
        experiments_in_scenario = sorted(scenario_df['experiment_name'].unique())

        for exp_name in experiments_in_scenario:
            subset = scenario_df[scenario_df['experiment_name'] == exp_name]
            eta_label = exp_name.split('eta_')[-1]
            ax.plot(subset['iteration'], subset['objective'], marker='.', linestyle='-', label=f'eta = {eta_label}')

        ax.set_title(f'Scenario: {scenario_name}', fontsize=16)
        ax.set_ylabel('Objective Value', fontsize=12)
        ax.legend(title='AV Penetration')
        ax.grid(True)

    axes[-1].set_xlabel('Iteration', fontsize=12)
    plt.tight_layout(rect=[0, 0, 1, 0.93])

    convergence_plot_file = 'rescued_ALL_convergence_curves.png'
    plt.savefig(convergence_plot_file)
    print(f"\n已生成【所有实验】的收敛曲线图: '{convergence_plot_file}'")

    # Plot 2: Final Objective Value vs. AV Penetration (Core Result Plot)
    fig2, ax2 = plt.subplots(figsize=(12, 8))

    colors = {'S1_Competition': '#1f77b4', 'S2_PartialCollab': '#ff7f0e', 'S3_FullCollab': '#2ca02c',
              'Baseline': 'black'}
    markers = {'S1_Competition': 'o', 'S2_PartialCollab': 's', 'S3_FullCollab': '^', 'Baseline': 'x'}

    scenarios_to_plot = sorted([s for s in df_final_processed['scenario'].unique() if s != 'Baseline'])

    for scenario in scenarios_to_plot:
        subset = df_final_processed[df_final_processed['scenario'] == scenario]
        ax2.plot(subset['eta'], subset['objective'],
                 marker=markers[scenario],
                 linestyle='--',
                 color=colors[scenario],
                 label=scenario)

    if not baseline_row.empty:
        ax2.scatter(baseline_row['eta'], baseline_row['objective'],
                    marker=markers['Baseline'], color=colors['Baseline'],
                    label='Baseline', zorder=5, s=100)

    ax2.set_title('Final Objective Value vs. AV Penetration by Scenario', fontsize=16)
    ax2.set_xlabel('AV Penetration Rate (eta)', fontsize=12)
    ax2.set_ylabel('Final Objective Value', fontsize=12)
    ax2.legend(title='Scenario')
    ax2.grid(True)
    ax2.xaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: f'{int(x * 100)}%'))
    plt.tight_layout()
    final_results_plot_file = 'rescued_final_results_plot.png'
    plt.savefig(final_results_plot_file)
    print(f"已生成【最终结果对比图】: '{final_results_plot_file}'")


if __name__ == '__main__':
    main()