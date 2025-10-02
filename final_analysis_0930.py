import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os
import glob
import numpy as np

try:
    import geopandas as gpd
    GEOPANDAS_AVAILABLE = True
except ImportError:
    GEOPANDAS_AVAILABLE = False

plt.rcParams['savefig.bbox'] = 'tight'

"""
final_analysis.py

This script performs a comprehensive analysis of the sensitivity experiment results.
It automatically discovers all scenario result folders and generates comparative
visualizations for convergence, final performance, equity (AV vs. HV travel time),
and network performance as a function of AV penetration rate.
"""

# ==============================================================================
# --- 1. Configuration ---
# ==============================================================================

# The main directory where all experiment result folders are stored.
MAIN_RESULTS_DIR = 'sensitivity_analysis_results'

# The name of the baseline folder, used for comparison.
BASELINE_NAME = 'Baseline'

# Output directory for all generated plots and tables.
PLOTS_OUTPUT_DIR = "final_analysis_plots"

# Filepath for the network GeoJSON, required for spatial analysis.
NETWORK_GEOJSON_PATH = "network.geojson"
EDGE_ID_COLUMN_IN_GEOJSON = "uniqueid"


# ==============================================================================
# --- 2. Data Loading and Processing Utility ---
# ==============================================================================

def load_all_results(results_dir: str) -> (pd.DataFrame, pd.DataFrame):
    """
    Loads and combines summary and final detailed results from all subdirectories.
    """
    all_summaries = []
    all_final_details = []

    # Find all subdirectories in the main results folder
    scenario_folders = glob.glob(os.path.join(results_dir, 'S*'))  # S* for S1, S2, S3
    if os.path.exists(os.path.join(results_dir, BASELINE_NAME)):
        scenario_folders.append(os.path.join(results_dir, BASELINE_NAME))

    for folder in scenario_folders:
        scenario_name = os.path.basename(folder)
        summary_file = os.path.join(folder, 'optimization_summary.csv')

        print(f"DEBUG: Checking folder: '{folder}'")

        if os.path.exists(summary_file):
            print(f"DEBUG: ---> Found and loading summary file: '{summary_file}'")
            # Load summary data
            summary_df = pd.read_csv(summary_file)
            summary_df['scenario'] = scenario_name
            all_summaries.append(summary_df)

            # Load final iteration's detailed data
            last_iteration = summary_df['iteration'].max()
            details_file = os.path.join(folder, f'iteration_{last_iteration}_details.csv')
            if os.path.exists(details_file):
                details_df = pd.read_csv(details_file)
                details_df['scenario'] = scenario_name
                all_final_details.append(details_df)

    if not all_summaries:
        print("ERROR: No result files found. Please run the optimizer script first.")
        return pd.DataFrame(), pd.DataFrame()

    # --- THIS IS THE FIX ---
    # By adding ignore_index=True, we force pandas to create a new, clean, unique index.
    return pd.concat(all_summaries, ignore_index=True), pd.concat(all_final_details, ignore_index=True)


# ==============================================================================
# --- 3. Analysis and Plotting Functions ---
# ==============================================================================

def analyze_equity_performance(final_details_df: pd.DataFrame):
    """
    Analyzes Equity: Compares the average travel times of AVs vs. HVs.
    Analyzes Network Performance: Compares average speed of AVs vs. HVs.
    """
    print("\n--- (1/3) Generating Equity and Performance Analysis Plots ---")

    # Calculate total flow and average travel times for AV and HV
    final_details_df['total_flow'] = final_details_df['final_flow_hv'] + final_details_df['final_flow_av']

    # We only care about edges with traffic
    df = final_details_df[final_details_df['total_flow'] > 0].copy()

    # Calculate average travel time, weighted by flow
    avg_tt_hv = (df['final_travel_time_hv'] * df['final_flow_hv']).sum() / df['final_flow_hv'].sum()
    avg_tt_av = (df['final_travel_time_av'] * df['final_flow_av']).sum() / df['final_flow_av'].sum()

    # Create a DataFrame for plotting
    equity_data = []
    for scenario in df['scenario'].unique():
        scenario_df = df[df['scenario'] == scenario]
        if scenario_df['final_flow_hv'].sum() > 0:
            avg_tt_hv = (scenario_df['final_travel_time_hv'] * scenario_df['final_flow_hv']).sum() / scenario_df[
                'final_flow_hv'].sum()
            equity_data.append({'scenario': scenario, 'Vehicle Type': 'HV', 'Average Travel Time (s)': avg_tt_hv})
        if scenario_df['final_flow_av'].sum() > 0:
            avg_tt_av = (scenario_df['final_travel_time_av'] * scenario_df['final_flow_av']).sum() / scenario_df[
                'final_flow_av'].sum()
            equity_data.append({'scenario': scenario, 'Vehicle Type': 'AV', 'Average Travel Time (s)': avg_tt_av})

    equity_df = pd.DataFrame(equity_data)

    plt.figure(figsize=(14, 8))
    sns.barplot(data=equity_df, x='scenario', y='Average Travel Time (s)', hue='Vehicle Type', palette='muted')
    plt.title('Equity Analysis: Average Travel Time by Vehicle Type', fontsize=16)
    plt.ylabel('Flow-Weighted Average Travel Time (seconds)', fontsize=12)
    plt.xlabel('Scenario', fontsize=12)
    plt.xticks(rotation=45, ha='right')
    plt.tight_layout()
    output_path = os.path.join(PLOTS_OUTPUT_DIR, "equity_analysis_travel_time.png")
    plt.savefig(output_path, dpi=300)
    plt.close()
    print(f"SUCCESS: Equity analysis plot saved to '{output_path}'")


def analyze_sensitivity_to_penetration_rate(summary_df: pd.DataFrame):
    """
    Analyzes how final system metrics change as AV penetration rate increases.
    """
    print("\n--- (2/3) Generating Sensitivity Analysis Plots (vs. AV Penetration) ---")

    sensitivity_df = summary_df[summary_df['scenario'] != BASELINE_NAME].copy()

    # 2. 检查过滤后是否还有数据，避免没有 S* 场景时报错。
    if sensitivity_df.empty:
        print("WARNING: No sensitivity scenarios (e.g., S1, S2) found to analyze. Skipping this step.")
        return

    # Extract eta and scenario type from the scenario name
    sensitivity_df['eta'] = sensitivity_df['scenario'].str.extract(r'eta_(\d\.\d)').astype(float)
    sensitivity_df['scenario_type'] = sensitivity_df['scenario'].str.split('_').str[0] + '_' + \
                                  sensitivity_df['scenario'].str.split('_').str[1]

    # Get the final iteration for each scenario run
    final_iter_df = sensitivity_df.loc[sensitivity_df.groupby('scenario')['iteration'].idxmax()]

    metrics_to_plot = ['objective', 'revenue', 'congestion']
    for metric in metrics_to_plot:
        plt.figure(figsize=(12, 7))
        sns.lineplot(data=final_iter_df, x='eta', y=metric, hue='scenario_type', marker='o', palette='viridis')
        plt.title(f'Sensitivity of Final {metric.title()} to AV Penetration Rate', fontsize=16)
        plt.xlabel('AV Penetration Rate (η)', fontsize=14)
        plt.ylabel(f'Final {metric.title()}', fontsize=14)
        plt.grid(True)
        plt.legend(title='Scenario Type')
        output_path = os.path.join(PLOTS_OUTPUT_DIR, f"sensitivity_{metric}_vs_eta.png")
        plt.savefig(output_path, dpi=300)
        plt.close()
        print(f"SUCCESS: Sensitivity plot for {metric} saved to '{output_path}'")


def generate_final_comparison_table(summary_df: pd.DataFrame):
    """
    Creates a summary table comparing the final metrics of all scenarios.
    """
    print("\n--- (3/3) Generating Final Performance Comparison Table ---")

    print("\n[DIAGNOSTIC INFO] -----------------------------------------")
    print(f"去重前 (summary_df) 的总行数: {len(summary_df)}")
    print(f"去重前 (summary_df) 的场景数量: {len(summary_df['scenario'].unique())}")
    print("去重前 (summary_df) 各场景出现次数:")
    print(summary_df['scenario'].value_counts().to_string())
    print("----------------------------------------------------------\n")

    final_iter_df = summary_df.loc[summary_df.groupby('scenario')['iteration'].idxmax()].copy()

    # ==================== 诊断代码开始 ====================
    print("\n[DIAGNOSTIC INFO] -----------------------------------------")
    print(f"去重后 (final_iter_df) 的总行数: {len(final_iter_df)}")
    print(f"去重后 (final_iter_df) 的场景数量: {len(final_iter_df['scenario'].unique())}")
    print("去重后 (final_iter_df) 各场景出现次数:")
    print(final_iter_df['scenario'].value_counts().to_string())
    print("----------------------------------------------------------\n")
    # ==================== 诊断代码结束 ====================

    baseline_congestion = final_iter_df[final_iter_df['scenario'] == BASELINE_NAME]['congestion'].iloc[0]

    def calc_change(value):
        return f"{((value - baseline_congestion) / baseline_congestion) * 100:.2f}%"

    final_iter_df['Congestion Change vs Baseline'] = final_iter_df['congestion'].apply(calc_change)

    # Select and format columns for the final report
    report_df = final_iter_df[
        ['scenario', 'objective', 'revenue', 'congestion', 'Congestion Change vs Baseline']].round(2)

    print("\nFinal Performance Summary:\n")
    print(report_df.to_string(index=False))

    output_path = os.path.join(PLOTS_OUTPUT_DIR, "final_performance_summary_table.csv")
    report_df.to_csv(output_path, index=False)
    print(f"\nSUCCESS: Summary table saved to '{output_path}'")


def plot_spatial_distribution(baseline_details_df: pd.DataFrame, optimized_dir: str, network_gdf: gpd.GeoDataFrame):
    """
    Generates spatial distribution maps for a single optimized scenario.
    """
    scenario_name = os.path.basename(optimized_dir)
    print(f"\n--- (4/4) Generating Spatial Maps for '{scenario_name}' ---")

    summary_file = os.path.join(optimized_dir, 'optimization_summary.csv')
    if not os.path.exists(summary_file): return

    last_iteration = pd.read_csv(summary_file)['iteration'].max()
    details_file = os.path.join(optimized_dir, f'iteration_{last_iteration}_details.csv')
    if not os.path.exists(details_file): return

    iter_df = pd.read_csv(details_file)

    # --- UPGRADE: Calculate a single 'optim' travel time for comparison ---
    # Create a flow-weighted average of AV and HV travel times
    total_flow = iter_df['final_flow_hv'] + iter_df['final_flow_av']
    weighted_time_num = (
                iter_df['final_travel_time_hv'] * iter_df['final_flow_hv'] + iter_df['final_travel_time_av'] * iter_df[
            'final_flow_av'])
    iter_df['final_travel_time_optim'] = np.divide(weighted_time_num, total_flow,
                                                   out=np.zeros_like(weighted_time_num, dtype=float),
                                                   where=(total_flow != 0))
    # For edges with no flow, use the free flow time as a neutral value
    mask = iter_df['final_travel_time_optim'] == 0
    iter_df.loc[mask, 'final_travel_time_optim'] = iter_df.loc[mask, 'free_flow_time']

    # Merge with baseline to calculate congestion reduction
    merged_df = pd.merge(iter_df, baseline_details_df[['uniqueid', 'final_travel_time_hv']], on='uniqueid',
                         suffixes=('', '_base'))
    merged_df['congestion_reduction_s'] = merged_df['final_travel_time_hv_base'] - merged_df['final_travel_time_optim']

    plot_gdf = network_gdf.merge(merged_df, left_on=EDGE_ID_COLUMN_IN_GEOJSON, right_on='uniqueid', how='left')

    max_toll = plot_gdf['toll_fee'].max()
    max_abs_reduction = plot_gdf['congestion_reduction_s'].abs().quantile(0.95)  # Use quantile for stable color range

    scenario_plot_dir = os.path.join(PLOTS_OUTPUT_DIR, "4_Spatial_Maps")
    os.makedirs(scenario_plot_dir, exist_ok=True)

    # Final Tolls Plot
    fig, ax = plt.subplots(1, 1, figsize=(15, 10))
    plot_gdf.plot(column='toll_fee', ax=ax, legend=True, cmap='viridis', vmin=0, vmax=max_toll if max_toll > 0 else 1)
    ax.set_title(f'Spatial Distribution of Final Tolls\n({scenario_name})', fontsize=16);
    ax.set_axis_off()
    output_filename = os.path.join(scenario_plot_dir, f'{scenario_name}_spatial_tolls.png')
    plt.savefig(output_filename, dpi=300, bbox_inches='tight');
    plt.close(fig)

    # Final Congestion Reduction Plot
    fig, ax = plt.subplots(1, 1, figsize=(15, 10))
    plot_gdf.plot(column='congestion_reduction_s', ax=ax, legend=True, cmap='RdYlGn', vmin=-max_abs_reduction,
                  vmax=max_abs_reduction)
    ax.set_title(f'Spatial Distribution of Final Congestion Reduction\n({scenario_name} vs. Baseline)', fontsize=16);
    ax.set_axis_off()
    output_filename = os.path.join(scenario_plot_dir, f'{scenario_name}_spatial_congestion.png')
    plt.savefig(output_filename, dpi=300, bbox_inches='tight');
    plt.close(fig)
    print(f"SUCCESS: Spatial maps for {scenario_name} saved.")


def main():
    """Main function to execute all analysis and plotting tasks."""
    print(f"{'=' * 60}\n=== STARTING SENSITIVITY ANALYSIS VISUALIZATION ===\n{'=' * 60}\n")

    os.makedirs(PLOTS_OUTPUT_DIR, exist_ok=True)

    summary_df, final_details_df = load_all_results(MAIN_RESULTS_DIR)

    if summary_df.empty or final_details_df.empty:
        print("Aborting analysis due to missing data.")
        return

    # Sequentially execute each analysis task
    analyze_equity_performance(final_details_df)
    analyze_sensitivity_to_penetration_rate(summary_df)
    generate_final_comparison_table(summary_df)

    # --- NEW: Loop to generate spatial plots for all scenarios ---
    """if GEOPANDAS_AVAILABLE and os.path.exists(NETWORK_GEOJSON_PATH):
        network_gdf = gpd.read_file(NETWORK_GEOJSON_PATH)
        baseline_df = final_details_df[final_details_df['scenario'] == BASELINE_NAME]

        scenario_folders = [f for f in glob.glob(os.path.join(MAIN_RESULTS_DIR, 'S*')) if os.path.isdir(f)]
        for folder in sorted(scenario_folders):
            plot_spatial_distribution(baseline_df, folder, network_gdf)
    else:
        print("\n--- Skipping Spatial Analysis ---")
        if not GEOPANDAS_AVAILABLE: print("Reason: Geopandas library not installed.")
        if not os.path.exists(NETWORK_GEOJSON_PATH): print(
            f"Reason: Network file not found at '{NETWORK_GEOJSON_PATH}'")"""

    print(f"\n{'=' * 60}\n=== ANALYSIS COMPLETE - All plots in '{PLOTS_OUTPUT_DIR}' ===\n{'=' * 60}")

    print(f"\n{'=' * 60}\n=== ANALYSIS COMPLETE - All plots saved in '{PLOTS_OUTPUT_DIR}' folder ===\n{'=' * 60}")


if __name__ == "__main__":
    main()

