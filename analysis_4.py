import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os
# import geopandas as gpd

print("--- Script started. Loading libraries, please wait... ---")

# --- 1. Configuration ---
BASELINE_RESULTS_DIR = 'baseline_results'
PLOTS_OUTPUT_DIR = "analysis_plots"
NETWORK_GEOJSON_PATH = "network.geojson"
EDGE_ID_COLUMN_IN_GEOJSON = "uniqueid"

SCENARIOS_TO_ANALYZE = [
    "Best_Params",
]

## NEW ## - Choose the spatial plotting mode
# 'final_only': Generates only the final summary maps for tolls and congestion. (Faster)
# 'full': Generates maps for every iteration plus the combined grid plots. (Slower, more detailed)
SPATIAL_PLOT_MODE = 'final_only'


# ==============================================================================
# --- 2. Plotting and Analysis Functions ---
# ==============================================================================

def plot_convergence(optimized_dir: str):
    """
    Generates separate convergence plots for objective function, revenue, and congestion.
    """
    print(f"--- (1/3) Generating Convergence Plots for '{optimized_dir}' ---")
    summary_file = os.path.join(optimized_dir, 'optimization_summary.csv')
    if not os.path.exists(summary_file):
        print(f"  ERROR: Cannot find '{summary_file}'. Skipping.")
        return

    scenario_plot_dir = os.path.join(PLOTS_OUTPUT_DIR, optimized_dir)
    os.makedirs(scenario_plot_dir, exist_ok=True)
    summary_df = pd.read_csv(summary_file)
    sns.set_theme(style="whitegrid")

    # --- Plot 1: Objective Function ---
    plt.figure(figsize=(10, 6))
    sns.lineplot(data=summary_df, x='iteration', y='objective', marker='s', linestyle='--')
    plt.title(f'Convergence of Objective Function\n({optimized_dir})', fontsize=16)
    plt.xlabel('Iteration', fontsize=14)
    plt.ylabel('Objective Function Value', fontsize=14)
    output_filename = os.path.join(scenario_plot_dir, 'convergence_objective.png')
    plt.savefig(output_filename, dpi=300)
    plt.close()

    # --- Plot 2: Total Revenue ---
    plt.figure(figsize=(10, 6))
    sns.lineplot(data=summary_df, x='iteration', y='revenue', color='g', marker='o')
    plt.title(f'Convergence of Total Revenue\n({optimized_dir})', fontsize=16)
    plt.xlabel('Iteration', fontsize=14)
    plt.ylabel('Total Revenue ($)', fontsize=14)
    output_filename = os.path.join(scenario_plot_dir, 'convergence_revenue.png')
    plt.savefig(output_filename, dpi=300)
    plt.close()

    # --- Plot 3: Total Congestion ---
    plt.figure(figsize=(10, 6))
    sns.lineplot(data=summary_df, x='iteration', y='congestion', color='r', marker='o')
    plt.title(f'Convergence of Total Congestion\n({optimized_dir})', fontsize=16)
    plt.xlabel('Iteration', fontsize=14)
    plt.ylabel('Total Congestion (vehicle-seconds)', fontsize=14)
    output_filename = os.path.join(scenario_plot_dir, 'convergence_congestion.png')
    plt.savefig(output_filename, dpi=300)
    plt.close()
    print(f"  SUCCESS: Convergence plots saved to '{scenario_plot_dir}'\n")


def create_comparison_table(baseline_dir: str, optimized_dir: str):
    """
    Generates a comparison table for a single scenario vs. baseline.
    """
    print(f"--- (2/3) Generating Performance Comparison for '{optimized_dir}' ---")
    baseline_summary_file = os.path.join(baseline_dir, 'optimization_summary.csv')
    optimized_summary_file = os.path.join(optimized_dir, 'optimization_summary.csv')

    if not (os.path.exists(baseline_summary_file) and os.path.exists(optimized_summary_file)):
        print("  ERROR: Cannot find summary files. Skipping.")
        return

    baseline_series = pd.read_csv(baseline_summary_file).iloc[-1]
    optimized_series = pd.read_csv(optimized_summary_file).iloc[-1]

    if baseline_series['revenue'] == 0:
        revenue_change = float('inf')
    else:
        revenue_change = ((optimized_series['revenue'] - baseline_series['revenue']) / baseline_series['revenue']) * 100
    congestion_change = ((optimized_series['congestion'] - baseline_series['congestion']) / baseline_series[
        'congestion']) * 100

    comparison_data = {
        'Metric': ['Total Revenue ($)', 'Total Congestion (vehicle-seconds)'],
        'Baseline (No-Toll)': [f"{baseline_series['revenue']:,.2f}", f"{baseline_series['congestion']:,.0f}"],
        'Optimized Tolls': [f"{optimized_series['revenue']:,.2f}", f"{optimized_series['congestion']:,.0f}"],
        'Change (%)': [f"+{revenue_change:.2f}%" if revenue_change != float('inf') else "N/A (from zero)",
                       f"{congestion_change:.2f}%"]
    }
    comparison_df = pd.DataFrame(comparison_data)

    print("  SUCCESS: Performance Comparison Table:\n")
    print(comparison_df.to_string(index=False))
    print("\n")


def plot_spatial_distribution(baseline_dir: str, optimized_dir: str, geojson_path: str, edge_id_col: str,
                              mode: str = 'full'):
    """
    Generates spatial distribution maps based on the selected mode.
    mode='full': Creates maps for every iteration and combined grids.
    mode='final_only': Creates only the final summary maps.
    """
    print(f"--- (3/3) Generating Spatial Maps for '{optimized_dir}' (Mode: {mode}) ---")
    try:
        import geopandas as gpd
    except ImportError:
        print("  ERROR: Geopandas is not installed. Skipping spatial plots.")
        return
    if not os.path.exists(geojson_path):
        print(f"  ERROR: Network GeoJSON file not found at '{geojson_path}'.")
        return

    # --- Step 1: Load data ---
    network_gdf = gpd.read_file(geojson_path)
    summary_file = os.path.join(optimized_dir, 'optimization_summary.csv')
    baseline_details_file = os.path.join(baseline_dir, 'iteration_1_details.csv')

    if not (os.path.exists(summary_file) and os.path.exists(baseline_details_file)):
        print("  ERROR: Missing summary or baseline files. Skipping.")
        return

    optimized_summary_df = pd.read_csv(summary_file)
    num_iterations = optimized_summary_df['iteration'].max()
    baseline_df = pd.read_csv(baseline_details_file)

    ## NEW ##: Load all iteration data only if needed for color scaling or full mode
    iteration_gdfs = []
    if mode == 'full':
        iterations_to_load = range(1, num_iterations + 1)
    else:  # final_only mode
        iterations_to_load = [num_iterations]  # Only load the last one

    for i in iterations_to_load:
        details_file = os.path.join(optimized_dir, f'iteration_{i}_details.csv')
        if not os.path.exists(details_file): continue
        iter_df = pd.read_csv(details_file)
        merged_df = pd.merge(
            iter_df[['uniqueid', 'final_travel_time', 'toll_fee']],
            baseline_df[['uniqueid', 'final_travel_time']],
            on='uniqueid', suffixes=('_final', '_baseline'), how='left'
        )
        merged_df['congestion_reduction_s'] = merged_df['final_travel_time_baseline'].fillna(0) - merged_df[
            'final_travel_time_final'].fillna(0)
        plot_gdf = network_gdf.merge(merged_df, left_on=edge_id_col, right_on='uniqueid', how='left')
        iteration_gdfs.append(plot_gdf)

    if not iteration_gdfs:
        print("  ERROR: No iteration data found. Aborting spatial plots.")
        return

    # Establish global color scales using all loaded data
    global_toll_max = pd.concat([gdf['toll_fee'].dropna() for gdf in iteration_gdfs]).max()
    all_reductions = pd.concat([gdf['congestion_reduction_s'].dropna() for gdf in iteration_gdfs])
    max_abs_val = max(abs(all_reductions.quantile(0.05)), abs(all_reductions.quantile(0.95)))
    global_congestion_vmin, global_congestion_vmax = -max_abs_val, max_abs_val

    scenario_plot_dir = os.path.join(PLOTS_OUTPUT_DIR, optimized_dir)
    os.makedirs(scenario_plot_dir, exist_ok=True)

    ## NEW ##: Conditional plotting based on mode
    if mode == 'full':
        print("\n  --- Generating individual spatial plots for each iteration ---")
        for i, plot_gdf in enumerate(iteration_gdfs):
            iteration_num = iterations_to_load[i]
            # Plot Tolls for iteration
            fig, ax = plt.subplots(figsize=(12, 8))
            plot_gdf.plot(column='toll_fee', ax=ax, legend=True, cmap='viridis', vmin=0, vmax=global_toll_max,
                          legend_kwds={'label': f"Toll Fee ($) - Iteration {iteration_num}",
                                       'orientation': "horizontal"})
            ax.set_title(f'Spatial Distribution of Tolls - Iteration {iteration_num}', fontsize=16)
            output_filename = os.path.join(scenario_plot_dir, f'spatial_tolls_iteration_{iteration_num}.png')
            plt.savefig(output_filename, dpi=300, bbox_inches='tight')
            plt.close(fig)

            # Plot Congestion for iteration
            fig, ax = plt.subplots(figsize=(12, 8))
            plot_gdf.plot(column='congestion_reduction_s', ax=ax, legend=True, cmap='RdYlGn',
                          vmin=global_congestion_vmin, vmax=global_congestion_vmax,
                          legend_kwds={'label': f"Travel Time Reduction (s) - Iteration {iteration_num}",
                                       'orientation': "horizontal"})
            ax.set_title(f'Spatial Distribution of Congestion Reduction - Iteration {iteration_num}', fontsize=16)
            output_filename = os.path.join(scenario_plot_dir,
                                           f'spatial_congestion_reduction_iteration_{iteration_num}.png')
            plt.savefig(output_filename, dpi=300, bbox_inches='tight')
            plt.close(fig)
        print("    SUCCESS: Per-iteration maps saved.")

        print("\n  --- Generating combined grid plots ---")
        n_rows, n_cols = 4, 5
        # Grid plot for Tolls
        fig_tolls, axes_tolls = plt.subplots(n_rows, n_cols, figsize=(25, 20), sharex=True, sharey=True)
        axes_tolls = axes_tolls.flatten()
        fig_tolls.suptitle('Evolution of Tolls Across Iterations', fontsize=28)
        for i, ax in enumerate(axes_tolls):
            if i < len(iteration_gdfs):
                plot_gdf = iteration_gdfs[i]
                plot_gdf.plot(column='toll_fee', cmap='viridis', vmin=0, vmax=global_toll_max, ax=ax, legend=False)
                ax.text(0.05, 0.9, f'Iter: {iterations_to_load[i]}', transform=ax.transAxes, fontsize=14, color='white')
            ax.set_xticks([]);
            ax.set_yticks([]);
            ax.set_facecolor('lightgray')
        sm_toll = plt.cm.ScalarMappable(cmap='viridis', norm=plt.Normalize(vmin=0, vmax=global_toll_max))
        cbar_toll = fig_tolls.colorbar(sm_toll, ax=axes_tolls, orientation='vertical', shrink=0.7)
        cbar_toll.set_label('Toll Fee ($)', fontsize=20)
        grid_output_path = os.path.join(scenario_plot_dir, 'GRID_spatial_tolls_all_iterations.png')
        plt.savefig(grid_output_path, dpi=300)
        plt.close(fig_tolls)

        # Grid plot for Congestion
        fig_cong, axes_cong = plt.subplots(n_rows, n_cols, figsize=(25, 20), sharex=True, sharey=True)
        axes_cong = axes_cong.flatten()
        fig_cong.suptitle('Evolution of Congestion Reduction Across Iterations', fontsize=28)
        for i, ax in enumerate(axes_cong):
            if i < len(iteration_gdfs):
                plot_gdf = iteration_gdfs[i]
                plot_gdf.plot(column='congestion_reduction_s', cmap='RdYlGn', vmin=global_congestion_vmin,
                              vmax=global_congestion_vmax, ax=ax, legend=False)
                ax.text(0.05, 0.9, f'Iter: {iterations_to_load[i]}', transform=ax.transAxes, fontsize=14)
            ax.set_xticks([]);
            ax.set_yticks([]);
            ax.set_facecolor('lightgray')
        sm_cong = plt.cm.ScalarMappable(cmap='RdYlGn',
                                        norm=plt.Normalize(vmin=global_congestion_vmin, vmax=global_congestion_vmax))
        cbar_cong = fig_cong.colorbar(sm_cong, ax=axes_cong, orientation='vertical', shrink=0.7)
        cbar_cong.set_label('Travel Time Reduction (s)', fontsize=20)
        grid_output_path = os.path.join(scenario_plot_dir, 'GRID_spatial_congestion_all_iterations.png')
        plt.savefig(grid_output_path, dpi=300)
        plt.close(fig_cong)
        print("    SUCCESS: Grid plots saved.")

    # --- Generate final summary plots (always) ---
    print("\n  --- Generating final summary spatial plots ---")
    final_gdf = iteration_gdfs[-1]

    # Final Tolls Plot
    fig, ax = plt.subplots(figsize=(12, 8))
    final_gdf.plot(column='toll_fee', ax=ax, legend=True, cmap='viridis', vmin=0, vmax=global_toll_max,
                   legend_kwds={'label': "Final Toll Fee ($)", 'orientation': "horizontal"})
    ax.set_title(f'Spatial Distribution of Final Tolls\n({optimized_dir})', fontsize=16)
    output_filename = os.path.join(scenario_plot_dir, 'spatial_final_tolls.png')
    plt.savefig(output_filename, dpi=300, bbox_inches='tight')
    plt.close(fig)

    # Final Congestion Reduction Plot
    fig, ax = plt.subplots(figsize=(12, 8))
    final_gdf.plot(column='congestion_reduction_s', ax=ax, legend=True, cmap='RdYlGn', vmin=global_congestion_vmin,
                   vmax=global_congestion_vmax,
                   legend_kwds={'label': "Final Travel Time Reduction (s)", 'orientation': "horizontal"})
    ax.set_title(f'Spatial Distribution of Final Congestion Reduction\n({optimized_dir})', fontsize=16)
    output_filename = os.path.join(scenario_plot_dir, 'spatial_final_congestion_reduction.png')
    plt.savefig(output_filename, dpi=300, bbox_inches='tight')
    plt.close(fig)

    print(f"  SUCCESS: Final spatial maps saved to '{scenario_plot_dir}'")


def create_master_comparison_table(baseline_dir: str, scenario_names: list):
    """
    Generates a single comparison table summarizing all scenarios.
    """
    print("\n--- Generating Master Performance Comparison Table ---")
    baseline_summary_file = os.path.join(baseline_dir, 'optimization_summary.csv')
    if not os.path.exists(baseline_summary_file):
        print(f"ERROR: Baseline summary file not found. Aborting.")
        return

    baseline_series = pd.read_csv(baseline_summary_file).iloc[-1]
    all_results = [{'Scenario': 'Baseline (No-Toll)', 'Total Revenue ($)': baseline_series['revenue'],
                    'Total Congestion (veh-sec)': baseline_series['congestion'], 'Revenue Change (%)': '---',
                    'Congestion Change (%)': '---'}]

    for name in scenario_names:
        optimized_dir = f"results_{name}"
        optimized_summary_file = os.path.join(optimized_dir, 'optimization_summary.csv')
        if not os.path.exists(optimized_summary_file):
            print(f"WARNING: Skipping '{name}', summary file not found.")
            continue
        optimized_series = pd.read_csv(optimized_summary_file).iloc[-1]
        revenue_change = ((optimized_series['revenue'] - baseline_series['revenue']) / baseline_series['revenue']) * 100
        congestion_change = ((optimized_series['congestion'] - baseline_series['congestion']) / baseline_series[
            'congestion']) * 100
        all_results.append({'Scenario': name, 'Total Revenue ($)': optimized_series['revenue'],
                            'Total Congestion (veh-sec)': optimized_series['congestion'],
                            'Revenue Change (%)': f"+{revenue_change:.2f}%",
                            'Congestion Change (%)': f"{congestion_change:.2f}%"})

    if len(all_results) <= 1:
        print("No optimized scenario data found to compare.")
        return

    comparison_df = pd.DataFrame(all_results)
    comparison_df['Total Revenue ($)'] = comparison_df['Total Revenue ($)'].apply(
        lambda x: f"{x:,.2f}" if isinstance(x, (int, float)) else x)
    comparison_df['Total Congestion (veh-sec)'] = comparison_df['Total Congestion (veh-sec)'].apply(
        lambda x: f"{x:,.0f}" if isinstance(x, (int, float)) else x)

    print("\nSUCCESS: Master Performance Comparison Table:\n")
    print(comparison_df.to_string(index=False))
    output_filename = os.path.join(PLOTS_OUTPUT_DIR, 'MASTER_performance_comparison.csv')
    comparison_df.to_csv(output_filename, index=False)
    print(f"\nTable saved to '{output_filename}'")


# ==============================================================================
# --- 3. Main Execution ---
# ==============================================================================

def main():
    """Main function to execute all analysis and plotting tasks."""
    print("=========================================")
    print("=== STARTING BATCH RESULT ANALYSIS ===")
    print(f"=== Spatial Plot Mode: {SPATIAL_PLOT_MODE.upper()} ===")
    print("=========================================\n")

    os.makedirs(PLOTS_OUTPUT_DIR, exist_ok=True)

    for scenario_name in SCENARIOS_TO_ANALYZE:
        optimized_results_dir = f"results_{scenario_name}"
        print(f"==========================================================")
        print(f"  PROCESSING SCENARIO: {scenario_name}")
        print(f"==========================================================\n")

        if not os.path.isdir(optimized_results_dir):
            print(f"WARNING: Directory '{optimized_results_dir}' not found. Skipping.")
            continue

        plot_convergence(optimized_results_dir)
        create_comparison_table(BASELINE_RESULTS_DIR, optimized_results_dir)
        plot_spatial_distribution(
            BASELINE_RESULTS_DIR,
            optimized_results_dir,
            NETWORK_GEOJSON_PATH,
            EDGE_ID_COLUMN_IN_GEOJSON,
            mode=SPATIAL_PLOT_MODE  ## NEW ## - Pass the mode to the function
        )

    create_master_comparison_table(BASELINE_RESULTS_DIR, SCENARIOS_TO_ANALYZE)
    print("\n=========================================")
    print("=== BATCH ANALYSIS COMPLETE ===")
    print("=========================================")


if __name__ == "__main__":
    main()