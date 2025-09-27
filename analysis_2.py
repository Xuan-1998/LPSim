import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os
import geopandas as gpd  # Moved import here to handle potential absence

# --- 1. Configuration ---
BASELINE_RESULTS_DIR = 'baseline_results'
OPTIMIZED_RESULTS_DIR = 'results_More_Aggressive_Learning'
#OPTIMIZED_RESULTS_DIR = 'optimization_results'
NETWORK_GEOJSON_PATH = "network.geojson"
EDGE_ID_COLUMN_IN_GEOJSON = "uniqueid"

# --- NEW: Directory to store all analysis plots ---
PLOTS_OUTPUT_DIR = "analysis_plots"

# ==============================================================================
# --- 2. Plotting and Analysis Functions ---
# ==============================================================================

def plot_convergence(optimized_dir: str):
    """
    Generates separate convergence plots for objective function, revenue, and congestion.
    """
    print("--- (1/3) Generating Separate Convergence Plots ---")
    summary_file = os.path.join(optimized_dir, 'optimization_summary.csv')
    if not os.path.exists(summary_file):
        print(f"ERROR: Cannot find '{summary_file}'. Skipping convergence plots.")
        return

    os.makedirs(os.path.join(PLOTS_OUTPUT_DIR, optimized_dir), exist_ok=True)
    summary_df = pd.read_csv(summary_file)
    sns.set_theme(style="whitegrid")

    # --- Plot 1: Objective Function ---
    plt.figure(figsize=(10, 6))
    sns.lineplot(data=summary_df, x='iteration', y='objective', marker='s', linestyle='--')
    plt.title('Convergence of Objective Function', fontsize=16)
    plt.xlabel('Iteration', fontsize=14)
    plt.ylabel('Objective Function Value', fontsize=14)
    plt.grid(True)
    output_filename = os.path.join(PLOTS_OUTPUT_DIR, optimized_dir, 'convergence_objective.png')
    plt.savefig(output_filename, dpi=300)
    print(f"SUCCESS: Objective function convergence plot saved to '{output_filename}'")
    plt.close()

    # --- Plot 2: Total Revenue ---
    plt.figure(figsize=(10, 6))
    sns.lineplot(data=summary_df, x='iteration', y='revenue', color='g', marker='o')
    plt.title('Convergence of Total Revenue', fontsize=16)
    plt.xlabel('Iteration', fontsize=14)
    plt.ylabel('Total Revenue ($)', fontsize=14)
    plt.grid(True)
    output_filename = os.path.join(PLOTS_OUTPUT_DIR, optimized_dir, 'convergence_revenue.png')
    plt.savefig(output_filename, dpi=300)
    print(f"SUCCESS: Revenue convergence plot saved to '{output_filename}'")
    plt.close()

    # --- Plot 3: Total Congestion ---
    plt.figure(figsize=(10, 6))
    sns.lineplot(data=summary_df, x='iteration', y='congestion', color='r', marker='o')
    plt.title('Convergence of Total Congestion', fontsize=16)
    plt.xlabel('Iteration', fontsize=14)
    plt.ylabel('Total Congestion (vehicle-seconds)', fontsize=14)
    plt.grid(True)
    output_filename = os.path.join(PLOTS_OUTPUT_DIR, optimized_dir, 'convergence_congestion.png')
    plt.savefig(output_filename, dpi=300)
    print(f"SUCCESS: Congestion convergence plot saved to '{output_filename}'\n")
    plt.close()


def create_comparison_table(baseline_dir: str, optimized_dir: str):
    # ... (same as before) ...
    print("--- (2/3) Generating Performance Comparison Table ---")
    baseline_summary_file = os.path.join(baseline_dir, 'optimization_summary.csv')
    optimized_summary_file = os.path.join(optimized_dir, 'optimization_summary.csv')

    if not os.path.exists(baseline_summary_file) or not os.path.exists(optimized_summary_file):
        print("ERROR: Cannot find summary files in results directories. Skipping comparison table.")
        return

    baseline_series = pd.read_csv(baseline_summary_file).iloc[-1] # Using last row for baseline as well for consistency
    optimized_series = pd.read_csv(optimized_summary_file).iloc[-1]

    if baseline_series['revenue'] == 0:
        revenue_change = float('inf')
    else:
        revenue_change = ((optimized_series['revenue'] - baseline_series['revenue']) / baseline_series['revenue']) * 100
    congestion_change = ((optimized_series['congestion'] - baseline_series['congestion']) / baseline_series['congestion']) * 100

    comparison_data = {
        'Metric': ['Total Revenue ($)', 'Total Congestion (vehicle-seconds)'],
        'Baseline (No-Toll)': [f"{baseline_series['revenue']:,.2f}", f"{baseline_series['congestion']:,.0f}"],
        'Optimized Tolls': [f"{optimized_series['revenue']:,.2f}", f"{optimized_series['congestion']:,.0f}"],
        'Change (%)': [f"+{revenue_change:.2f}%" if revenue_change != float('inf') else "N/A (from zero)", f"{congestion_change:.2f}%"]
    }
    comparison_df = pd.DataFrame(comparison_data)

    print("SUCCESS: Performance Comparison Table:\n")
    print(comparison_df.to_string(index=False))

    output_filename = os.path.join(PLOTS_OUTPUT_DIR, 'performance_comparison.csv')
    comparison_df.to_csv(output_filename, index=False)
    print(f"\nTable saved to '{output_filename}' for easy import into your paper.\n")


def plot_spatial_distribution(baseline_dir: str, optimized_dir: str, geojson_path: str, edge_id_col: str):
    """
    Generates individual and combined grid maps for final tolls and congestion reduction.
    This version corrects the figure handling to prevent memory warnings.
    """
    print("--- (3/3) Generating Spatial Distribution Maps ---")
    try:
        import geopandas as gpd
    except ImportError:
        print("ERROR: Geopandas is not installed. Please run 'pip3 install geopandas'.")
        print("Skipping spatial distribution plot.")
        return
    if not os.path.exists(geojson_path):
        print(f"ERROR: Network GeoJSON file not found at '{geojson_path}'.")
        return

    # --- Step 1: Pre-load all iteration data (no changes here) ---
    print("\n--- Pre-loading all iteration data for consistent color scales ---")
    # ... (This entire data loading section remains unchanged) ...
    network_gdf = gpd.read_file(geojson_path)
    optimized_summary_df = pd.read_csv(os.path.join(optimized_dir, 'optimization_summary.csv'))
    num_iterations = optimized_summary_df['iteration'].max()
    baseline_details_file = os.path.join(baseline_dir, 'iteration_1_details.csv')
    baseline_df = pd.read_csv(baseline_details_file)
    iteration_gdfs = []
    for i in range(1, num_iterations + 1):
        details_file = os.path.join(optimized_dir, f'iteration_{i}_details.csv')
        if not os.path.exists(details_file):
            print(f"WARNING: Skipping iteration {i} due to missing file: {details_file}")
            continue
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
        print("ERROR: No iteration data found. Aborting spatial plot generation.")
        return
    global_toll_max = pd.concat([gdf['toll_fee'].dropna() for gdf in iteration_gdfs]).max()
    all_reductions = pd.concat([gdf['congestion_reduction_s'].dropna() for gdf in iteration_gdfs])
    max_abs_val = max(abs(all_reductions.quantile(0.05)), abs(all_reductions.quantile(0.95)))
    global_congestion_vmin, global_congestion_vmax = -max_abs_val, max_abs_val
    print(f"Global Toll Range: 0 to {global_toll_max:.2f}")
    print(f"Global Congestion Reduction Range: {global_congestion_vmin:.2f}s to {global_congestion_vmax:.2f}s")

    # --- Step 2: Generate and save individual plot for each iteration ---
    print("\n--- Generating individual spatial plots for each iteration ---")
    for i, plot_gdf in enumerate(iteration_gdfs):
        iteration_num = i + 1

        # FIX: Plot (a): Tolls - Correctly create and close the figure
        fig, ax = plt.subplots(figsize=(12, 8))  # Create figure and axis together
        plot_gdf.plot(column='toll_fee', ax=ax, legend=True, cmap='viridis', vmin=0, vmax=global_toll_max,
                      legend_kwds={'label': f"Toll Fee ($) - Iteration {iteration_num}", 'orientation': "horizontal"},
                      missing_kwds={'color': 'lightgrey', 'label': 'No Data'})
        ax.set_title(f'Spatial Distribution of Tolls - Iteration {iteration_num}', fontsize=16)
        ax.set_xticks([])
        ax.set_yticks([])
        output_filename = os.path.join(PLOTS_OUTPUT_DIR, optimized_dir, f'spatial_tolls_iteration_{iteration_num}.png')
        plt.savefig(output_filename, dpi=300, bbox_inches='tight')
        plt.close(fig)  # Explicitly close the figure
        print(f"  SUCCESS: Toll distribution map (iteration {iteration_num}) saved.")

        # FIX: Plot (b): Congestion - Correctly create and close the figure
        fig, ax = plt.subplots(figsize=(12, 8))  # Create figure and axis together
        plot_gdf.plot(column='congestion_reduction_s', ax=ax, legend=True, cmap='RdYlGn', vmin=global_congestion_vmin,
                      vmax=global_congestion_vmax,
                      legend_kwds={'label': f"Travel Time Reduction (s) - Iteration {iteration_num}",
                                   'orientation': "horizontal"},
                      missing_kwds={'color': 'lightgrey', 'label': 'No Data'})
        ax.set_title(f'Spatial Distribution of Congestion Reduction - Iteration {iteration_num}', fontsize=16)
        ax.set_xticks([])
        ax.set_yticks([])
        output_filename = os.path.join(PLOTS_OUTPUT_DIR, optimized_dir,
                                       f'spatial_congestion_reduction_iteration_{iteration_num}.png')
        plt.savefig(output_filename, dpi=300, bbox_inches='tight')
        plt.close(fig)  # Explicitly close the figure
        print(f"  SUCCESS: Congestion reduction map (iteration {iteration_num}) saved.")

    # --- Step 3: Generate and save combined grid plots (no changes here) ---
    print("\n--- Generating combined grid plots for all iterations ---")
    # ... (This entire grid plot section remains unchanged and is already correct) ...
    n_rows, n_cols = 4, 5
    fig_tolls, axes_tolls = plt.subplots(n_rows, n_cols, figsize=(25, 20), sharex=True, sharey=True)
    axes_tolls = axes_tolls.flatten()
    fig_tolls.suptitle('Evolution of Tolls Across Iterations', fontsize=28, fontweight='bold')
    for i, ax in enumerate(axes_tolls):
        if i < len(iteration_gdfs):
            plot_gdf = iteration_gdfs[i]
            plot_gdf.plot(column='toll_fee', cmap='viridis', vmin=0, vmax=global_toll_max, ax=ax, legend=False)
            ax.text(0.05, 0.9, f'Iter: {i + 1}', transform=ax.transAxes, fontsize=14, fontweight='bold', color='white')
        ax.set_xticks([]);
        ax.set_yticks([]);
        ax.set_facecolor('lightgray')
    sm_toll = plt.cm.ScalarMappable(cmap='viridis', norm=plt.Normalize(vmin=0, vmax=global_toll_max))
    cbar_toll = fig_tolls.colorbar(sm_toll, orientation='vertical', shrink=0.7, pad=0.02)
    cbar_toll.set_label('Toll Fee ($)', fontsize=20)
    fig_tolls.tight_layout(rect=[0, 0, 1, 0.96])
    grid_output_path = os.path.join(PLOTS_OUTPUT_DIR, optimized_dir, 'GRID_spatial_tolls_all_iterations.png')
    plt.savefig(grid_output_path, dpi=300)
    plt.close(fig_tolls)
    print(f"SUCCESS: Combined grid plot for tolls saved to '{grid_output_path}'")
    fig_cong, axes_cong = plt.subplots(n_rows, n_cols, figsize=(25, 20), sharex=True, sharey=True)
    axes_cong = axes_cong.flatten()
    fig_cong.suptitle('Evolution of Congestion Reduction Across Iterations', fontsize=28, fontweight='bold')
    for i, ax in enumerate(axes_cong):
        if i < len(iteration_gdfs):
            plot_gdf = iteration_gdfs[i]
            plot_gdf.plot(column='congestion_reduction_s', cmap='RdYlGn', vmin=global_congestion_vmin,
                          vmax=global_congestion_vmax, ax=ax, legend=False)
            ax.text(0.05, 0.9, f'Iter: {i + 1}', transform=ax.transAxes, fontsize=14, fontweight='bold', color='black')
        ax.set_xticks([]);
        ax.set_yticks([]);
        ax.set_facecolor('lightgray')
    sm_cong = plt.cm.ScalarMappable(cmap='RdYlGn',
                                    norm=plt.Normalize(vmin=global_congestion_vmin, vmax=global_congestion_vmax))
    cbar_cong = fig_cong.colorbar(sm_cong, orientation='vertical', shrink=0.7, pad=0.02)
    cbar_cong.set_label('Travel Time Reduction (seconds)', fontsize=20)
    fig_cong.tight_layout(rect=[0, 0, 1, 0.96])
    grid_output_path = os.path.join(PLOTS_OUTPUT_DIR, optimized_dir, 'GRID_spatial_congestion_all_iterations.png')
    plt.savefig(grid_output_path, dpi=300)
    plt.close(fig_cong)
    print(f"SUCCESS: Combined grid plot for congestion saved to '{grid_output_path}'")

    # --- Step 4: Generate final summary spatial plots ---
    print("\n--- Generating final summary spatial plots ---")
    final_gdf = iteration_gdfs[-1]

    # FIX: Plot (a): Final Tolls - Correctly create and close the figure
    fig, ax = plt.subplots(figsize=(12, 8))  # Create figure and axis together
    final_gdf.plot(column='toll_fee', ax=ax, legend=True, cmap='viridis', vmin=0, vmax=global_toll_max,
                   legend_kwds={'label': "Final Toll Fee ($)", 'orientation': "horizontal"},
                   missing_kwds={'color': 'lightgrey', 'label': 'No Data'})
    ax.set_title('Spatial Distribution of Final Tolls', fontsize=16)
    ax.set_xticks([])
    ax.set_yticks([])
    output_filename = os.path.join(PLOTS_OUTPUT_DIR, optimized_dir, 'spatial_final_tolls.png')
    plt.savefig(output_filename, dpi=300, bbox_inches='tight')
    plt.close(fig)  # Explicitly close the figure
    print(f"SUCCESS: Final toll distribution map saved to '{output_filename}'")

    # FIX: Plot (b): Final Congestion - Correctly create and close the figure
    fig, ax = plt.subplots(figsize=(12, 8))  # Create figure and axis together
    final_gdf.plot(column='congestion_reduction_s', ax=ax, legend=True, cmap='RdYlGn', vmin=global_congestion_vmin,
                   vmax=global_congestion_vmax,
                   legend_kwds={'label': "Final Travel Time Reduction (s)", 'orientation': "horizontal"},
                   missing_kwds={'color': 'lightgrey', 'label': 'No Data'})
    ax.set_title('Spatial Distribution of Final Congestion Reduction', fontsize=16)
    ax.set_xticks([])
    ax.set_yticks([])
    output_filename = os.path.join(PLOTS_OUTPUT_DIR, optimized_dir, 'spatial_final_congestion_reduction.png')
    plt.savefig(output_filename, dpi=300, bbox_inches='tight')
    plt.close(fig)  # Explicitly close the figure
    print(f"SUCCESS: Final congestion reduction map saved to '{output_filename}'")


# ==============================================================================
# --- 3. Main Execution ---
# ==============================================================================

def main():
    """Main function to execute all analysis and plotting tasks in sequence."""
    print("=========================================")
    print("=== STARTING NUMERICAL RESULT ANALYSIS ===")
    print("=========================================\n")

    # --- NEW: Create the plots output directory if it doesn't exist ---
    os.makedirs(PLOTS_OUTPUT_DIR, exist_ok=True)

    # 1. Generate convergence plots (separately)
    plot_convergence(OPTIMIZED_RESULTS_DIR)

    # 2. Generate performance comparison table
    create_comparison_table(BASELINE_RESULTS_DIR, OPTIMIZED_RESULTS_DIR)

    # 3. Generate spatial distribution maps (final and per-iteration)
    plot_spatial_distribution(
        BASELINE_RESULTS_DIR,
        OPTIMIZED_RESULTS_DIR,
        NETWORK_GEOJSON_PATH,
        EDGE_ID_COLUMN_IN_GEOJSON
    )

    print("\n=========================================")
    print("=== ANALYSIS COMPLETE ===")
    print("=========================================")


if __name__ == "__main__":
    main()