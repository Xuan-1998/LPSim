import pandas as pd

import matplotlib.pyplot as plt

import seaborn as sns

import os

import glob

import imageio.v2 as imageio

# Attempt to import geopandas

try:

    import geopandas as gpd

    GEOPANDAS_AVAILABLE = True

except ImportError:

    GEOPANDAS_AVAILABLE = False

"""

visualize_evolution.py



This script focuses on visualizing the dynamic, iterative process of a single

optimization run. It generates a series of spatial maps for each iteration

and compiles them into animated GIFs, showing the evolution of tolls, congestion,

and traffic flow over time.

"""

# ==============================================================================

# --- 1. Configuration: CHOOSE WHICH EXPERIMENT TO VISUALIZE ---

# ==============================================================================


# !!! CHOOSE THE EXPERIMENT FOLDER YOU WANT TO ANIMATE !!!

# Example: 'S2_PartialCollab_eta_0.5' or 'S1_Competition_eta_0.7'

EXPERIMENT_TO_VISUALIZE = 'S2_PartialCollab_eta_0.5'

# --- General Paths (should match your other scripts) ---

MAIN_RESULTS_DIR = 'sensitivity_analysis_results'

BASELINE_NAME = 'Baseline'

PLOTS_OUTPUT_DIR = "evolution_visualizations"  # A new, dedicated folder for animations

NETWORK_GEOJSON_PATH = "network.geojson"

EDGE_ID_COLUMN_IN_GEOJSON = "uniqueid"

# --- Animation Settings ---

CREATE_ANIMATION = True  # Set to False if you only want the individual map images

ANIMATION_FPS = 3  # Frames per second for the GIF


# ==============================================================================

# --- 2. Main Visualization Logic ---

# ==============================================================================


def create_evolution_visuals():
    """

    Generates and optionally animates the spatial evolution of a single experiment.

    """

    print(f"{'=' * 60}\n=== STARTING DYNAMIC EVOLUTION VISUALIZATION ===\n{'=' * 60}\n")

    # --- Setup and Pre-flight Checks ---

    if not GEOPANDAS_AVAILABLE:
        print("ERROR: Geopandas is not installed. This script requires it.")

        print("Please run: pip install geopandas imageio imageio-ffmpeg")

        return

    experiment_dir = os.path.join(MAIN_RESULTS_DIR, EXPERIMENT_TO_VISUALIZE)

    baseline_dir = os.path.join(MAIN_RESULTS_DIR, BASELINE_NAME)

    if not os.path.isdir(experiment_dir):
        print(f"FATAL ERROR: The specified experiment directory was not found:\n{experiment_dir}")

        return

    os.makedirs(PLOTS_OUTPUT_DIR, exist_ok=True)

    print(f"Target Experiment: '{EXPERIMENT_TO_VISUALIZE}'")

    print(f"Output will be saved to: '{PLOTS_OUTPUT_DIR}'")

    # --- Load Geospatial and Baseline Data ---

    network_gdf = gpd.read_file(NETWORK_GEOJSON_PATH)

    baseline_details_file = os.path.join(baseline_dir, 'iteration_1_details.csv')

    if not os.path.exists(baseline_details_file):
        print(f"ERROR: Baseline data not found at '{baseline_details_file}'. Cannot calculate congestion reduction.")

        return

    baseline_df = pd.read_csv(baseline_details_file)

    baseline_df['baseline_travel_time'] = (baseline_df['final_travel_time_hv'] + baseline_df[
        'final_travel_time_av']) / 2

    # --- Load all iteration data for the chosen experiment ---

    print("\n--- (1/3) Loading all iteration data... ---")

    iteration_files = sorted(glob.glob(os.path.join(experiment_dir, 'iteration_*.csv')))

    if not iteration_files:
        print("ERROR: No iteration detail files found in the specified directory.")

        return

    all_gdfs = []

    for f in iteration_files:
        iter_df = pd.read_csv(f)

        merged_df = pd.merge(iter_df, baseline_df[['uniqueid', 'baseline_travel_time']], on='uniqueid')

        merged_df['congestion_reduction_s'] = merged_df['baseline_travel_time'] - (
                    merged_df['final_travel_time_hv'] + merged_df['final_travel_time_av']) / 2

        plot_gdf = network_gdf.merge(merged_df, left_on=EDGE_ID_COLUMN_IN_GEOJSON, right_on='uniqueid')

        all_gdfs.append(plot_gdf)

    print(f"Loaded data for {len(all_gdfs)} iterations.")

    # --- Establish consistent color scales across all frames ---

    global_toll_max = pd.concat([gdf['toll_fee'].dropna() for gdf in all_gdfs]).max()

    all_reductions = pd.concat([gdf['congestion_reduction_s'].dropna() for gdf in all_gdfs])

    max_abs_val = max(abs(all_reductions.quantile(0.05)), abs(all_reductions.quantile(0.95)))

    global_congestion_vmin, global_congestion_vmax = -max_abs_val, max_abs_val

    global_flow_max = pd.concat([(gdf['final_flow_hv'] + gdf['final_flow_av']).dropna() for gdf in all_gdfs]).quantile(
        0.98)

    # --- Generate individual frames for each metric ---

    print("\n--- (2/3) Generating individual map frames for each iteration... ---")

    frame_paths = {'tolls': [], 'congestion': [], 'flow_av': [], 'flow_hv': []}

    for i, gdf in enumerate(all_gdfs):
        iteration_num = i + 1

        print(f"  - Processing frame {iteration_num}/{len(all_gdfs)}")

        # Plot Tolls

        fig, ax = plt.subplots(1, 1, figsize=(12, 8))

        ax.set_facecolor('white')  # <--- FIX: Set consistent background color

        gdf.plot(column='toll_fee', ax=ax, legend=True, cmap='viridis', vmin=0, vmax=global_toll_max)

        ax.set_title(f'Toll Fees - Iteration {iteration_num}', fontsize=16)

        ax.set_axis_off()

        path = os.path.join(PLOTS_OUTPUT_DIR, f'frame_toll_{iteration_num:03d}.png')

        plt.savefig(path, dpi=150, bbox_inches='tight')

        plt.close(fig)

        frame_paths['tolls'].append(path)

        # Plot Congestion Reduction

        fig, ax = plt.subplots(1, 1, figsize=(12, 8))

        ax.set_facecolor('#f0f0f0')  # <--- FIX: Set consistent background color

        gdf.plot(column='congestion_reduction_s', ax=ax, legend=True, cmap='RdYlGn', vmin=global_congestion_vmin,
                 vmax=global_congestion_vmax)

        ax.set_title(f'Congestion Reduction (vs Baseline) - Iteration {iteration_num}', fontsize=16)

        ax.set_axis_off()

        path = os.path.join(PLOTS_OUTPUT_DIR, f'frame_congestion_{iteration_num:03d}.png')

        plt.savefig(path, dpi=150, bbox_inches='tight')

        plt.close(fig)

        frame_paths['congestion'].append(path)

        # Plot AV Flow

        fig, ax = plt.subplots(1, 1, figsize=(12, 8))

        ax.set_facecolor('#f0f0f0')  # <--- FIX: Set consistent background color

        gdf.plot(column='final_flow_av', ax=ax, legend=True, cmap='Blues', vmin=0, vmax=global_flow_max)

        ax.set_title(f'AV Flow - Iteration {iteration_num}', fontsize=16)

        ax.set_axis_off()

        path = os.path.join(PLOTS_OUTPUT_DIR, f'frame_flow_av_{iteration_num:03d}.png')

        plt.savefig(path, dpi=150, bbox_inches='tight')

        plt.close(fig)

        frame_paths['flow_av'].append(path)

        # Plot HV Flow

        fig, ax = plt.subplots(1, 1, figsize=(12, 8))

        ax.set_facecolor('#f0f0f0')  # <--- FIX: Set consistent background color

        gdf.plot(column='final_flow_hv', ax=ax, legend=True, cmap='Oranges', vmin=0, vmax=global_flow_max)

        ax.set_title(f'HV Flow - Iteration {iteration_num}', fontsize=16)

        ax.set_axis_off()

        path = os.path.join(PLOTS_OUTPUT_DIR, f'frame_flow_hv_{iteration_num:03d}.png')

        plt.savefig(path, dpi=150, bbox_inches='tight')

        plt.close(fig)

        frame_paths['flow_hv'].append(path)

    # --- Compile animations ---

    if CREATE_ANIMATION:

        print("\n--- (3/3) Compiling frames into animated GIFs... ---")

        for metric, paths in frame_paths.items():

            output_gif_path = os.path.join(PLOTS_OUTPUT_DIR, f'evolution_{metric}.gif')

            with imageio.get_writer(output_gif_path, mode='I', fps=ANIMATION_FPS) as writer:

                for filename in paths:
                    image = imageio.imread(filename)

                    writer.append_data(image)

            print(f"  SUCCESS: Animation saved to '{output_gif_path}'")

            # Optional: Clean up individual frame images

            for filename in paths:
                os.remove(filename)

    print(f"\n{'=' * 60}\n=== EVOLUTION VISUALIZATION COMPLETE ===\n{'=' * 60}")


if __name__ == "__main__":
    # Before running, make sure you have the required libraries:

    # pip install geopandas imageio imageio-ffmpeg

    create_evolution_visuals()