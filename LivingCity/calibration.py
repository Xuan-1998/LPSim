import sys
import os
import time
import math
import pandas as pd
import numpy as np
import time
import json
import subprocess
import matplotlib.pyplot as plt
from shapely.geometry import LineString
pd.options.display.max_colwidth = 100
from datetime import datetime
import re
from termcolor import colored
from mpl_toolkits.axes_grid.anchored_artists import AnchoredText
from pdb import set_trace as st

def log(text):
    print(colored(text, 'cyan'))

def load_network(nodes_file, edges_file, path):
    nodes = pd.read_csv("{}/{}".format(path, nodes_file))
    edges = pd.read_csv("{}/{}".format(path, edges_file))
    return nodes, edges

def merge_edges_with_uber_edges(edges_df, uber_df):
    return pd.merge(edges_df, uber_df,  how='inner', left_on=['osmid_u','osmid_v'], right_on = ['osm_start_node_id','osm_end_node_id'])

def merge_edges_with_times_with_uber_edges(edge_vels_df, same_edges_df):
    return pd.merge(edge_vels_df, same_edges_df,  how='inner', on=['osmid_u','osmid_v'])

def write_options_file(params):
    filedata = "\n".join(["[General]",\
                        "GUI=false",\
                        "USE_CPU=false",\
                        "NETWORK_PATH=berkeley_2018/new_full_network/",\
                        "USE_JOHNSON_ROUTING=false",\
                        "USE_SP_ROUTING=true",\
                        "USE_PREV_PATHS=false",\
                        "LIMIT_NUM_PEOPLE=256000",\
                        "ADD_RANDOM_PEOPLE=false",\
                        "NUM_PASSES=1",\
                        "TIME_STEP=0.5",\
                        "START_HR=5",\
                        "END_HR=12",\
                        "SHOW_BENCHMARKS=false",\
                        "a=0.8",\
                        "b=0.8",\
                        "T=7.0",\
                        "s_0=3.5", \
                        "OD_DEMAND_FILENAME=od_demand_5to12.csv", \
                        " "])

    for (parameter_name, parameter_value) in params.items():
        filedata = re.sub('{}=(-|(0-9)|.)*\n'.format(parameter_name),
                          "{}={}\n".format(parameter_name, parameter_value),
                          filedata)

    with open('command_line_options.ini', 'w') as file:
        file.write(filedata)

def load_edges_u_v(path):
    edges_u = pd.read_csv('{}/edges_u.txt'.format(path), names=['index'], header=None)
    edges_v = pd.read_csv('{}/edges_v.txt'.format(path), names=['index'], header=None)
    edges_u = edges_u.astype(int)
    edges_v = edges_v.astype(int)
    edges_u = edges_u['index'].tolist()
    edges_v = edges_v['index'].tolist()
        
    return edges_u, edges_v

def meters_per_second_to_miles_per_hour(df):
    return df * 2.23694

def create_network_from_edges_node_ids(edges_u, edges_v, path):
    num_printouts = 7
    all_edges_vel = pd.DataFrame({'osmid_u': edges_u, 'osmid_v': edges_v})

    for ind in range(num_printouts):
        df_one_edge_vel = pd.read_csv('{}/all_edges_vel_{}.txt'.format(path, ind))
        all_edges_vel['time_{}'.format(ind)] = meters_per_second_to_miles_per_hour(df_one_edge_vel)
        
    all_edges_vel['microsim_avg'] = all_edges_vel[['time_0', 'time_1', 'time_2', 'time_3', 'time_4', 'time_5', 'time_6']].mean(axis=1)

    return all_edges_vel


class benchmark():
    def __init__(self, task_name):
        self.task_name = task_name
        self.start_time_benchmark = time.time()
        print(colored("Starting task '{}'".format(task_name), 'green'))
    
    def end(self):
        elapsed_time = time.time() - self.start_time_benchmark
        print(colored("Ended task '{}'. Time elapsed: ~{} mins"\
            .format(self.task_name, round(elapsed_time/60,4)),'green'))


def add_custom_legend(ax, txt, fontsize = 12, loc = 1, *args, **kwargs):
    at = AnchoredText(txt,
                    prop=dict(size=fontsize), 
                    frameon=True,
                    loc=loc)
    at.patch.set_boxstyle("round,pad=0.2,rounding_size=0.2")
    ax.add_artist(at)
    return at

def make_legend_string_human_readable(params, diff):
    result = ""
    for param in ['a', 'b', 'T', 's_0']:
        result += param + ': ' + str(params[param]) + '\n'
    result += 'Avg diff. with Uber: {}'.format(round(diff, 4))
    
    return result

def calibrate(param_list, path, new_uber, plot_num_steps_on_each_run = False):
    min_diff = None
    min_params = None

    for params in param_list:
        write_options_file(params)

        print("Running LivingCity with params [a:{}, b:{}, T:{}, s_0:{}]".\
                        format(params["a"], params["b"], params["T"], params["s_0"]))

        simulation_benchmark = benchmark("Simulation")
        subprocess.run(["./LivingCity", "&"], check=True)
        simulation_benchmark.end()

        df_people = pd.read_csv("0_people5to12.csv")

        print("Number of nans: {}".format(df_people['avg_v(mph)'].isna().sum()))

        edges_u, edges_v = load_edges_u_v(path)
        
        #load street network and edge/node data
        nodes, edges = load_network("nodes.csv", "edges.csv", "berkeley_2018/new_full_network")

        #get edge speed data from microsim
        edge_vels_df = create_network_from_edges_node_ids(edges_u, edges_v, path)

        # Merge our network edges with Uber edges
        merged_edges = merge_edges_with_uber_edges(edges, new_uber)
        merged_edges = merged_edges.dropna()

        # Merge edge vels (with x timesteps) with the merged Uber data
        merged_edges = merge_edges_with_times_with_uber_edges(edge_vels_df, merged_edges)

        merged_edges = merged_edges.dropna()

        # Average the speeds over all the timesteps from the microsim and add column to the df
        merged_time_edges = merged_edges.copy()

        merged_time_edges['microsim_avg'] = merged_time_edges[['time_0', 'time_1', 'time_2', 'time_3', 'time_4', 'time_5', 'time_6']].mean(axis=1)

        # Add *difference* column to show delta between our microsim and Uber speeds
        merged_time_edges['diff'] = merged_time_edges['speed_mph_mean'] - merged_time_edges['microsim_avg']

        
        merged_time_edges['microsim_avg'] = merged_time_edges[['time_0', 'time_1', 'time_2', 'time_3', 'time_4', 'time_5', 'time_6']].mean(axis=1)
        merged_time_edges['speed_limit_vs_microsim'] = merged_time_edges['speed_mph'] - merged_time_edges['microsim_avg']

        print("uber avg = {}".format(merged_time_edges['speed_mph_mean'].mean()))
        print("microsim avg = {}".format(merged_time_edges['microsim_avg'].mean()))

        merged_time_edges['uber_microsim_ratio'] = merged_time_edges['speed_mph_mean'] / merged_time_edges['microsim_avg']

        #calculate RMSE
        diff = abs(merged_time_edges['diff'].mean())


        #calculate RMSE
        rmse = np.sqrt(sum(merged_time_edges['diff']**2) / merged_time_edges.shape[0])

        print("For [a:{}, b:{}, T:{}, s_0:{}] we have diff:{} and RMSE:{}".format( \
            params["a"], params["b"], params["T"], params["s_0"], diff, rmse))

        
        if plot_num_steps_on_each_run:
            # Plot num steps vs distance
            filename = os.path.join('num_steps_vs_distance_experiment', 'run_{}.png'.format(params))
            log('Plotting {}'.format(filename))
            plt.scatter(df_people.distance / 1000, df_people.num_steps / 60)
            ax = plt.gca()
            add_custom_legend(ax, make_legend_string_human_readable(params, diff))
            plt.title("Number of steps (simulated trip minutes) vs trip distance")
            plt.xlabel('Trip distance (miles)')
            plt.ylabel('Number of steps (simulated minutes)') 
            fig = plt.gcf()
            fig.set_size_inches(18.5, 10.5)
            os.makedirs('num_steps_vs_distance_experiment', exist_ok=True)
            plt.savefig(filename)
            ax.clear()
            plt.clf()
            plt.close()
        
        
        if (min_diff is None or diff < min_diff):
            print("Found new minimum.")
            min_diff = diff
            min_params = params.copy()
    
    return min_diff, min_params


def generate_next_step_parameters(params, decay, learning_rate_params, lower_bound_params, upper_bound_params):
    # Based on the current parameters, we return a possibility for the next step's parameters

    # Set default bounds
    for param in params.keys():
        if not param in lower_bound_params or lower_bound_params[param] < 0:
            lower_bound_params[param] = 0
        
        if not param in upper_bound_params:
            upper_bound_params[param] = math.inf

    # Search for next step parameters inside those bounds
    next_step_params = {}
    for param in params.keys():
        minimum_value_for_next_step_param = max(lower_bound_params[param],
                                                params[param] - learning_rate_params[param]*decay)

        maximum_value_for_next_step_param = min(upper_bound_params[param],
                                                params[param] + learning_rate_params[param]*decay)
        next_step_params[param] = np.random.uniform(minimum_value_for_next_step_param, maximum_value_for_next_step_param)
    
    
    return next_step_params.copy()

def first_step_parameters(lower_bound_params, upper_bound_params):
    default_low = {'a': 1, 'b': 1, 'T': 0.1, 's_0': 1}
    default_high = {'a':10, 'b':10, 'T':2, 's_0': 5}

    first_step_params = {}
    for param in ['b', 'b', 'T', 's_0']:
        first_step_params[param] = np.random.uniform(low=max(default_low[param], lower_bound_params[param]),
                                                    high=min(default_high[param], upper_bound_params[param]))

    return first_step_params.copy()

def determine_decay(number_of_steps):
    if number_of_steps <= 7:
        decay = 1 # no decay
    elif number_of_steps > 7:
        decay = 0.1
    elif number_of_steps > 12:
        decay = 0.01
    elif number_of_steps > 15:
        decay = 0.001
    return decay

def gradient_descent(epsilon=0.04,
                    learning_rate_params = None,
                    progress_filename = None,
                    starting_params = None,
                    load_saved_progress = False,
                    lower_bound_params = None,
                    upper_bound_params = None,
                    plot_num_steps_on_each_run = False,
                    start_time=5, end_time=12):
    
    assert starting_params is None or set(starting_params.keys()) == {'a', 'b', 'T', 's_0'}
    assert not (starting_params and load_saved_progress), \
            'Error! You cannot load saved progress and determine the starting parameters at the same time'
    if load_saved_progress:
        print("Loading progress from file {}".format(progress_filename))
    else:
        print("Saving progress at file {}".format(progress_filename))
    if lower_bound_params:
        print("Using lower bound for parameters {}".format(lower_bound_params))
    if upper_bound_params:
        print("Using upper bound for parameters {}".format(upper_bound_params))

    for param in ['a', 'b', 'T', 's_0']:
        if not param in lower_bound_params.keys():
            lower_bound_params[param] = -math.inf
        if not param in upper_bound_params.keys():
            upper_bound_params[param] = math.inf
    
    print("Epsilon: {} (max delta in mph that we're willing to have our RMSE)".format(epsilon))

    #filter Uber data to 5am to 12pm
    df_uber = pd.read_csv("berkeley_2018/uber_data/movement-speeds-quarterly-by-hod-san-francisco-2019-Q2.csv")
    df_uber = df_uber[(df_uber['hour_of_day'] >= start_time) & (df_uber['hour_of_day'] < end_time)]
    df_uber = df_uber[['osm_start_node_id','osm_end_node_id','speed_mph_mean']].groupby(['osm_start_node_id','osm_end_node_id']).mean()
    df_uber = df_uber.reset_index(level=['osm_start_node_id', 'osm_end_node_id'])
    print("number of trips from {} to {} = {} in actual Uber data".format(start_time, end_time, df_uber.shape[0]))

    progress = []

    iteration = 0
    prev_params = {"a": None, "b": None, "T": None, "s_0": None}
    current_params = {"a": None, "b": None, "T": None, "s_0": None}
    prev_diff = None

    default_learning_rate_params = {"a":1, "b":1, "T": 0.5, "s_0": 0.1}
    for param in default_learning_rate_params.keys():
        if not param in learning_rate_params:
            learning_rate_params[param] = default_learning_rate_params[param]
    print("Learning rate for parameters: {}".format(learning_rate_params))

    decay = determine_decay(len(progress)) 
    
    if load_saved_progress:
        print("Loading saved progress from {}...".format(progress_filename))
        with open('{}.json'.format(progress_filename), 'r') as fp:
            loaded_progress = json.load(fp)
        iteration = loaded_progress["number_of_iterations"] + 1
        progress = loaded_progress["progress"]
        current_params = progress[-1]["params"].copy()
        current_diff = progress[-1]["current_diff"]
        prev_diff = current_diff
    else:
        if starting_params:
            possible_next_params = [starting_params]
        else:
            possible_next_params = [first_step_parameters(lower_bound_params, upper_bound_params) for _ in range(5)]
        current_diff = None

    while True:
        print(colored("----------------- ITERATION #{}\n".format(iteration), "cyan"))
        bench_iteration = benchmark("Iteration #{}".format(iteration))
        if iteration > 0:
            # we store the previous values
            prev_params = current_params.copy()
            bench_next_step = benchmark("Getting next step parameters")
            possible_next_params = [generate_next_step_parameters(current_params, \
                                                                decay, \
                                                                learning_rate_params=learning_rate_params, \
                                                                lower_bound_params=lower_bound_params, \
                                                                upper_bound_params=upper_bound_params) for _ in range(5)]
            bench_next_step.end()

        print("Current progress: {}" .format(progress))
        print("Current parameters: {}".format(current_params))
        print("Current diff: {}".format(current_diff))
        
        print("Next steps that will be tested on this iteration:")
        for one_possible_step in possible_next_params:
            print(one_possible_step)
        prev_params = current_params.copy()
        current_diff, current_params = calibrate(possible_next_params, ".", df_uber, plot_num_steps_on_each_run)

        if (prev_diff != None and current_diff >= prev_diff):
            print("Not moving in that direction since current_diff={} and prev_diff={}".format(current_diff, prev_diff))
            current_params = prev_params.copy()
            current_diff = prev_diff
        else:
            prev_diff = current_diff
            progress.append({"params":current_params, "current_diff":current_diff})

        if progress_filename:
            print("Saving progress at {}...".format(progress_filename))
            saved_progress = {}
            saved_progress["number_of_iterations"] = iteration
            saved_progress["progress"] = progress
            with open('{}.json'.format(progress_filename), 'w') as fp:
                json.dump(saved_progress, fp)    

        if abs(current_diff) < epsilon:
            print("Found parameters that provide a diff minor to the given epsilon.")
            print(current_params)
            print("epsilon: {}  |  diff: {}".format(epsilon, current_diff))
            return current_params
        iteration += 1
        bench_iteration.end()

def count_number_of_nans(params):
    log("Running LivingCity with params [a:{}, b:{}, T:{}, s_0:{}]". \
                        format(params["a"], params["b"], params["T"], params['s_0']))
    subprocess.run(["./LivingCity", "&"], check=True)
    df_people = pd.read_csv("0_people5to12.csv")
    return df_people['avg_v(mph)'].isna().sum()

def find_s_0_that_provides_no_nans(fixed_params, s_0_lower, s_0_upper, s_0_step):
    assert fixed_params.keys() == {"a", "b", "T"}
    log("Fixed parameters: {}".format(fixed_params))
    log("Searching for a s_0 from {} to {} with a step of {}...".format(s_0_lower, s_0_upper, s_0_step))

    nans_found_for_each_s_0 = {}

    for s_0 in np.arange(s_0_lower, s_0_upper, s_0_step):
        log("Running LivingCity with params [a:{}, b:{}, T:{}, s_0:{}]". \
                        format(fixed_params["a"], fixed_params["b"], fixed_params["T"], s_0))
        subprocess.run(["./LivingCity", "&"], check=True)

        df_people = pd.read_csv("0_people5to12.csv")
        number_of_nans = df_people['avg_v(mph)'].isna().sum()
        log("Number of nans found at the column 'avg_v(mph)': {}".format(number_of_nans))

        nans_found_for_each_s_0[s_0] = number_of_nans

    log("Fixed parameters used: {}".format(fixed_params))
    log("Searched for a s_0 from {} to {} with a step of {}.".format(s_0_lower, s_0_upper, s_0_step))
    log("The values found were: {}".format(nans_found_for_each_s_0))

    return fixed_params

def plot_num_steps_vs_distance(save_path):
    df_people = pd.read_csv('0_people5to12.csv')
    plt.scatter(df_people.distance / 1000, df_people.num_steps / 60)
    plt.savefig(save_path)

def plot_num_steps_vs_distance_experiment():
    params = {'a': 3,
            'b': 5,
            'T': 0.4,
            's_0': 1.1}

    write_options_file(params)
    print("Running LivingCity with params [a:{}, b:{}, T:{}, s_0:{}]".\
                        format(params["a"], params["b"], params["T"], params["s_0"]))
    subprocess.run(["./LivingCity", "&"], check=True)

    df_people = pd.read_csv('0_people5to12.csv')
    plt.scatter(df_people.distance / 1000, df_people.num_steps / 60)

    output_dir = os.path.join('numstepsvsdistance', 'changinga')
    fig_filename = 'a_{}_b_{}_t_{}_s0_{}.png'.format(\
        round(params['a'], 2), round(params['b'], 2), round(params['T'], 2), round(params['s_0'], 2))
    plt.savefig(os.path.join(output_dir, fig_filename))

if __name__== "__main__":
    now = datetime.now()
    print("Running at {}hs PST on the day {} (d/m/Y).".format(now.strftime("%H:%M:%S"),now.strftime("%d/%m/%Y")))


    # For [a:0.557040909258405, b:2.9020578588167, T:0.5433027817144876, s_0:1.3807498735425845]
    # we have diff:0.3459298931668383 and RMSE:10.326982421083729

    params = {'a':0.557040909258405, 'b':2.9020578588167, 'T':0.5433027817144876, 's_0':1.3807498735425845}
    LR = {'a': 0.8, 'b': 0.5, 'T': 0.1, 's_0':0.1}
    for epsilon in [1, 0.5, 0.12, 0.12]:
        params = gradient_descent(epsilon = 0.4,
                        learning_rate_params = {"a": LR["a"], "b": LR["b"], 'T': LR['T'], 's_0': LR['s_0']},
                        starting_params = params, 
                        progress_filename = "calibration_progress",
                        lower_bound_params = {'a': 0.5, 'b': 2, "s_0": 1.0, "T": 0.3},
                        upper_bound_params = {"a": 7, "b": 4, "s_0": 1.5, 'T': 0.85},
                        plot_num_steps_on_each_run = True)
                    
        # decay after reaching epsilon
        for k in LR.keys():
            LR[k] = LR[k] * 0.6
