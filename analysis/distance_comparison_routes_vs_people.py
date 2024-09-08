import pandas as pd
from tqdm import tqdm
from pdb import set_trace as st
import os
import sys

import config

"""
  Merges the distances of each person according to the output obtained from 0_people5to12.csv
  and the sum of the edges obtained from 0_route5to12.csv and edges.csv.
  Saves the merge of both distances for each person as a csv file.
  In case of finding any discrepancies, outputs one of them.

  Args:
    edges_file: Path to the edges information file.

    people_file: Path to the people information file. Default given by config.py

    route_file: Path to the route information file. Default given by config.py

    stop_if_discrepancy_found:  if true, stops as soon as it finds a discrepancy between
                                the two distances of a person. Useful for testing without having to
                                wait for the whole network to be processed. Default: true

    output_file: Path to distance merge output. Default given by config.py
                Outputted as csv with 3 columns: person_id,distance_sum_of_edges,distance_people_info
  
  Returns:
    Nothing
"""
def merge_distances_from_route_and_people_files(
      edges_file,
      people_file = config.people_file,
      route_file = config.route_file,
      stop_if_discrepancy_found=True,
      output_file = config.distance_merge_file):
  
  # check if output file already exists
  if (os.path.isfile(output_file)):
    answer = "no answer yet"
    while(not answer.lower() in {"y","yes","n","no",""}):
      answer = input("{} already exists. Do you want to replace it? (Y/n): ".format(output_file))
    if (answer.lower() in {"n","no"}):
      print("Stopping.")
      return

  # check if input files do not exist
  for an_input_file in [edges_file,people_file,route_file]:
    if (not os.path.isfile(an_input_file)):
      raise FileNotFoundError("{} not found.".format(an_input_file))

  print("Loading edges from {}...".format(edges_file))
  print("Loading people from {}...".format(people_file))
  print("Loading routes from {}...".format(route_file))
  pd_people = pd.read_csv(people_file)
  number_of_people = len(pd_people.index)
  pd_edges = pd.read_csv(edges_file)

  f = open(output_file, "w")
  f.write("person_id,distance_sum_of_edges,distance_people_info\n")

  print("Processing routes...")
  # reads in chunks to reduce memory usage
  (discrepancy_person, discrepancy_distance_people_info, discrepancy_distance_sum_of_edges) = (None, None, None)
  for chunk_route in tqdm(pd.read_csv(route_file, sep=":", chunksize=config.pandas_chunksize), total=number_of_people/1000):
    for _, row in chunk_route.iterrows():
      person_id = str(row["p"])

      route = str(row["route"])
      route = route.replace("[", "").replace("]", "")
      route = route.split(",")
      route = route[:-1]  # delete last extra comma

      distance_sum_of_edges = 0
      for edge_id in route:
        distance_sum_of_edges += pd_edges.loc[int(edge_id)]["length"]

      distance_people_info = pd_people.loc[int(person_id)]['distance']

      f.write("{},{},{}\n".format(person_id, distance_sum_of_edges, distance_people_info))

      if (distance_sum_of_edges != distance_people_info):
        discrepancy_person = person_id
        discrepancy_distance_people_info = distance_people_info
        discrepancy_distance_sum_of_edges = distance_sum_of_edges
    
    if (stop_if_discrepancy_found and discrepancy_person):
      break

  print("Saved {}.".format(output_file))
  
  if (discrepancy_person):
    print("Discrepancy has been found for person {}. \
           Distance according to people info: {}. \
           Distance according to sum of edges: {}.\n Stopping.". \
           format(discrepancy_person,discrepancy_distance_people_info,discrepancy_distance_sum_of_edges))
  else:
    print("No discrepancies found")

  f.close()


if __name__ == '__main__':
  merge_distances_from_route_and_people_files(
    config.edges_file,
    stop_if_discrepancy_found=True)
