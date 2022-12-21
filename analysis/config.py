import os

edges_file = os.getenv("EDGES_FILE", "../LivingCity/berkeley_2018/new_full_network/edges.csv")
people_file = os.getenv("PEOPLE_FILE","../LivingCity/0_people5to12.csv")
route_file = os.getenv("ROUTE_FILE","../LivingCity/0_route5to12.csv")
pandas_chunksize = int(os.getenv("PANDAS_CHUNKSIZE", "1000"))
distance_merge_file = os.getenv("DISTANCE_MERGE_FILE","distances.csv")