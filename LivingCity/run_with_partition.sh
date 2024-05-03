#!/bin/bash
set -e
python3 -m pip install --upgrade pip

python3 -m pip install pandas numpy tqdm python-igraph matplotlib networkx pymetis contextily scikit-learn
living_city_dir="LivingCity"
ini_file="command_line_options.ini"
routing_script_path="algorithm_partition.py"

start_hr=$(grep "^START_HR=" $ini_file | cut -d'=' -f2)
end_hr=$(grep "^END_HR=" $ini_file | cut -d'=' -f2)
network_path=$(grep "^NETWORK_PATH=" $ini_file | cut -d'=' -f2)

partition_filename=$(grep "^PARTITION_FILENAME=" $ini_file | cut -d'=' -f2)
partition_filename=${partition_filename:-partitions.txt}

echo $start_hr
echo $partition_filename
route_file="0_route${start_hr}to${end_hr}.csv"

if [ -f "$route_file" ]; then
    echo "Route file found. Running Python script to generate partition file."
    echo "python3 ${routing_script_path} $route_file $partition_filename $network_path"
    python3 ${routing_script_path} "$route_file" "$partition_filename" "$network_path"
    echo "Gernerate $partition_filename"
    echo "Run LivingCity Using $partition_filename"
    sed -i "s/^PARTITION_FILENAME=.*/PARTITION_FILENAME=${partition_filename}/" $ini_file
    docker run -it --rm --init --gpus all -v "$PWD":/lpsim -w /lpsim xuanjiang1998/lpsim:v1 bash -c "./LivingCity"
else
    echo "Route file not found. Running LivingCity to generate the route file."
    docker run -it --rm --init --gpus all -v "$PWD":/lpsim -w /lpsim xuanjiang1998/lpsim:v1 bash -c "./LivingCity"
    python3 ${routing_script_path}"$route_file" "$partition_filename" "$network_path"
    echo "Gernerate $partition_filename"
    echo "Run LivingCity Using $partition_filename"
    sed -i "s/^PARTITION_FILENAME=.*/PARTITION_FILENAME=${partition_filename}/" $ini_file
    docker run -it --rm --init --gpus all -v "$PWD":/lpsim -w /lpsim xuanjiang1998/lpsim:v1 bash -c "./LivingCity"
fi
