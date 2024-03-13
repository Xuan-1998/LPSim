import pandas as pd

# Function to replicate and update data, change num_replicas to get the number you want
def replicate_and_update(data, num_replicas=480):
    replicated_data = pd.DataFrame()  # Initialize an empty dataframe
    max_sampn = data['SAMPN'].max()  # Get the maximum SAMPN from the original data
    for i in range(num_replicas):
        temp_data = data.copy()  # Make a copy of the original data
        temp_data['dep_time'] += 0.5 * i  # Update dep_time for each replica
        temp_data['SAMPN'] = temp_data['SAMPN'] + (max_sampn + 1) * i  # Update SAMPN for each replica
        replicated_data = pd.concat([replicated_data, temp_data])  # Append updated copy to the new dataframe
    return replicated_data

# Load the original data
file_path = 'generated_data_50000.csv'  # Adjust the file path as needed
data = pd.read_csv(file_path)

# Replicate and update the data 10 times
replicated_data = replicate_and_update(data)

# Save the replicated and updated data to a new CSV file
new_file_path = 'new_replicated_data_24000000.csv'
replicated_data.to_csv(new_file_path, index=False)  # Save without the index column
