import pandas as pd
import numpy as np

# Load original data
df = pd.read_csv('demand.csv')

num_rows = 50000  # Set the number of rows you want to generate

# Determine the repeat factor to expand 'SAMPN' and 'PERNO' correctly
repeat_factor = np.ceil(num_rows / len(df)).astype(int)

# Expand 'SAMPN' and 'PERNO' to match the new desired length, then trim to exact size
expanded_SAMPN = np.repeat(df['SAMPN'], repeat_factor)[:num_rows]
expanded_PERNO = np.repeat(df['PERNO'], repeat_factor)[:num_rows]

# Generating new data with correct lengths
new_data = pd.DataFrame({
    'SAMPN': expanded_SAMPN,
    'PERNO': expanded_PERNO,
    'origin_osmid': np.arange(1, num_rows + 1),
    'destination_osmid': np.arange(1, num_rows + 1) + 1,
    'dep_time': np.random.uniform(0, 43200, num_rows).astype(int), #默认范围[10800, 97320]
    'origin': np.random.choice(df['origin'], num_rows),
    'destination': np.random.choice(df['destination'], num_rows)
})

# Ensuring 'origin' and 'destination' are not equal
mask = new_data['origin'] == new_data['destination']
while mask.any():
    new_data.loc[mask, 'destination'] = np.random.choice(df['destination'], mask.sum())
    mask = new_data['origin'] == new_data['destination']

# Save to new CSV file
new_filename = 'generated_data_50000.csv'
new_data.to_csv(new_filename, index=False)
print(f"Generated file saved as {new_filename}")


'''
# Create new DataFrame based on the requirements
new_df = original_df.copy()
new_df['origin_osmid'] = range(1, len(original_df) + 1)  # Sequential starting from 1
new_df['destination_osmid'] = range(1, len(original_df) + 1)  # Sequential starting from 1
new_df['dep_time'] = np.random.choice(np.arange(10800, 97320, 30), size=len(original_df))  # Random dep_time

# Ensure origin and destination are randomly chosen from unique values and not equal
unique_origins = original_df['origin'].unique()
unique_destinations = original_df['destination'].unique()
new_origins = []
new_destinations = []

for _ in range(len(original_df)):
    origin = np.random.choice(unique_origins)
    destination = np.random.choice(unique_destinations)
    while origin == destination:  # Ensure they are not equal
        destination = np.random.choice(unique_destinations)
    new_origins.append(origin)
    new_destinations.append(destination)

new_df['origin'] = new_origins
new_df['destination'] = new_destinations

# Save the new DataFrame to a CSV file
new_csv_path = 'modified_data.csv'
new_df.to_csv(new_csv_path, index=False)
new_csv_path
'''