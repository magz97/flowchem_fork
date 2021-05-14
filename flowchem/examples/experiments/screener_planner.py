import pandas as pd
from pathlib import Path

# Target dataframe to be saved
conditions = pd.DataFrame(columns=['eq', 'tR', 'T'])

# Parameter range
equivalents = {1, 1.1, 1.2}  # {1, 1.05, 1.1, 1.15, 1.2}
residence_times = {0.5, 1, 2.5, 5, 10, 15, 20}
temperature = {None, 30, 40, 50, 60}

# Create df entry for each combination of parameter (fully factorial)
experimental_conditions = [(eq, t_res, temp) for temp in temperature for t_res in residence_times for eq in equivalents]
conditions['eq'], conditions['tR'], conditions['T'] = zip(*experimental_conditions)

# Destination file
path_to_write_csv = Path().home() / "Documents"
conditions.to_csv(path_to_write_csv / "chlorination_14_05_21.csv")
print(path_to_write_csv)
