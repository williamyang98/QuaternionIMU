import numpy as np
import pandas as pd
import argparse
from convert_readings import MeasurementConverter

parser = argparse.ArgumentParser()
parser.add_argument("i")
parser.add_argument("--input", default="./data/data_{sensor}_{i}.csv")
parser.add_argument("--output", default="./data/data_combined_{i}.csv")

args = parser.parse_args()

# load the data
data_file_fmt = args.input
i = 0
gyro_df = pd.read_csv(data_file_fmt.format(sensor='gyro', i=args.i))
compass_df = pd.read_csv(data_file_fmt.format(sensor='compass', i=args.i))

gyro_data = gyro_df.to_numpy(dtype=np.float32)
compass_data = compass_df.to_numpy(dtype=np.float32)

# synchonize data
Ndata = min([gyro_data.shape[0], compass_data.shape[0]])
gyro_data = gyro_data[:Ndata]
compass_data = compass_data[:Ndata]

dt = gyro_data[:,0]
m_xyz = compass_data[:,[1,2,3]]
a_xyz = gyro_data[:,[1,2,3]]
pqr = gyro_data[:,[4,5,6]]

# combine data
converter = MeasurementConverter()
converter.bias_magnetometer = np.zeros((3,1), dtype=np.float32)

combined_data = np.zeros((Ndata, 1+3+3+3), dtype=np.float32)
combined_data[:,0] = dt
combined_data[:,1:4] = a_xyz
combined_data[:,4:7] = pqr
combined_data[:,7:10] = m_xyz

combined_df = pd.DataFrame(combined_data, columns=None, index=None)
combined_df.to_csv(args.output.format(i=args.i), header=None, index=None)
