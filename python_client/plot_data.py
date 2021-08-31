# %%
import matplotlib.pyplot as  plt
import numpy as np
import pandas as pd

# %%
data_file_fmt = "./data/data_{sensor}_{i}.csv"
i = 15
gyro_df = pd.read_csv(data_file_fmt.format(sensor='gyro', i=i))
compass_df = pd.read_csv(data_file_fmt.format(sensor='compass', i=i))

gyro_data = gyro_df.to_numpy(dtype=np.float32)
compass_data = compass_df.to_numpy(dtype=np.float32)

start_time = min([gyro_data[:,0].min(), compass_data[:,0].min()])
gyro_data[:,0] -= start_time
compass_data[:,0] -= start_time

accel_data = gyro_data[:,[0,1,2,3]]
pqr_data = gyro_data[:,[0,4,5,6]] 

# %% Plot the data
data = [accel_data, pqr_data, compass_data]
data_labels = ["Ax Ay Az".split(), "p q r".split(), "Mx My Mz".split()]
ylabels = ["Linear Acceleration (ms-2)", "Rotation Velocity (rads-1)", "Magnetic Field Strength (Gauss)"]
titles = ["Linear Acceleration", "Body Rates", "Compass"]

fig, axs = plt.subplots(3,1, figsize=(20,20))
for j, (data, labels, ylabel, title) in enumerate(zip(data, data_labels, ylabels, titles)):
    ax = axs[j]
    for i, label in enumerate(labels):
        ax.plot(data[:,0], data[:,i+1], label=label)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True)

plt.tight_layout()
plt.show()

# %% Compute the dot produce between the accelerometer and magnetometer vectors
# If we have the axes aligned properly after converter
# Then the angle between the vectors will remain the same
# Otherwise, our axes are misaligned
N = min(accel_data.shape[0], compass_data.shape[0])
a_xyz = accel_data[:N,1:]
m_xyz = compass_data[:N,1:]
a_xyz_norm = a_xyz / np.linalg.norm(a_xyz, axis=1)[:,None]
m_xyz_norm = m_xyz / np.linalg.norm(m_xyz, axis=1)[:,None]
angle = np.arccos(np.sum(a_xyz_norm*m_xyz_norm, axis=1))

dt = gyro_data[:,0]
plt.figure(figsize=(10,10))
plt.plot(dt, angle)
plt.grid(True)
plt.xlabel("Time (seconds)")
plt.ylabel("Angle between Axyz and Mxyz (rads)")
plt.show()

# %% Plot the time values to make sure they are linear
dt = gyro_data[:,0]
plt.figure()
plt.plot(dt)
plt.show()