# %%
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from ekf import LoggedExtendedKalmanFilter

# %%
filename_fmt = "./data/data_combined_{i}.csv"
i = 18
df = pd.read_csv(filename_fmt.format(i=i), header=None)
data = df.to_numpy()

dt = data[:,0]
a_xyz = data[:,[1,2,3]]
pqr   = data[:,[4,5,6]]
m_xyz = data[:,[7,8,9]]

dt -= dt[0]
Ts = np.mean(dt[1:]-dt[:-1])
Ncalibrate = int(np.ceil(1.0/Ts))

a_len = np.linalg.norm(a_xyz, axis=1)
m_len = np.linalg.norm(m_xyz, axis=1)

pqr -= np.mean(pqr[:Ncalibrate], axis=0)

# Find calibration values
a_xyz_0 = np.mean(a_xyz[:Ncalibrate], axis=0)
m_xyz_0 = np.mean(m_xyz[:Ncalibrate], axis=0)

# %%
# Get kalman filter measurements
ekf = LoggedExtendedKalmanFilter()
ekf.a_xyz_0 = a_xyz_0.reshape((3,1))
ekf.m_xyz_0 = m_xyz_0.reshape((3,1))
ekf.last_dt = dt[0]-Ts

# Get regular gyro predictions
gyro_only = LoggedExtendedKalmanFilter()
gyro_only.a_xyz_0 = a_xyz_0.reshape((3,1))
gyro_only.m_xyz_0 = m_xyz_0.reshape((3,1))
gyro_only.last_dt = dt[0]-Ts

N = len(dt)
for i in range(N):
    dt_i = dt[i]
    a_xyz_i = a_xyz[i].reshape((3,1))
    m_xyz_i = m_xyz[i].reshape((3,1))
    pqr_i = pqr[i].reshape((3,1))
    ekf.update(dt_i, a_xyz_i, m_xyz_i, pqr_i)
    gyro_only.update(dt_i, None, None, pqr_i, measure_step=False)

ekf.quats = np.array(ekf.quats).squeeze()
gyro_only.quats = np.array(gyro_only.quats).squeeze()

# %%
ekf_dcm = ekf.find_observation_matrix(ekf.quats.T).transpose([2,0,1])
gyro_dcm = gyro_only.find_observation_matrix(gyro_only.quats.T).transpose([2,0,1])

# Find the predicted body vectors
a_xyz_kf = ekf_dcm@a_xyz_0
m_xyz_kf = ekf_dcm@m_xyz_0

a_xyz_gy = gyro_dcm@a_xyz_0
m_xyz_gy = gyro_dcm@m_xyz_0

# %% Plot our data readings
def plot_data(dt, a_xyz, m_xyz, pqr, a_len, m_len):
    fig, axs = plt.subplots(2,3, figsize=(20,10))

    ax = axs[0,0]
    ax.plot(dt, a_xyz[:,0], label="x")
    ax.plot(dt, a_xyz[:,1], label="y")
    ax.plot(dt, a_xyz[:,2], label="z")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Acceleration (ms-2)")
    ax.set_title("Accelerometer Readings")
    ax.grid(True)
    ax.legend()

    ax = axs[0,1]
    ax.plot(dt, m_xyz[:,0], label="x")
    ax.plot(dt, m_xyz[:,1], label="y")
    ax.plot(dt, m_xyz[:,2], label="z")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Magnetic Field Strength (Gauss)")
    ax.set_title("Compass Readings")
    ax.grid(True)
    ax.legend()

    ax = axs[0,2]
    ax.plot(dt, pqr[:,0], label="p")
    ax.plot(dt, pqr[:,1], label="q")
    ax.plot(dt, pqr[:,2], label="r")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angular Velocity (degree/s)")
    ax.set_title("Gyroscope Readings")
    ax.grid(True)
    ax.legend()

    ax = axs[1,0]
    ax.plot(dt, a_len)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Acceleration (ms-2)")
    ax.set_title("Accelerometer Magnitude")
    ax.grid(True)

    ax = axs[1,1]
    ax.plot(dt, m_len)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Magnetic Field Strength (Gauss)")
    ax.set_title("Compass Magnitude")
    ax.grid(True)

    plt.tight_layout()
    return fig, axs
fig, axs = plot_data(dt, a_xyz, m_xyz, pqr, a_len, m_len)
plt.show()

# %% Plot the dot product between acceleration and magnetic vectors
a_xyz_norm = a_xyz / np.linalg.norm(a_xyz, axis=1)[:,None]
m_xyz_norm = m_xyz / np.linalg.norm(m_xyz, axis=1)[:,None]
angle = np.arccos(np.sum(a_xyz_norm*m_xyz_norm, axis=1))

plt.figure(figsize=(10,10))
plt.plot(dt, angle)
plt.grid(True)
plt.xlabel("Time (seconds)")
plt.ylabel("Angle between Axyz and Mxyz (rads)")
plt.show()

# %% Plot the determined quaternions
def plot_quaternions(dt, ekf_quats, gyro_only_quats):
    fig, axs = plt.subplots(2, 1, figsize=(20,10))
    ax = axs[0]
    ax.plot(dt, ekf_quats[:,0], label="w")
    ax.plot(dt, ekf_quats[:,1], label="i")
    ax.plot(dt, ekf_quats[:,2], label="j")
    ax.plot(dt, ekf_quats[:,3], label="k")
    ax.legend()
    ax.grid(True)
    ax.set_xlabel("Time (s)")
    ax.set_title("EKF")

    ax = axs[1]
    ax.plot(dt, gyro_only_quats[:,0], label="w")
    ax.plot(dt, gyro_only_quats[:,1], label="i")
    ax.plot(dt, gyro_only_quats[:,2], label="j")
    ax.plot(dt, gyro_only_quats[:,3], label="k")
    ax.legend()
    ax.grid(True)
    ax.set_xlabel("Time (s)")
    ax.set_title("Gyro Only")

    plt.tight_layout()

    return fig, axs
plot_quaternions(dt, ekf.quats, gyro_only.quats)
plt.show()

# %% Plot the predicted body vectors
def plot_predicted_body_vectors(dt, a_xyz, m_xyz, a_xyz_kf, m_xyz_kf, a_xyz_gy, m_xyz_gy):
    fig, axs = plt.subplots(3,2, figsize=(20,10))

    ax = axs[0,0]
    ax.plot(dt, a_xyz[:,0], label="x")
    ax.plot(dt, a_xyz[:,1], label="y")
    ax.plot(dt, a_xyz[:,2], label="z")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Acceleration (ms-2)")
    ax.set_title("Accelerometer Readings")
    ax.grid(True)
    ax.legend()

    ax = axs[0,1]
    ax.plot(dt, m_xyz[:,0], label="x")
    ax.plot(dt, m_xyz[:,1], label="y")
    ax.plot(dt, m_xyz[:,2], label="z")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Magnetic Field Strength (Gauss)")
    ax.set_title("Compass Readings")
    ax.grid(True)
    ax.legend()

    ax = axs[1,0]
    ax.plot(dt, a_xyz_gy[:,0], label="x")
    ax.plot(dt, a_xyz_gy[:,1], label="y")
    ax.plot(dt, a_xyz_gy[:,2], label="z")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Acceleration (ms-2)")
    ax.set_title("Accelerometer Readings Gyro-Only")
    ax.grid(True)
    ax.legend()

    ax = axs[1,1]
    ax.plot(dt, m_xyz_gy[:,0], label="x")
    ax.plot(dt, m_xyz_gy[:,1], label="y")
    ax.plot(dt, m_xyz_gy[:,2], label="z")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Magnetic Field Strength (Gauss)")
    ax.set_title("Compass Readings Gyro-Only")
    ax.grid(True)
    ax.legend()

    ax = axs[2,0]
    ax.plot(dt, a_xyz_kf[:,0], label="x")
    ax.plot(dt, a_xyz_kf[:,1], label="y")
    ax.plot(dt, a_xyz_kf[:,2], label="z")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Acceleration (ms-2)")
    ax.set_title("Accelerometer Readings KF")
    ax.grid(True)
    ax.legend()

    ax = axs[2,1]
    ax.plot(dt, m_xyz_kf[:,0], label="x")
    ax.plot(dt, m_xyz_kf[:,1], label="y")
    ax.plot(dt, m_xyz_kf[:,2], label="z")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Magnetic Field Strength (Gauss)")
    ax.set_title("Compass Readings KF")
    ax.grid(True)
    ax.legend()

    plt.tight_layout()

    return fig, axs
plot_predicted_body_vectors(dt, a_xyz, m_xyz, a_xyz_kf, m_xyz_kf, a_xyz_gy, m_xyz_gy)
plt.show()
