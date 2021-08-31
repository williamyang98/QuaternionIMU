# %%
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from ekf_client import EKFClient

# %% Load the files
filename_fmt = "./data/data_{sensor}_{i}.csv"
i = 18

# load gyro and accel data
df0 = pd.read_csv(filename_fmt.format(sensor='gyro', i=i))
data = df0.to_numpy(dtype=np.float32)
dt0 = data[:,0]
a_xyz = data[:,[1,2,3]]
pqr   = data[:,[4,5,6]]

# load compass data
df1 = pd.read_csv(filename_fmt.format(sensor='compass', i=i))
data = df1.to_numpy(dtype=np.float32)
dt1 = data[:,0]
m_xyz = data[:,[1,2,3]]

# %% Create the event stream
def create_event_stream(dt0, dt1, a_xyz, pqr, m_xyz):
    i0 = 0
    i1 = 0

    N0 = dt0.shape[0]
    N1 = dt1.shape[0]

    events = []

    while i0 < N0 and i1 < N1:
        # gyro has priority
        if dt0[i0] < dt1[i1]:
            events.append(('G', dt0[i0], a_xyz[i0], pqr[i0]))
            i0 += 1
        # compass has priority
        else:
            events.append(('C', dt1[i1], m_xyz[i1]))
            i1 += 1

    # ingest remaining events (one of them will be completed)
    while i0 < N0:
        events.append(('G', dt0[i0], a_xyz[i0], pqr[i0]))
        i0 += 1

    while i1 < N1:
        events.append(('C', dt1[i1], m_xyz[i1]))
        i1 += 1

    return events
events = create_event_stream(dt0, dt1, a_xyz, pqr, m_xyz)

# %% Begin ingesting the event stream
# create kalman filter
client = EKFClient()
client.ekf.Pk = 0.1*np.eye(4)

# log our values
Nevents = len(events)
Nprocessed = 0
dt_kf    = np.zeros((Nevents,1))
quats_kf = np.zeros((Nevents,4))
Pk_kf    = np.zeros((Nevents,4))

client.set_calibrate(True)

for i, event in enumerate(events):
    if i == 100:
        client.set_calibrate(False)

    tag, dt, *contents = event
    
    # update measurements using compass
    if tag == 'C':
        m_xyz_i = contents[0].reshape((3,1))
        client.on_compass(dt, m_xyz_i)
    # predict orientation using body rates, and update with accelerometer measurements
    elif tag == 'G':
        a_xyz_i, pqr_i = contents
        a_xyz_i = a_xyz_i.reshape((3,1))
        pqr_i = pqr_i.reshape((3,1))
        client.on_gyro(dt, a_xyz_i, pqr_i)
    else:
        raise ValueError(f"Unknown event tag {tag}")
    
    if not client.is_calibrating:
        dt_kf[Nprocessed] = dt
        quats_kf[Nprocessed,:] = client.ekf.E.T.squeeze()
        Pk_kf[Nprocessed,:] = np.sum(client.ekf.Pk**2, axis=1)
        Nprocessed += 1

dt_kf = dt_kf[:Nprocessed]
quats_kf = quats_kf[:Nprocessed]
Pk_kf = Pk_kf[:Nprocessed]

# %% PLot the live ekf values
fig = plt.figure(figsize=(20,10))
plt.plot(dt_kf, quats_kf[:,0], label="w")
plt.plot(dt_kf, quats_kf[:,1], label="i")
plt.plot(dt_kf, quats_kf[:,2], label="j")
plt.plot(dt_kf, quats_kf[:,3], label="k")
plt.legend()
plt.grid(True)
plt.xlabel("Time (s)")
plt.title("Multi rate Extended Kalman Filter")
plt.show()