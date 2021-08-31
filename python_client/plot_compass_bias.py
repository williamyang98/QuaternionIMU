# %%
import matplotlib.pyplot as  plt
import numpy as np
import pandas as pd

# %%
data_file_fmt = "./data/data_{sensor}_{i}.csv"
i = 20
compass_df = pd.read_csv(data_file_fmt.format(sensor='compass', i=i))
compass_data = compass_df.to_numpy(dtype=np.float32)
dt = compass_data[:,0]
dt -= dt[0]
m_xyz = compass_data[:,1:]

# %% Plot xyz graphs
fig, axs = plt.subplots(3, 3, figsize=(10,10))
ax_names = ["x","y","z"]

for i in range(3):
    for j in range(3):
        ax = axs[j,i]
        ax.plot(m_xyz[:,i], m_xyz[:,j])
        ax.grid(True)
        ax.set_xlabel(f"{ax_names[i]}")
        ax.set_ylabel(f"{ax_names[j]}")
        ax.set_title(f"{ax_names[i]}/{ax_names[j]}")

plt.tight_layout()
plt.show()
