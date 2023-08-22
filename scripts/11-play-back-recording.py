import time

import numpy as np
from pupperfetch.legged_gym.utils.plotter import LivePlotter

filename = "real-log-2023-08-03.npz"

data = np.load(filename, allow_pickle=True)
obs = data["obs"]
act = data["act"]

plotter = LivePlotter(100, 0.25)
for idx in range(len(obs)):
    plotter.plot_stuff(obs[idx], act[idx])
    time.sleep(0.1)


