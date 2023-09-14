import numpy as np
from pupperfetch.legged_gym.utils.plotter import AsyncLivePlotter
from tkinter import Tk, Scale, HORIZONTAL, Button, mainloop
import torch
from unitree_api_wrapper.go1_controller import Go1Controller

HISTORY = 1
# controller = Go1Controller(policy_path="best_position.pt", obs_history=HISTORY, with_angvel=True)
# controller = Go1Controller(policy_path="lower_base.pt", obs_history=HISTORY, with_angvel=True)
controller = Go1Controller(policy_path="Sep13_21-43-55_s2r_test.pt", obs_history=HISTORY, with_angvel=True, with_bool=True, bool_value=0)



root = Tk()

maxval = 0
init_steps = 10
step_counter = 0
robot_is_up = False

plotter = AsyncLivePlotter("real", 100, 0.25, with_angvel=True)
LOG_FOR = 1000
log_obs = []
log_act = []

def step_env():
    global obs, maxval, step_counter, robot_is_up

    if not robot_is_up:
        controller.connect_and_stand()
        robot_is_up = True

    slider_vals = [v.get() for v in sliders]
    cmd = torch.Tensor(slider_vals)
    state, obs, action = controller.control_highlevel(cmd)
    plotter.plot_stuff(obs[:, :, -1], action)
    log_obs.append(obs.detach().cpu().numpy())
    log_act.append(action)

    if len(log_obs) > LOG_FOR:
        np.savez("s2r_test.npz", obs = log_obs, act=log_act)
        print("saved file")
        quit()

    root.after(int(controller.dt * 1000), step_env)  # reschedule event in 10 milliseconds


sliders = []
labels = ["pos_x", "pos_y", "orientation"]
for label in labels:
    slider = Scale(root, from_=-1.4, to=1.4, orient=HORIZONTAL, length=500, resolution=0.2, label=label)
    slider.set(0)
    slider.pack()
    sliders.append(slider)

slider = Scale(root, from_=0.2, to=0.4, orient=HORIZONTAL, length=500, resolution=0.01, label="height")
slider.set(0.3)
slider.pack()
sliders.append(slider)

def stop():
    plotter.kill()
    quit()

butt = Button(root, text="Quit", command=stop)
butt.pack()

root.after(10, step_env)
root.mainloop()


# [x] make sim playback script that is single robot and interactive - to check clipping when walking sideways
# [x] change heading command to angular vel
# [ ] add angular vel to environment (sim and real), retrain policy, run on robot
# [x] live plotter for obs and policy
# [ ] simon: add history
# [ ] ken: progressively increasing penalties
