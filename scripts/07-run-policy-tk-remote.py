import numpy as np
from pupperfetch.legged_gym.utils.plotter import AsyncLivePlotter
from tkinter import Tk, Scale, HORIZONTAL, Button, mainloop
import torch
from unitree_api_wrapper.go1_controller import Go1Controller
from functools import partial




# controller = Go1Controller(policy_path="23Aug22_11-26-34_.pt")
HISTORY = 1
WITH_ANGVEL = True
WITH_DOFVEL = True
Go1C = partial(Go1Controller, obs_history=HISTORY, with_angvel=WITH_ANGVEL, with_dofvel=WITH_DOFVEL)

# controller = Go1Controller(policy_path="go1_easy_rough_backport_23Sep05_16-44-58_v7-sanitycheck.pt", obs_history=HISTORY, with_angvel=True)
# controller = Go1Controller(policy_path="23Aug29_18-59-23_lhum1.pt", obs_history=HISTORY, with_angvel=False) # best policy currently
# controller = Go1Controller(policy_path="23Aug29_23-56-58_go1_flat_lhum2.pt", obs_history=HISTORY, with_angvel=False)
# controller = Go1Controller(policy_path="jump.pt", obs_history=HISTORY, with_angvel=False)
# controller = Go1Controller(policy_path="23Aug30_02-04-45_go1_easy_rough.pt", obs_history=HISTORY, with_angvel=False)
# controller = Go1Controller(policy_path="23Sep06_21-00-20_v5-real-angvelcmd.pt", obs_history=HISTORY, with_angvel=WITH_ANGVEL) # shit, walks but wiggles when still
# controller = Go1Controller(policy_path="23Sep09_16-25-45_v6-reproducing-lhum1.pt", obs_history=HISTORY, with_angvel=WITH_ANGVEL)
# controller = Go1Controller(policy_path="23Sep09_16-25-45_v6-lhum-og3k.pt", obs_history=HISTORY, with_angvel=WITH_ANGVEL)
# controller = Go1Controller(
#     policy_path="23Sep12-lhum2-3k0.pt",
#     obs_history=HISTORY,
#     with_angvel=WITH_ANGVEL,
#     with_dofvel=WITH_DOFVEL,
# ) # new best
# controller = Go1C(policy_path="23Sep12-lhum3-pd-6k.pt") # new best
# controller = Go1C(policy_path="23Sep12_16-31-12_v9-lhum2-av-head.pt") # kinda shit
# controller = Go1C(policy_path="23Sep12_16-32-52_v9-lhum2-av-head-delay0.5.pt") # kinda shit
# controller = Go1C(policy_path="23Sep12_16-33-58_v9-lhum2-av-head-delay1.pt") # kinda works, kinda shit
controller = Go1C(policy_path="23Sep12_16-33-58_v9-lhum2-av-head-delay1-2k5.pt") # kinda works, kinda shit

input("press enter to launch when holding the go1")
standing = False

root = Tk()

maxval = 0
init_steps = 10
step_counter = 0

plotter = AsyncLivePlotter("real", 100, 0.25, with_angvel=WITH_ANGVEL, with_dofvel=WITH_DOFVEL)
# LOG_FOR = 1000``
# log_obs = []
# log_act = []


def step_env():
    global obs, maxval, step_counter, standing

    slider_vals = [v.get() for v in sliders]
    cmd = torch.Tensor(slider_vals)

    if not standing:
        controller.connect_and_stand()
        standing = True

    state, obs, action = controller.control_highlevel(cmd)
    plotter.plot_stuff(obs[:, :, -1], action)
    # log_obs.append(obs)
    # log_act.append(action)

    # if len(log_obs) > LOG_FOR:
    #     np.savez("real-log-2023-08-03.npz", obs = log_obs, act=log_act)
    #     quit()

    root.after(int(controller.dt * 1000), step_env)  # reschedule event in 10 milliseconds


sliders = []
labels = ["lin_vel_x", "lin_vel_y", "ang_vel"]
for label in labels:
    slider = Scale(root, from_=-3, to=3, orient=HORIZONTAL, length=500, resolution=0.2, label=label)
    slider.set(0)
    slider.pack()
    sliders.append(slider)

slider = Scale(root, from_=0.15, to=0.35, orient=HORIZONTAL, length=500, resolution=0.01, label="height")
slider.set(0.30)
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
