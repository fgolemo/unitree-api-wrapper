from tkinter import Tk, Scale, HORIZONTAL, Button, mainloop
import torch
from unitree_api_wrapper.go1_controller import Go1Controller
from pupperfetch.legged_gym.utils.plotter import LivePlotter

controller = Go1Controller(policy_path="go1_easy_rough-23Jul27_14-50-28_v1.pt")
controller.connect_and_stand()

root = Tk()

maxval = 0
init_steps = 10
step_counter = 0

plotter = LivePlotter()


def step_env():
    global obs, maxval, step_counter

    slider_vals = [v.get() for v in sliders]
    cmd = torch.Tensor(slider_vals)
    state, obs, action = controller.control_highlevel(cmd)
    plotter.plot_stuff(obs, action)

    root.after(int(controller.dt * 1000), step_env)  # reschedule event in 10 milliseconds


sliders = []
labels = ["lin_vel_x", "lin_vel_y", "ang_vel"]
for label in labels:
    slider = Scale(root, from_=-3, to=3, orient=HORIZONTAL, length=500, resolution=0.2, label=label)
    slider.set(0)
    slider.pack()
    sliders.append(slider)

slider = Scale(root, from_=0.15, to=0.3, orient=HORIZONTAL, length=500, resolution=0.01, label="height")
slider.set(0.25)
slider.pack()
sliders.append(slider)

butt = Button(root, text="Quit", command=quit)
butt.pack()

root.after(10, step_env)
root.mainloop()


# [x] make sim playback script that is single robot and interactive - to check clipping when walking sideways
# [x] change heading command to angular vel
# [ ] add angular vel to environment (sim and real), retrain policy, run on robot
# [x] live plotter for obs and policy
# [ ] simon: add history
# [ ] ken: progressively increasing penalties
