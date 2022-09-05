from tkinter import Tk, Scale, HORIZONTAL, Button, mainloop
import torch
from unitree_api_wrapper.go1_controller import Go1Controller

controller = Go1Controller(policy_path="go1_flat_novel-Aug24_13-58-37_-jitted.pt")
controller.connect_and_stand()

root = Tk()

maxval = 0
init_steps = 10
step_counter = 0


def step_env():
    global obs, maxval, step_counter

    slider_vals = [v.get() for v in sliders]
    cmd = torch.Tensor(slider_vals)
    state, obs, action = controller.control_highlevel(cmd)

    root.after(controller.dt, step_env)  # reschedule event in 10 milliseconds


sliders = []
labels = ["lin_vel_x", "lin_vel_y", "ang_vel"]
for _, label in zip(range(6), labels):
    slider = Scale(root, from_=-5, to=5, orient=HORIZONTAL, length=500, resolution=0.2, label=label)
    slider.set(0)
    slider.pack()
    sliders.append(slider)

butt = Button(root, text="Quit", command=quit)
butt.pack()

root.after(10, step_env)
root.mainloop()
