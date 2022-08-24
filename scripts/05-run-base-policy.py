import time
import numpy as np
import torch

import unitree_api_wrapper
from unitree_api_wrapper.go1_controller import Go1Controller


# TODO: Make sure to hang the robot from the gantry when testing this!!!

if __name__ == "__main__":
    controller = Go1Controller(policy_path="go1_flat_novel-Aug24_13-58-37_-jitted.pt")
    controller.connect_and_stand()

    print("------------------------------------------------------")
    print("Starting policy")
    print("------------------------------------------------------")
    cmd = torch.Tensor([0.5, 0, 0])
    while True:
        state, obs, action = controller.control_highlevel(cmd)
        time.sleep(controller.dt) # very important - same amount of time between actions like in sim, respecting decimation
        print ("===")
        print (obs)
        print (action)


