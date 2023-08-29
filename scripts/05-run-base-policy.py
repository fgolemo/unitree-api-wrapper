import time
import numpy as np
import torch

import unitree_api_wrapper
from unitree_api_wrapper.go1_controller import Go1Controller


# TODO: Make sure to hang the robot from the gantry when testing this!!!

if __name__ == "__main__":
    controller = Go1Controller(policy_path="TODO.pt", with_angvel=True)
    controller.connect_and_stand()

    print("------------------------------------------------------")
    print("Starting policy")
    print("------------------------------------------------------")
    cmd = torch.Tensor([0.0, 0.0, 0.0])
    while True:
        state, obs, action = controller.control_highlevel(cmd)
        print ("===")
        print (obs)
        print (action)


