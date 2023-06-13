import time
import numpy as np
import torch
import socket
import struct

import unitree_api_wrapper
from unitree_api_wrapper.go1_controller import Go1Controller


# TODO: Make sure to hang the robot from the gantry when testing this!!!

def create_socket():
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # IP and port to listen on
    listen_ip = '0.0.0.0'  # Listen on all available interfaces
    listen_port = 5006  # Replace with the actual port to listen on
    sock.bind((listen_ip, listen_port))
    return sock

if __name__ == "__main__":
    cmd_socket = create_socket()
    controller = Go1Controller(policy_path="go1_flat_novel-Aug24_13-58-37_-jitted.pt")
    controller.connect_and_stand()

    print("------------------------------------------------------")
    print("Starting policy")
    print("------------------------------------------------------")
    while True:
        # Receive data
        data, addr = cmd_socket.recvfrom(1024)
        lin_x, lin_y, ang_z = struct.unpack('!fff', data)
        state, obs, action = controller.control_highlevel(torch.Tensor([lin_x, lin_y, ang_z]))
        print ("===")
        print (obs)
        print (action)
