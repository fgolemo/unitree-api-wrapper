import time
import numpy as np
import torch
import socket
import struct
import threading
import time

import unitree_api_wrapper
from unitree_api_wrapper.go1_controller import Go1Controller


# TODO: Make sure to hang the robot from the gantry when testing this!!!

def receive_floats(shared_data):
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # IP and port to listen on
    listen_ip = '127.0.0.1'  # Listen on all available interfaces
    listen_port = 5006  # Replace with the actual port to listen on
    sock.bind((listen_ip, listen_port))
    
    while True:
         # Discard any previous unread data
        while sock.recv(1024):
            pass
        # Receive data
        data, addr = sock.recvfrom(1024)

        # Unpack the received data into three floats
        float1, float2, float3 = struct.unpack('!fff', data)

        # Update the shared data with the latest message and timestamp
        shared_data['timestamp'] = time.time()
        shared_data['cmd'] = (float1, float2, float3)

    # Close the socket (this will never be reached in this example)
    sock.close()

def main_loop(shared_data):
    # Init the controller
    controller = Go1Controller(policy_path="go1_flat_novel-Aug24_13-58-37_-jitted.pt")
    controller.connect_and_stand()
    print("------------------------------------------------------")
    print("Starting policy")
    print("------------------------------------------------------")

    while True:
        # Check if there is a latest message available
        if 'timestamp' in shared_data and 'cmd' in shared_data:
            # Retrieve the latest message and its timestamp
            timestamp = shared_data['timestamp']
            lin_x, lin_y, ang_z = shared_data['floats']

            # Calculate the age of the message
            current_time = time.time()
            message_age = current_time - timestamp
            if message_age > 0.1:
                print("WARNING: Message is", message_age, "seconds old.")
                lin_x, lin_y, ang_z = 0.0, 0.0, 0.0

            # Run the controller
            state, obs, action = controller.control_highlevel(torch.Tensor([lin_x, lin_y, ang_z]))
            print ("===")
            print (obs)
            print (action)

if __name__ == "__main__":
    # Create shared data dictionary
    shared_data = {}

    # Start the listener in a separate thread, passing the shared data dictionary
    listener_thread = threading.Thread(target=receive_floats, args=(shared_data,))
    listener_thread.start()

    # Start the main loop, passing the shared data dictionary
    main_loop(shared_data)
