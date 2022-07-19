import time
import numpy as np
import unitree_api_wrapper
from unitree_api_wrapper.go1_controller import Go1Controller

controller = Go1Controller()
controller.connect()

while True:
    time.sleep(0.002)
    state = controller.send_pos_cmd()
    # print([(x.q, x.q_raw) for x in state.motorState])
    # break
