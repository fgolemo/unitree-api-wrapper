import time
import numpy as np
from pprint import pprint
import unitree_api_wrapper
from unitree_api_wrapper.go1_controller import Go1Controller


if __name__ == "__main__":
    controller = Go1Controller(policy_path=None)
    controller.connect()

    counter = 0
    while True:
        if counter < 100:
            controller.kp = [10, 10, 10]
        elif counter < 200:
            controller.kp = [40, 40, 40]
        else:
            controller.kp = [60, 60, 60]
        time.sleep(0.02)
        state = controller.send_pos_cmd(pos_cmd=[[0,0,0],[0,0,0],[0,0,0],[0,0,0]])
        # state = controller.send_pos_cmd()
        if counter % 100 == 0:
            print('------------------------------------------------------')
            print(state.motorState[0].q, state.motorState[1].q,state.motorState[2].q)
            print(state.motorState[3].q, state.motorState[4].q,state.motorState[5].q)
            print(state.motorState[6].q, state.motorState[7].q,state.motorState[8].q)
            print(state.motorState[9].q, state.motorState[10].q,state.motorState[11].q)
        counter += 1
