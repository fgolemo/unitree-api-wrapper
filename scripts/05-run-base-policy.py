import time
import numpy as np
import unitree_api_wrapper
from unitree_api_wrapper.go1_controller import Go1Controller


if __name__ == "__main__":
    dt = 1.0 / 50.0
    controller = Go1Controller()
    controller.connect()
    
    for _ in range(int(8.0 / dt)):
        controller.send_pos_cmd()
        time.sleep(dt)
    
    controller.kp = [40, 40, 40]
    counter = 0
    obs_list = []
    action_list = []
    while True:
        start = time.perf_counter()
        state, obs, action = controller.control_highlevel()
        obs_list.append(obs)
        action_list.append(action)
        diff = dt - (time.perf_counter() - start)
        if diff > 0:
            time.sleep(diff)
        counter += 1
        
        if counter % 100 == 0:
            print(action)
        
        if counter == 1000:
            np.savez('go1_data_2022_08_01.npz', obs_list=obs_list, action_list=action_list)
            exit()
            
        