import time
import numpy as np
import unitree_api_wrapper
from unitree_api_wrapper.go1_controller import Go1Controller


if __name__ == "__main__":
    dt = 1.0 / 50.0
    controller = Go1Controller()
    controller.connect()
    
    for counter in range(int(3.0 / dt)):
        if counter < 50:
            controller.kp = [10, 10, 10]
        elif counter < 100:
            controller.kp = [40, 40, 40]
        else:
            controller.kp = [60, 60, 60]
        base_lin_vel, base_ang_vel, projected_gravity = controller.tracker.compute_velocity()
        controller.send_pos_cmd()
        time.sleep(dt)
    
    print("------------------------------------------------------")
    print("Starting policy")
    print("------------------------------------------------------")
    counter = 0
    obs_list = []
    action_list = []
    while True:
        start = time.perf_counter()
        state, obs, action, obs_raw = controller.control_highlevel()
        obs_list.append(obs)
        action_list.append(action)
        diff = dt - (time.perf_counter() - start)
        if diff > 0:
            time.sleep(diff)
        counter += 1
        
        if counter % 100 == 0:
            print('------------------------------------------------------')
            print("Observation:")
            print("base_lin_vel:", obs_raw['base_lin_vel'])
            print("base_ang_vel:", obs_raw['base_ang_vel'])
            print("projected_gravity:", obs_raw['projected_gravity'])
        
        if counter == 900:
            print("Shutting down soon...")
        if counter == 1000:
            np.savez('go1_data_2022_08_04.npz', obs_list=obs_list, action_list=action_list)
            exit()
            
        