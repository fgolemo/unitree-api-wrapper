import time
import numpy as np
import unitree_api_wrapper
from unitree_api_wrapper.go1_controller import Go1Controller


if __name__ == "__main__":
    controller = Go1Controller()
    controller.connect_and_stand()

    motiontime = 0

    ITS = 100
    measurements = []
    for _ in range(ITS):
        # time.sleep(0.002)
        start = time.perf_counter()
        state = controller.send_pos_cmd()
        diff = time.perf_counter() - start
        measurements.append(diff)
        # print(state.motorState[3].q, state.motorState[4].q,state.motorState[5].q)
    mt = np.mean(measurements)
    print(f"mean time {mt}s, mean Hz {1/mt} Hz")
