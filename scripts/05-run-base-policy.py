import time
import numpy as np
from unitree_api_wrapper.go1_controller import Go1Controller


if __name__ == "__main__":
    controller = Go1Controller()
    controller.connect()
    # TODO: add a loop to send high level commands to the robot