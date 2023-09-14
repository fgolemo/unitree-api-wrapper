import time

import numpy as np
from matplotlib import pyplot as plt

from unitree_api_wrapper.go1_controller import Go1Controller

ctrl = Go1Controller(obs_history=1, with_angvel=True, with_dofvel=True)
ctrl.connect_and_stand(target_p=[0, 0, 0], target_d=[0, 0, 0])
input("press enter to launch when holding the go1")

TIME_PER_CMD = 1000  # ms
dt = 1.0 / 50.0
steps = int(TIME_PER_CMD * dt)
print("steps", steps)

dof_poss = []
dof_vels = []


def record_leggie(leg=0, pos_cmd=[0, 0, 0], kp=[0, 0, 0], kd=[0, 0, 0]):
    for s in range(steps):
        state = ctrl.send_pos_cmd_single_leg(leg=leg, pos_cmd=pos_cmd, kp=kp, kd=kd)
        p, v = ctrl.get_pos_vel_from_state(state)
        dof_poss.append(p)
        dof_vels.append(v)
        time.sleep(dt)


kps = [100, 300, 300]
kds = [5, 8, 8]
for i in range(3):
    kp = [0, 0, 0]
    kp[:i+1] = [60]*(i+1)

    kd = [0, 0, 0]
    kd[:i+1] = [4]*(i+1)

    pos_cmd = [0, 0, 0]
    pos_cmd[i] = 0.5
    record_leggie(pos_cmd=pos_cmd, kp=kp, kd=kd)
    pos_cmd[i] = -0.5
    record_leggie(pos_cmd=pos_cmd, kp=kp, kd=kd)

    pos_cmd[i] = 0.5
    kp[:i+1] = [20]*(i+1)
    kd[:i+1] = [0.5]*(i+1)
    record_leggie(pos_cmd=pos_cmd, kp=kp, kd=kd)
    pos_cmd[i] = -0.5
    record_leggie(pos_cmd=pos_cmd, kp=kp, kd=kd)

    pos_cmd[i] = 0.5
    kp[:i+1] = [kps[i]]*(i+1)
    kd[:i+1] = [kds[i]]*(i+1)
    record_leggie(pos_cmd=pos_cmd, kp=kp, kd=kd)
    pos_cmd[i] = -0.5
    record_leggie(pos_cmd=pos_cmd, kp=kp, kd=kd)

np.savez("real-single-joint-record-230912.npz", dof_pos=dof_poss, dof_vel=dof_vels)

plt.plot(np.arange(len(dof_poss)), np.array(dof_poss)[:, 0])
plt.plot(np.arange(len(dof_poss)), np.array(dof_poss)[:, 1])
plt.plot(np.arange(len(dof_poss)), np.array(dof_poss)[:, 2])
plt.show()
# real joint pos are relative. Are sim obs too?
