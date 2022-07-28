import numpy as np
import torch
import os
from pupperfetch.legged_gym.envs.go1.go1_config import Go1FlatCfg

# TODO: get these from vicon
CURRENT_LIN_VEL = np.zeros((3))
CURRENT_ANG_VEL = np.zeros((3))

# TODO: get from IMU
CURRENT_GRAV = np.zeros((3))

# TODO control this with TKinter UI
CURRENT_COMMAND = np.zeros((3))

cfg = Go1FlatCfg()
dev = torch.device("cpu")  #
model = torch.jit.load("../policies/go1_flat---22May28_17-36-59_.pt").to(dev)
input_ = torch.zeros((32, 48), dtype=torch.float32, device=dev)
output = model(input_)
print(output.shape)
# obs
# self.base_lin_vel * self.obs_scales.lin_vel,  # N x 3, 0-2
# self.base_ang_vel * self.obs_scales.ang_vel,  # N x 3, 3-5
# self.projected_gravity,  # N x 3, 6-8
# self.commands[:, :3] * self.commands_scale[:3],  # N x 3, 9-11
# ((self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos),  # N x 12 (w/o arm) or 15 (arm)
# (self.dof_vel * self.obs_scales.dof_vel),  # N x 12 (w/o arm) or 15 (arm)
# self.actions,  # N x 12 (w/o arm) or 15 (arm),


def get_gravity():
    # TODO read IMU (euler angle, `imu.rpy`)
    return CURRENT_GRAV


lin_scale = cfg.normalization.obs_scales.lin_vel
ang_scale = cfg.normalization.obs_scales.ang_vel
cmd_scale = np.array([lin_scale, lin_scale, ang_scale])


def get_obs():
    lin_vel = CURRENT_LIN_VEL * lin_scale
    ang_vel = CURRENT_ANG_VEL * ang_scale
    orientation = get_gravity()
    cmds = CURRENT_COMMAND * cmd_scale

    # TODO from here
    dof_pos = (self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos)
