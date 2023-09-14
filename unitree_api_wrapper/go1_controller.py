import time

import unitree_api_wrapper
import math
import numpy as np
import robot_interface as sdk  # if you import `unitree_api_wrapper` above, then that should take of that
from unitree_api_wrapper.format import LowState, LowCmd
from unitree_api_wrapper.utils import quat_rotate_inverse

import torch


class Go1CfgScales:
    lin_vel = 2.0
    ang_vel = 0.25
    dof_pos = 1.0
    dof_vel = 0.05
    action = 0.25

    def __init__(self):
        super().__init__()
        # self.cmd_scale = np.array([self.lin_vel, self.lin_vel, self.ang_vel])
        # ang_vel scaling actually seems to be 1 when using heading cmd
        self.cmd_scale = np.array([self.lin_vel, self.lin_vel, 1])


class Go1Controller:
    def __init__(
        self,
        device="cpu",
        policy_path=None,
        with_linvel=False,
        with_angvel=False,
        use_vicon=False,
        obs_history=1,
        with_dofvel=True,
    ):
        self.d = {
            "FR_0": 0,
            "FR_1": 1,
            "FR_2": 2,
            "FL_0": 3,
            "FL_1": 4,
            "FL_2": 5,
            "RR_0": 6,
            "RR_1": 7,
            "RR_2": 8,
            "RL_0": 9,
            "RL_1": 10,
            "RL_2": 11,
        }
        self.pos_stop_f = math.pow(10, 9)
        self.vel_stop_f = 16000.0
        self.HIGHLEVEL = 0xEE
        self.LOWLEVEL = 0xFF
        self.kp = [10, 10, 10]
        self.kd = [4, 4, 4]
        self.cfg = Go1CfgScales()
        self.dof_len = 12
        self.dof_start = 6
        self.with_dofvel = with_dofvel
        obs_len = 43

        self.with_linvel = with_linvel
        if with_linvel:
            obs_len += 3
            self.dof_start += 3

        self.with_angvel = with_angvel
        if with_angvel:
            obs_len += 3
            self.dof_start += 3

        if not with_dofvel:
            obs_len -= 12

        self.obs = torch.zeros((1, obs_len, obs_history))  # batch, obs, history
        # FR, FL, RR, RL
        # rest pos - copied from legged_gym
        self.offset = np.array([[-0.1, 0.8, -1.5], [0.1, 0.8, -1.5], [-0.1, 1.0, -1.5], [0.1, 1.0, -1.5]])
        self.dt = 1 / 200  # run a 200Hz control loop
        self.decimation = 4  # but repeat action 4 times, so effective control freq = 50Hz

        if policy_path is not None:
            self.device = torch.device("cpu")
            self.model = torch.jit.load(unitree_api_wrapper.get_policy_path(policy_path)).to(self.device)
            self.last_action = np.zeros(12)
            self.tracker = None
            if use_vicon:
                from unitree_api_wrapper.vicon_wrapper import ViconTracker

                self.tracker = ViconTracker(object_id="go", host="172.19.0.61:801")

    def connect_and_stand(self, target_p=[60, 60, 60], target_d=[4, 4, 4]):
        self.udp = sdk.UDP(self.LOWLEVEL, 8080, "192.168.123.10", 8007)
        self.safe = sdk.Safety(sdk.LeggedType.Go1)

        self.cmd: LowCmd = sdk.LowCmd()
        self.state: LowState = sdk.LowState()
        self.udp.InitCmdData(self.cmd)

        dt = 1.0 / 50.0
        # slowly ramping up kp over 3 secs
        for counter in range(int(3.0 / dt)):
            if counter < 50:
                print("p value ramp-up, stage 1")
                self.kp = [int(p / 4) for p in target_p]
                self.kd = [int(d / 4) for d in target_d]
            elif counter < 100:
                print("p value ramp-up, stage 2")
                self.kp = [int(p / 2) for p in target_p]
                self.kd = [int(d / 2) for d in target_d]
            else:
                print("p value ramp-up, stage 3")
                self.kp = [int(p / 1) for p in target_p]
                self.kd = [int(d / 1) for d in target_d]

            self.send_pos_cmd()  # send zero command, i.e. rest pos
            time.sleep(dt)

    def send_pos_cmd(
        self,
        pos_cmd=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        vel_cmd=[[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
        torque_cmd=[[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]],
    ):
        pos_cmd = np.array(pos_cmd) + self.offset

        # Iterate legs
        for leg_idx in range(4):
            for motor_idx in range(3):
                # self.cmd.motorCmd[leg_idx * 3 + motor_idx].mode = 1
                self.cmd.motorCmd[leg_idx * 3 + motor_idx].q = pos_cmd[leg_idx][motor_idx]
                self.cmd.motorCmd[leg_idx * 3 + motor_idx].dq = vel_cmd[leg_idx][motor_idx]
                self.cmd.motorCmd[leg_idx * 3 + motor_idx].Kp = self.kp[motor_idx]
                self.cmd.motorCmd[leg_idx * 3 + motor_idx].Kd = self.kd[motor_idx]
                self.cmd.motorCmd[leg_idx * 3 + motor_idx].tau = torque_cmd[leg_idx][motor_idx]

        self.state = self.get_low_state()
        self.safe.PowerProtect(self.cmd, self.state, 1)
        self.udp.SetSend(self.cmd)
        self.udp.Send()
        return self.state

    def send_pos_cmd_single_leg(
        self,
        leg=0,  # 0=FR, 1=FL, 2=RR, 3=RL
        pos_cmd=[0.0, 0.0, 0.0],
        vel_cmd=[0, 0, 0],
        torque_cmd=[0, 0, 0],
        kp=[0, 0, 0],
        kd=[0, 0, 0],
    ):
        pos_cmd_tmp = np.copy(self.offset)
        pos_cmd_tmp[leg] += np.array(pos_cmd)

        # Iterate legs
        for motor_idx in range(3):
            # self.cmd.motorCmd[leg_idx * 3 + motor_idx].mode = 1
            self.cmd.motorCmd[leg * 3 + motor_idx].q = pos_cmd_tmp[leg][motor_idx]
            self.cmd.motorCmd[leg * 3 + motor_idx].dq = vel_cmd[motor_idx]
            self.cmd.motorCmd[leg * 3 + motor_idx].Kp = kp[motor_idx]
            self.cmd.motorCmd[leg * 3 + motor_idx].Kd = kd[motor_idx]
            self.cmd.motorCmd[leg * 3 + motor_idx].tau = torque_cmd[motor_idx]

        self.state = self.get_low_state()
        self.safe.PowerProtect(self.cmd, self.state, 1)
        self.udp.SetSend(self.cmd)
        self.udp.Send()
        return self.state

    def get_low_state(self) -> LowState:
        self.udp.Recv()
        self.udp.GetRecv(self.state)
        return self.state

    def get_pos_vel_from_state(self, state):
        dof_pos = np.array([m.q for m in state.motorState[:12]])
        dof_pos = dof_pos - self.offset.reshape(-1)  # subtract resting position
        dof_vel = np.array([m.dq for m in state.motorState[:12]])
        return dof_pos, dof_vel

    def get_obs(self, command=torch.zeros(4)):
        # self.obs.fill_(0)  # zero out the obs before we do anything

        # Get low level state information
        state = self.get_low_state()

        dof_pos = np.array([m.q for m in state.motorState[:12]])
        dof_pos = torch.from_numpy(dof_pos - self.offset.reshape(-1))  # subtract resting position
        dof_vel = np.array([m.dq for m in state.motorState[:12]])
        dof_vel = torch.from_numpy(dof_vel)

        if self.tracker is not None:
            # Get Vicon state
            base_lin_vel, base_ang_vel, projected_gravity = self.tracker.compute_velocity()
        else:
            base_quat = np.array(state.imu.quaternion)
            projected_gravity = quat_rotate_inverse(base_quat, np.array([0, 0, -1]))
            base_ang_vel = np.array(state.imu.gyroscope)

        obs_out = []
        if self.with_linvel:
            obs_out.append(base_lin_vel * self.cfg.lin_vel)
        if self.with_angvel:
            obs_out.append(base_ang_vel * self.cfg.ang_vel)

        obs_out.append(projected_gravity)
        obs_out.append(command[:-1] * self.cfg.cmd_scale)  # lin vel/lin vel/ang vel
        obs_out.append(dof_pos * self.cfg.dof_pos)
        if self.with_dofvel:
            obs_out.append(dof_vel * self.cfg.dof_vel)
        obs_out.append(self.last_action)
        obs_out.append(command[-1:] * 1)  # height cmd

        # breakpoint()

        o = np.concatenate(obs_out).ravel()

        self.obs = self.obs.roll(-1, dims=2)
        self.obs[0, :, -1] = torch.from_numpy(o)

        return self.obs

    def clip_action_rate(self, last_obs, action, clip_max=0.3):

        action_diff = action - last_obs[0, self.dof_start : self.dof_start + self.dof_len]
        # print ("action diff", action_diff)
        action_diff = torch.clip(action_diff, -clip_max, clip_max)
        action_out = last_obs[0, self.dof_start : self.dof_start + self.dof_len] + action_diff
        return action_out

    def get_action(self, obs):
        # important: the last action is the unscaled action but the thing that runs on the robot is the scaled action
        obs = obs.to(self.device).view(1, -1)  # flatten the history obs
        with torch.no_grad():
            action = self.model(obs)

        action_scaled = action * self.cfg.action
        action_out = self.clip_action_rate(obs, action_scaled)

        self.last_action = action[0]
        return action_out.squeeze().cpu().numpy()

    def control_highlevel(self, cmd):
        obs = self.get_obs(cmd)
        action = self.get_action(obs)
        action = action.reshape((4, 3))
        for _ in range(self.decimation):
            state = self.send_pos_cmd(pos_cmd=action)
            time.sleep(self.dt)  # important

        return state, obs, action
