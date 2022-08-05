import unitree_api_wrapper
import math
import numpy as np
import robot_interface as sdk
from unitree_api_wrapper.format import LowState
from unitree_api_wrapper.vicon_wrapper import ViconTracker 
from pupperfetch.legged_gym.envs.go1.go1_config import Go1FlatCfg
import torch


class Go1Controller:
    def __init__(self, device="cpu", policy_path="policies/go1_flat---22May28_17-36-59_.pt"):
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
        # FR, FL, RR, RL
        self.offset = np.array([[ -0.1, 0.8, -1.5], 
                                [  0.1, 0.8, -1.5], 
                                [ -0.1, 1.0, -1.5], 
                                [  0.1, 1.0, -1.5]])
        if policy_path is not None:
            self.load_policy(policy_path) 
            self.tracker = ViconTracker(object_id='go', host='172.19.0.61:801')
            self.last_action = torch.zeros(12)

    def connect(self):
        self.udp = sdk.UDP(self.LOWLEVEL, 8080, "192.168.123.10", 8007)
        self.safe = sdk.Safety(sdk.LeggedType.Go1)

        self.cmd = sdk.LowCmd()
        self.state = sdk.LowState()
        self.udp.InitCmdData(self.cmd)
    
    def load_policy(self, policy_path):
        self.device = torch.device("cpu")
        self.cfg = Go1FlatCfg()
        self.lin_scale = self.cfg.normalization.obs_scales.lin_vel
        self.ang_scale = self.cfg.normalization.obs_scales.ang_vel
        self.dof_scale = self.cfg.normalization.obs_scales.dof_pos
        self.dof_vel_scale = self.cfg.normalization.obs_scales.dof_vel
        self.cmd_scale = np.array([self.lin_scale, self.lin_scale, self.ang_scale])
        self.action_scale = self.cfg.control.action_scale
        self.model = torch.jit.load(policy_path).to(self.device)

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

        state = self.get_low_state()
        self.safe.PowerProtect(self.cmd, self.state, 1)
        self.udp.SetSend(self.cmd)
        self.udp.Send()
        return state

    def get_low_state(self) -> LowState:
        ## return value is a LowState, see here:
        # https://github.com/unitreerobotics/unitree_legged_sdk/blob/master/include/unitree_legged_sdk/comm.h#L93
        self.udp.Recv()
        self.udp.GetRecv(self.state)
        return self.state
    
    def get_model_obs(self):
        obs = torch.zeros((1, 48))
        
        # Get low level state information
        state = self.get_low_state()
        
        dof_pos = np.array([m.q for m in state.motorState[:12]])
        dof_pos = torch.from_numpy(dof_pos - self.offset.reshape(-1))
        dof_vel = np.array([m.dq for m in state.motorState[:12]])
        dof_vel = torch.from_numpy(dof_vel)
        
        # Get Vicon state
        base_lin_vel, base_ang_vel, projected_gravity = self.tracker.compute_velocity()
        
        # Add command
        commands = torch.Tensor([0.5, 0, 0]) # TODO: implement
        
        obs_dict = {"base_lin_vel": base_lin_vel, 
                    "base_ang_vel": base_ang_vel, 
                    "projected_gravity": projected_gravity,
                    "commands": commands,
                    "dof_pos": dof_pos,
                    "dof_vel": dof_vel, 
                    "last_action": self.last_action}
        
        obs[0, 0:3] = base_lin_vel * self.lin_scale
        obs[0, 3:6] = base_ang_vel * self.ang_scale
        obs[0, 6:9] = projected_gravity
        obs[0, 9:12] = commands * self.cmd_scale
        obs[0, 12:24] = dof_pos * self.dof_scale
        obs[0, 24:36] = dof_vel * self.dof_vel_scale
        obs[0, 36:48] = self.last_action
        return obs, obs_dict
    
    def get_action(self, obs):
        obs = obs.to(self.device)
        with torch.no_grad():
            action = self.model(obs)
            action_scaled = action * self.action_scale
        self.last_action = action
        return action_scaled.squeeze().cpu().numpy()
    
    def control_highlevel(self):
        obs, obs_raw = self.get_model_obs()
        action = self.get_action(obs)
        action = action.reshape((4,3))
        state = self.send_pos_cmd(pos_cmd=action)
        return state, obs, action, obs_raw
    
