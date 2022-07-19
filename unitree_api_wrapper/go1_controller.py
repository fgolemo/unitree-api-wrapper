
import unitree_api_wrapper
import math
import robot_interface as sdk


class Go1Controller:
    def __init__(self):
        self.d = {'FR_0':0, 'FR_1':1, 'FR_2':2,
                'FL_0':3, 'FL_1':4, 'FL_2':5, 
                'RR_0':6, 'RR_1':7, 'RR_2':8, 
                'RL_0':9, 'RL_1':10, 'RL_2':11 }
        self.pos_stop_f  = math.pow(10,9)
        self.vel_stop_f  = 16000.0
        self.HIGHLEVEL = 0xee
        self.LOWLEVEL  = 0xff
        self.kp = [5, 5, 5]
        self.kd = [1, 1, 1]

    def connect(self):
        self.udp = sdk.UDP(self.LOWLEVEL, 8080, "192.168.123.10", 8007)
        self.safe = sdk.Safety(sdk.LeggedType.Go1)
        
        self.cmd = sdk.LowCmd()
        self.state = sdk.LowState()
        self.udp.InitCmdData(self.cmd)
        
    def send_pos_cmd(self, pos_cmd=[[-1.1,  1.8, -2.8],
                                    [ 1.1,  1.8, -2.8],
                                    [-1.1,  1.8, -2.8],
                                    [ 1.1,  1.8, -2.8]], 
                         vel_cmd=[[0,0,0],[0,0,0],[0,0,0],[0,0,0]],
                         torque_cmd=[[0,0,0],[0,0,0],[0,0,0],[0,0,0]]):

        # Iterate legs
        for leg_idx in range(4):
            for motor_idx in range(3):
                self.cmd.motorCmd[leg_idx*3 + motor_idx].q = pos_cmd[leg_idx][motor_idx]
                self.cmd.motorCmd[leg_idx*3 + motor_idx].dq = vel_cmd[leg_idx][motor_idx]
                self.cmd.motorCmd[leg_idx*3 + motor_idx].Kp = self.kp[motor_idx]
                self.cmd.motorCmd[leg_idx*3 + motor_idx].Kd = self.kd[motor_idx]
                self.cmd.motorCmd[leg_idx*3 + motor_idx].tau = torque_cmd[leg_idx][motor_idx]
        
        state = self.get_state()
        self.safe.PowerProtect(self.cmd, self.state, 1)
        self.udp.SetSend(self.cmd)
        self.udp.Send()
        return state
        
    def get_state(self):
        self.udp.Recv()
        self.udp.GetRecv(self.state)
        return self.state
