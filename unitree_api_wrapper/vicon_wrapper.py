
import time
import numpy as np
from pyvicon.pyvicon import PyVicon, StreamMode
from isaacgym.torch_utils import quat_rotate_inverse
import torch


class ViconTracker:
    def __init__(self, object_id='go', host='172.19.0.61:801'):
        self.gravity_vec = torch.tensor([0,0,-1], dtype=torch.double)
        self.id = object_id
        self.tracker = PyVicon()
        print("frame", self.tracker.get_frame())
        self.tracker.connect(host)
        self.tracker.set_stream_mode(StreamMode.ClientPull)
        self.tracker.connect("172.19.0.61:801")
        print("Connection status : {}".format(self.tracker.is_connected()))
        self.tracker.enable_marker_data()
        self.tracker.enable_segment_data()
        self.tracker.get_frame()
        self.last_time = time.time()
        self.last_position = self.tracker.get_segment_global_translation(self.id, self.id)
        self.last_rotation = self.tracker.get_segment_global_rotation_euler_xyz(self.id, self.id)
        self.last_quat = self.tracker.get_segment_global_quaternion(self.id, self.id) # (w, x, y, z)
        self.last_lin_vel = None
        self.last_ang_vel = None
        self.last_projected_gravity = None
    
    def get_position_rotation(self):
        self.tracker.get_frame()
        pos = self.tracker.get_segment_global_translation(self.id, self.id)
        rot = self.tracker.get_segment_global_rotation_euler_xyz(self.id, self.id)
        q = self.tracker.get_segment_global_quaternion(self.id, self.id)
        return pos, rot, q
    
    def quat_to_torch(self, quat):
        return torch.tensor([quat[1], quat[2], quat[3], quat[0]])

    def pos_to_torch(self, pos):
        return torch.tensor(pos)
    
    def compute_velocity(self):
        pos, rot, quat = self.get_position_rotation()
        dt = time.time() - self.last_time
        self.last_time = time.time()
        lin_vel = self.last_lin_vel
        ang_vel = self.last_ang_vel
        projected_gravity = self.last_projected_gravity
        if pos is not None and rot is not None and quat is not None and \
            self.last_position is not None and self.last_rotation is not None and self.last_quat is not None:
            q = self.quat_to_torch(quat).unsqueeze(0).double()
            lin_vel = quat_rotate_inverse(q, torch.tensor((pos - self.last_position), dtype=torch.double)) / dt
            ang_vel = quat_rotate_inverse(q, torch.tensor((rot - self.last_rotation), dtype=torch.double)) / dt
            projected_gravity = quat_rotate_inverse(q, self.gravity_vec)
            lin_vel = lin_vel / 1000 # convert to m/s
            self.last_lin_vel = lin_vel
            self.last_ang_vel = ang_vel
            self.last_projected_gravity = projected_gravity
        self.last_position = pos
        self.last_rotation = rot
        self.last_quat = quat
        return lin_vel, ang_vel, projected_gravity
        

if __name__ == "__main__":
    tracker = ViconTracker()
    while True:
        pos, rot, quat = tracker.get_position_rotation()
        lin_vel, ang_vel, projected_gravity = tracker.compute_velocity()
        print(lin_vel)
    