
from time import time
import numpy as np
import torch
from pyvicon.pyvicon import PyVicon, StreamMode
from isaacgym.torch_utils import quat_rotate_inverse


class ViconTracker:
    def __init__(self, object_id='go', host='172.19.0.61:801'):
        self.id = object_id
        self.tracker = PyVicon()
        print("frame", self.test.get_frame())
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
        if pos is not None and rot is not None and quat is not None:
            self.last_position = pos
            self.last_rotation = rot
            self.last_quat = quat
            q = self.quat_to_torch(quat)
            lin_vel = quat_rotate_inverse(q, (pos - self.last_position)) / dt
            ang_vel = quat_rotate_inverse(q, (rot - self.last_rotation)) / dt
            return lin_vel, ang_vel
        else:
            return None, None
        
