
import time
import numpy as np
from pyvicon.pyvicon import PyVicon, StreamMode
import torch


def quat_rotate(q, v):
    w, x, y, z = q
    q_vec = q[1:]
    a = v * (2.0 * w ** 2 - 1.0)
    b = np.cross(q_vec, v) * w * 2.0
    c = q_vec * np.dot(q_vec, v) * 2.0
    return a + b + c

def quat_rotate_inverse(q, v):
    w, x, y, z = q
    q_vec = q[1:]
    a = v * (2.0 * w ** 2 - 1.0)
    b = np.cross(q_vec, v) * w * 2.0
    c = q_vec * np.dot(q_vec, v) * 2.0
    return a - b + c

class SlidingWindowSmoothing():
  def __init__(self, n=3):
    self.values = [0]*n
  
  def add(self, value):
    self.values = self.values[1:] + [value]

  def read(self):
    return np.mean(self.values, axis=0)


class ViconTracker:
    def __init__(self, object_id='go1', host='172.19.0.61:801'):
        self.gravity_vec = np.array([0,0,-1])
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
        self.lin_vel_filter = SlidingWindowSmoothing(n=10)
        self.ang_vel_filter = SlidingWindowSmoothing(n=10)
        self.projected_gravity_filter = SlidingWindowSmoothing(n=10)
    
    def get_position_rotation(self):
        self.tracker.get_frame()
        pos = self.tracker.get_segment_global_translation(self.id, self.id)
        rot = self.tracker.get_segment_global_rotation_euler_xyz(self.id, self.id)
        q = self.tracker.get_segment_global_quaternion(self.id, self.id)
        return pos, rot, q
    
    def compute_velocity(self):
        pos, rot, quat = self.get_position_rotation()
        dt = time.time() - self.last_time
        self.last_time = time.time()
        lin_vel = self.last_lin_vel
        ang_vel = self.last_ang_vel
        projected_gravity = self.last_projected_gravity
        if pos is not None and rot is not None and quat is not None and \
            self.last_position is not None and self.last_rotation is not None and self.last_quat is not None:
            lin_vel = quat_rotate_inverse(quat, (pos - self.last_position)) / dt
            ang_vel = quat_rotate_inverse(quat, (rot - self.last_rotation)) / dt
            projected_gravity = quat_rotate_inverse(quat, self.gravity_vec)
            lin_vel = lin_vel / 1000 # convert to m/s
            self.last_lin_vel = lin_vel
            self.last_ang_vel = ang_vel
            self.last_projected_gravity = projected_gravity
            self.lin_vel_filter.add(lin_vel)
            self.ang_vel_filter.add(ang_vel)
            self.projected_gravity_filter.add(projected_gravity)
        self.last_position = pos
        self.last_rotation = rot
        self.last_quat = quat
        return self.lin_vel_filter.read(), self.ang_vel_filter.read(), self.projected_gravity_filter.read()
            

if __name__ == "__main__":
    tracker = ViconTracker()
    
    lin_vel_list = []
    ang_vel_list = []
    projected_gravity_list = []
    time_list = []
    current_time = 0
    print("Starting in 5...")
    time.sleep(5.0)
    print("Starting...")
    start_time = time.time()
    print_count = 0
    while current_time < 15.0:
        pos, rot, quat = tracker.get_position_rotation()
        lin_vel, ang_vel, projected_gravity = tracker.compute_velocity()
        current_time = time.time() - start_time
        if not None in [lin_vel, ang_vel, projected_gravity]:
            lin_vel_list.append(lin_vel)
            ang_vel_list.append(ang_vel)
            projected_gravity_list.append(projected_gravity)
            time_list.append(current_time)
        
        if print_count == 0 and current_time > 5.0:
            print('5 seconds')
            print_count += 1
        if print_count == 1 and current_time > 10.0:
            print('10 seconds')
            print_count += 1
    
    print("Done.")
    np.savez('vicon_data.npz', lin_vel=lin_vel_list, 
                               ang_vel=ang_vel_list,
                               projected_gravity=projected_gravity_list,
                               time=time_list)
    