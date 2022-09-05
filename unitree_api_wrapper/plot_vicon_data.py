
import numpy as np
import matplotlib.pyplot as plt


def smooth(y, box_pts):
    box = np.ones(box_pts)/box_pts
    y_smooth = np.convolve(y, box, mode='same')
    return y_smooth


if __name__ == "__main__":
    data = np.load('vicon_data_raw.npz', allow_pickle=True)
    SMOOTH = 10
    
    lin_vel = data['lin_vel']
    ang_vel = data['ang_vel']
    projected_gravity = data['projected_gravity']
    time = data['time']
    
    plt.figure()
    plt.title("Lin vel")
    plt.plot(time, smooth(lin_vel[:,0,0], SMOOTH), label='X')
    plt.plot(time, smooth(lin_vel[:,0,1], SMOOTH), label='Y')
    plt.plot(time, smooth(lin_vel[:,0,2], SMOOTH), label='Z')    
    plt.legend()
    
    plt.figure()
    plt.title("Ang vel")
    plt.plot(time, smooth(ang_vel[:,0,0], SMOOTH), label='X')
    plt.plot(time, smooth(ang_vel[:,0,1], SMOOTH), label='Y')
    plt.plot(time, smooth(ang_vel[:,0,2], SMOOTH), label='Z')    
    plt.legend()
    
    plt.figure()
    plt.title("Projected gravity")
    plt.plot(time, smooth(projected_gravity[:,0,0], SMOOTH), label='X')
    plt.plot(time, smooth(projected_gravity[:,0,1], SMOOTH), label='Y')
    plt.plot(time, smooth(projected_gravity[:,0,2], SMOOTH), label='Z')    
    plt.legend()
    plt.show()