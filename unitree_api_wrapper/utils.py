import numpy as np


def quat_rotate_inverse(q, v):
    w, x, y, z = q
    q_vec = q[1:]
    a = v * (2.0 * w ** 2 - 1.0)
    b = np.cross(q_vec, v) * w * 2.0
    c = q_vec * np.dot(q_vec, v) * 2.0
    return a - b + c