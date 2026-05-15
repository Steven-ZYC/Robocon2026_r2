import numpy as np

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle

def get_distance(p1, p2):
    """Euclidean distance between two points (dict with x, y)."""
    return np.sqrt((p1['x'] - p2['x'])**2 + (p1['y'] - p2['y'])**2)

def get_yaw_from_quaternion(q):
    """Extract yaw from geometry_msgs/Quaternion."""
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return np.arctan2(siny_cosp, cosy_cosp)
