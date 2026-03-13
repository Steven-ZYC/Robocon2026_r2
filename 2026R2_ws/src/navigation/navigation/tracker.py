import numpy as np
from .utils_math import normalize_angle

class Tracker:
    def __init__(self):
        pass

    def compute_pid_cte(self, current_pose, start_wp, end_wp, config):
        """
        Compute target velocity vector in World Frame using Cross-Track Error PID.
        """
        A = np.array([start_wp['x'], start_wp['y']])
        B = np.array([end_wp['x'], end_wp['y']])
        P = np.array([current_pose['x'], current_pose['y']])

        AB = B - A
        L2 = np.sum(AB**2)
        if L2 == 0:
            return 0.0, 0.0, 0.0

        # Progress t along AB
        t = np.clip(np.dot(P - A, AB) / L2, 0.0, 1.0)
        projection = A + t * AB

        # Cross Track Error (distance from P to AB)
        # Vector from projection to P
        cte_vec = P - projection
        # We need the sign. Let's use 2D cross product with AB
        # error = (Px-Ax)(By-Ay) - (Py-Ay)(Bx-Ax)
        cte_val = (P[0]-A[0])*(B[1]-A[1]) - (P[1]-A[1])*(B[0]-A[0])
        cte = cte_val / np.sqrt(L2)

        # PID parameters from segment config
        kp = config.get('k_p', 1.0)
        # For simplicity, we implement P-controller for CTE lateral correction
        # v_forward = cruise_speed
        # v_lateral = -kp * cte
        
        cruise_speed = config.get('speed_mps', 0.5)
        
        # Unit vector for forward and lateral
        unit_forward = AB / np.sqrt(L2)
        unit_lateral = np.array([-unit_forward[1], unit_forward[0]])

        # Combine
        v_vec = cruise_speed * unit_forward - (kp * cte) * unit_lateral
        
        vx_raw = v_vec[0]
        vy_raw = v_vec[1]

        # Heading error
        target_yaw = end_wp.get('yaw', current_pose['yaw'])
        heading_error = normalize_angle(target_yaw - current_pose['yaw'])
        
        # Simple P control for rotation
        k_heading = 2.0 # Default gain
        omega_raw = k_heading * heading_error
        
        return vx_raw, vy_raw, omega_raw
