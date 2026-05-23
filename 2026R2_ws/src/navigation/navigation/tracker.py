import math
import numpy as np
from .utils_math import normalize_angle


class Tracker:
    """Cross-track error PID tracker with heading control.

    Returns forward/lateral/heading components separately so the caller
    can clamp each axis independently (limits read from YAML profile).

    Config keys (passed via config dict):
      k_cte_p:       CTE proportional gain, lateral correction
      k_heading_p:   heading P gain (rad/s per rad of error)
      k_heading_d:   heading D gain for damping (0 = off)
      speed_mps:     cruise speed along the path
    """

    def __init__(self):
        self._prev_heading_error = 0.0

    def compute_pid_cte(self, current_pose, start_wp, end_wp, config):
        """Return (fwd_mps, lat_mps, omega_rad, u_fwd, u_lat).

        fwd_mps:   scalar speed along path AB  (always >= 0)
        lat_mps:   scalar speed toward path    (signed, + = left of path)
        omega_rad: heading correction (rad/s)
        u_fwd:     unit vector along AB (world frame, 2-element np.array)
        u_lat:     unit vector left-perpendicular to AB (world frame)
        """
        A = np.array([start_wp['x'], start_wp['y']])
        B = np.array([end_wp['x'], end_wp['y']])
        P = np.array([current_pose['x'], current_pose['y']])

        AB = B - A
        L2 = np.sum(AB**2)
        if L2 == 0:
            return 0.0, 0.0, 0.0, np.array([1.0, 0.0]), np.array([0.0, 1.0])

        L = np.sqrt(L2)

        # Cross Track Error (signed perpendicular distance from P to AB)
        cte_val = (P[0] - A[0]) * (B[1] - A[1]) - (P[1] - A[1]) * (B[0] - A[0])
        cte = cte_val / L

        kp = config.get('k_cte_p', 0.5)
        cruise_speed = config.get('speed_mps', 0.5)

        u_fwd = AB / L                         # unit forward along path
        u_lat = np.array([-u_fwd[1], u_fwd[0]])  # unit left-perpendicular

        fwd_mps = cruise_speed                  # scalar forward
        lat_mps = kp * cte                      # scalar lateral (signed)

        # --- heading control with optional D damping ---
        target_yaw = end_wp.get('yaw', current_pose['yaw'])
        heading_error = normalize_angle(target_yaw - current_pose['yaw'])

        k_heading = config.get('k_heading_p', 1.0)
        k_heading_d = config.get('k_heading_d', 0.0)

        omega_raw = (
            k_heading * heading_error
            + k_heading_d * (heading_error - self._prev_heading_error)
        )
        self._prev_heading_error = heading_error

        return fwd_mps, lat_mps, omega_raw, u_fwd, u_lat
