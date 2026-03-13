import numpy as np

class SpeedProfiler:
    def __init__(self):
        pass

    def ease(self, s):
        """Cubic ease function: 3s^2 - 2s^3."""
        s = np.clip(s, 0.0, 1.0)
        return 3 * (s**2) - 2 * (s**3)

    def compute_alpha(self, current_pose, start_wp, end_wp, profiler_config):
        """
        Compute the speed scale factor alpha based on distance to start/end.
        """
        A = np.array([start_wp['x'], start_wp['y']])
        B = np.array([end_wp['x'], end_wp['y']])
        P = np.array([current_pose['x'], current_pose['y']])

        AB = B - A
        L = np.linalg.norm(AB)
        if L == 0:
            return 0.0

        # Distance from start along segment
        t = np.clip(np.dot(P - A, AB) / (L**2), 0.0, 1.0)
        d_from = t * L
        d_to = L - d_from

        R_start = profiler_config.get('start_radius_m', 0.1)
        R_end = profiler_config.get('end_radius_m', 0.1)

        # Alpha from start ease
        alpha_start = 1.0
        if R_start > 0:
            alpha_start = self.ease(d_from / R_start)

        # Alpha from end ease
        alpha_end = 1.0
        if R_end > 0:
            alpha_end = self.ease(d_to / R_end)

        # Result is the minimum of both
        alpha = min(alpha_start, alpha_end)
        return alpha
