import yaml
import os

class RouteLoader:
    def __init__(self, logger):
        self.logger = logger
        self.waypoints = {}
        self.segments = []
        self.frame_id = "map"

    def load(self, file_path):
        if not os.path.exists(file_path):
            self.logger.error(f"Route file not found: {file_path}")
            return False

        try:
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)
            
            self.frame_id = data.get('frame_id', 'map')
            defaults = data.get('defaults', {})
            action_presets = data.get('action_presets', {})

            # Load Waypoints
            for wp in data.get('waypoints', []):
                wp_id = wp['id']
                # Merge actions from presets if applicable
                actions = []
                for act in wp.get('actions', []):
                    if 'preset' in act:
                        actions.append(action_presets[act['preset']])
                    else:
                        actions.append(act)
                
                self.waypoints[wp_id] = {
                    'pose': wp['pose'],
                    'actions': actions,
                    'pos_tolerance': wp.get('pos_tolerance', defaults.get('pos_tolerance_m', 0.05)),
                    'yaw_tolerance': wp.get('yaw_tolerance', defaults.get('yaw_tolerance_rad', 0.1))
                }

            # Load Segments
            for seg in data.get('segments', []):
                self.segments.append(seg)

            self.logger.info(f"Loaded {len(self.waypoints)} waypoints and {len(self.segments)} segments.")
            return True
        except Exception as e:
            self.logger.error(f"Failed to load route: {e}")
            return False
