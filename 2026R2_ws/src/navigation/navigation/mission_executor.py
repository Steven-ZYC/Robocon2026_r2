"""Mission executor that interprets mission YAML and drives all robot subsystems.

Loads a mission file containing waypoints, navigation profiles, actuator
mappings, and a stage list. Executes stages sequentially with support for
navigate, arm, sequential, parallel, conditional, wait, and terminate types.

All numeric values (coordinates, speeds, positions) live in the YAML.
Python code only does interpretation and publishing — no hardcoded values.
"""

import os
import time
import math
import numpy as np
import yaml

from std_msgs.msg import Float32MultiArray

from .tracker import Tracker
from .speed_profiler import SpeedProfiler
from .utils_math import normalize_angle, get_distance


class MissionExecutor:
    """Interprets and executes a mission YAML."""

    def __init__(self, logger, node):
        self.logger = logger
        self.node = node

        # YAML data
        self.waypoints = {}
        self.profiles = {}
        self.actuators = {}
        self.stages = []

        # Navigation components
        self.tracker = Tracker()
        self.speed_profiler = SpeedProfiler()

        # State
        self.current_pose = None
        self.stage_index = 0
        self.phase = 'idle'  # idle, running, done
        self.arrived_counter = 0
        self.arrived_stable_count = 5

        # Sequential sub-stage state
        self._seq_steps = []
        self._seq_step_index = 0

        # Parallel sub-stage state
        self._parallel_active = []

        # Wait state
        self._wait_start = 0.0
        self._wait_duration = 0.0

        # Navigate state
        self._nav_target_wp = None
        self._nav_profile = {}
        self._nav_from_pose = None

        # Conditional state
        self.sensor_cache = {}
        self._condition_result = None

        # Publishers (set after init by global_navigation_node)
        self.pub_driving = None
        self.pub_joint = None   # arm/joint_command
        self.pub_pneu = None    # arm/pneu_command

        # Sensor subscriptions (set after init)
        self._sensor_subs = {}

    # ------------------------------------------------------------------
    # Loading
    # ------------------------------------------------------------------

    def load(self, filepath):
        """Load a mission YAML file.

        Relative paths are resolved against the navigation package share directory
        so that launch-file overrides like mission_file:=routes/red_area.yaml work
        regardless of the process working directory.
        """
        if not os.path.isabs(filepath):
            from ament_index_python.packages import get_package_share_directory
            pkg_dir = get_package_share_directory('navigation')
            filepath = os.path.join(pkg_dir, filepath)
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)

        self.waypoints = data.get('waypoints', {})
        self.profiles = data.get('profiles', {})
        self.actuators = data.get('actuators', {})
        self.stages = data.get('stages', [])
        self.frame_id = data.get('frame_id', 'map')

        self.logger.info(
            f"Mission loaded: {len(self.waypoints)} waypoints, "
            f"{len(self.profiles)} profiles, "
            f"{len(self.actuators)} actuators, "
            f"{len(self.stages)} stages"
        )
        self._validate_stages()

    def _validate_stages(self):
        """Check that all referenced stage IDs, waypoints, and profiles exist."""
        stage_ids = {s['id'] for s in self.stages}
        for s in self.stages:
            sid = s.get('id', '?')
            stype = s.get('type', '?')

            if stype == 'navigate':
                if s.get('to') not in self.waypoints:
                    self.logger.warn(f"[{sid}] waypoint '{s.get('to')}' not found")
                if s.get('profile') not in self.profiles:
                    self.logger.warn(f"[{sid}] profile '{s.get('profile')}' not found")

            elif stype == 'arm':
                for key in s:
                    if key == 'type' or key == 'id':
                        continue
                    if key not in self.actuators:
                        self.logger.warn(f"[{sid}] actuator '{key}' not defined")

            elif stype == 'conditional':
                then_id = s.get('then')
                else_id = s.get('else')
                if then_id and then_id not in stage_ids:
                    self.logger.warn(f"[{sid}] then stage '{then_id}' not found")
                if else_id and else_id not in stage_ids:
                    self.logger.warn(f"[{sid}] else stage '{else_id}' not found")

            elif stype == 'sequential':
                for step in s.get('steps', []):
                    if step.get('type') == 'arm':
                        for key in step:
                            if key == 'type':
                                continue
                            if key not in self.actuators:
                                self.logger.warn(f"[{sid}] actuator '{key}' not defined")

    # ------------------------------------------------------------------
    # Main update loop (called at control rate, e.g. 50Hz)
    # ------------------------------------------------------------------

    def update(self):
        """Advance the mission state machine. Call at control rate."""
        if self.phase == 'idle':
            self.phase = 'running'
            self.stage_index = 0
            self.logger.info("Mission started")

        if self.phase in ('done', 'terminated'):
            return

        if self.stage_index >= len(self.stages):
            self.phase = 'done'
            self.logger.info("Mission complete")
            return

        stage = self.stages[self.stage_index]
        stype = stage.get('type', 'wait')

        if stype == 'navigate':
            self._update_navigate(stage)
        elif stype == 'arm':
            self._execute_arm(stage)
            self._advance_stage()
        elif stype == 'sequential':
            self._update_sequential(stage)
        elif stype == 'parallel':
            self._update_parallel(stage)
        elif stype == 'conditional':
            self._update_conditional(stage)
        elif stype == 'wait':
            self._update_wait(stage)
        elif stype == 'terminate':
            self._execute_terminate()
        else:
            self.logger.warn(f"Unknown stage type '{stype}' at [{stage.get('id', '?')}], skipping")
            self._advance_stage()

    # ------------------------------------------------------------------
    # Stage: navigate
    # ------------------------------------------------------------------

    def _update_navigate(self, stage):
        wp = self.waypoints[stage['to']]
        profile_name = stage.get('profile', 'normal')
        profile = self.profiles.get(profile_name, {})

        end_pose = wp['pose']

        if self.current_pose is None:
            self._pub_zero_driving()
            return

        dist = get_distance(self.current_pose, end_pose)
        yaw_err = abs(normalize_angle(end_pose['yaw'] - self.current_pose['yaw']))

        pos_tol = wp.get('pos_tolerance', 0.05)
        yaw_tol = wp.get('yaw_tolerance', 0.1)

        if dist < pos_tol and yaw_err < yaw_tol:
            self.arrived_counter += 1
        else:
            self.arrived_counter = 0

        if self.arrived_counter >= self.arrived_stable_count:
            self.logger.info(f"Arrived at {stage['to']}")
            self.arrived_counter = 0
            self._pub_zero_driving()
            self._advance_stage()
            return

        # Compute driving command
        start_pose = self._nav_from_pose or self.current_pose
        vx_raw, vy_raw, omega_raw = self.tracker.compute_pid_cte(
            self.current_pose, start_pose, end_pose, {'method': 'pid_cte', 'k_p': 1.5}
        )

        alpha = self.speed_profiler.compute_alpha(
            self.current_pose, start_pose, end_pose,
            {
                'start_radius_m': profile.get('start_radius_m', 0.3),
                'end_radius_m': profile.get('end_radius_m', 0.3),
                'curve': 'cubic_ease',
            }
        )

        vx = vx_raw * alpha
        vy = vy_raw * alpha
        omega = max(alpha, 0.3) * omega_raw

        max_speed = profile.get('speed_mps', 0.6)
        v_mag = math.sqrt(vx ** 2 + vy ** 2)
        if v_mag > max_speed:
            scale = max_speed / v_mag
            vx *= scale
            vy *= scale

        max_omega = profile.get('yaw_rate_rps', 1.5)
        omega = max(-max_omega, min(max_omega, omega))

        self._pub_driving_body(vx, vy, omega)

    def _pub_driving_body(self, vx_body, vy_body, omega):
        """Publish body-frame velocity command to /local_driving."""
        if self.pub_driving is None:
            return

        direction = math.atan2(vy_body, vx_body)
        speed_mps = math.sqrt(vx_body ** 2 + vy_body ** 2)

        msg = Float32MultiArray()
        msg.data = [float(direction), float(speed_mps * 100.0), float(omega)]
        self.pub_driving.publish(msg)

    def _pub_zero_driving(self):
        if self.pub_driving is None:
            return
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0, 0.0]
        self.pub_driving.publish(msg)

    # ------------------------------------------------------------------
    # Stage: arm
    # ------------------------------------------------------------------

    def _execute_arm(self, stage):
        """Translate semantic arm commands → arm/joint_command + arm/pneu_command.

        arm/joint_command uses triplet format:
          [motor_id, position_rad, speed_rad_s, ...]

        motor_id comes directly from the actuator definition in YAML.
        position is looked up from the actuator's positions table.
        speed is read from the actuator's speed field (default 3.0 rad/s).
        """
        num_pneu = sum(1 for a in self.actuators.values() if a['type'] == 'pneumatic')

        joint_triplets = []          # [motor_id, pos, speed, ...]
        pneu_targets = [0.0] * max(num_pneu, 1)

        for name, value in stage.items():
            if name == 'type' or name == 'id':
                continue

            act = self.actuators.get(name)
            if act is None:
                self.logger.warn(f"Unknown actuator '{name}'")
                continue

            if act['type'] == 'motor':
                motor_id = int(act['motor_id'])
                position = float(act['positions'][value])
                speed = float(act.get('speed', 3.0))
                joint_triplets.extend([float(motor_id), position, speed])

            elif act['type'] == 'pneumatic':
                idx = act['index']
                if idx < len(pneu_targets):
                    # states is ordered list: [state0, state1] → 0.0 / 1.0
                    pneu_targets[idx] = float(act['states'].index(value))

        if joint_triplets:
            self._pub_joint_cmd(joint_triplets)
        self._pub_pneu_cmd(pneu_targets)

    def _pub_joint_cmd(self, targets):
        """Publish joint triplets to arm/joint_command.

        targets format: [motor_id, pos_rad, speed_rad_s, ...]
        """
        if self.pub_joint is None:
            return
        msg = Float32MultiArray()
        msg.data = [float(t) for t in targets]
        self.pub_joint.publish(msg)

    def _pub_pneu_cmd(self, targets):
        """Publish pneumatic target array to arm/pneu_command."""
        if self.pub_pneu is None:
            return
        msg = Float32MultiArray()
        msg.data = [float(t) for t in targets]
        self.pub_pneu.publish(msg)

    # ------------------------------------------------------------------
    # Stage: sequential
    # ------------------------------------------------------------------

    def _update_sequential(self, stage):
        steps = stage.get('steps', [])
        if not steps:
            self._advance_stage()
            return

        if self._seq_step_index >= len(steps):
            self._seq_step_index = 0
            self._advance_stage()
            return

        step = steps[self._seq_step_index]
        stype = step.get('type', 'wait')

        if stype == 'arm':
            self._execute_arm(step)
            self._seq_step_index += 1
        elif stype == 'wait':
            self._update_wait(step)
        else:
            self.logger.warn(f"Unknown sequential step type '{stype}', skipping")
            self._seq_step_index += 1

    # ------------------------------------------------------------------
    # Stage: parallel
    # ------------------------------------------------------------------

    def _update_parallel(self, stage):
        actions = stage.get('actions', [])
        if not actions:
            self._advance_stage()
            return

        # Fire all actions on first call
        if not self._parallel_active:
            for action in actions:
                atype = action.get('type')
                if atype == 'arm':
                    self._execute_arm(action)
            self._parallel_active = actions
            self._parallel_start_time = time.time()

        # Wait for all to "complete" (arms are instantaneous, so done immediately)
        if stage.get('wait_until') == 'all_complete':
            self._parallel_active = []
            self._advance_stage()

    # ------------------------------------------------------------------
    # Stage: conditional
    # ------------------------------------------------------------------

    def _update_conditional(self, stage):
        cond = stage.get('condition', {})
        field = cond.get('field')
        op = cond.get('op', 'gt')
        threshold = cond.get('value', 0)

        # Try to get sensor value from cache
        sensor_data = self.sensor_cache.get(cond.get('topic'))
        if sensor_data is None:
            self.logger.warn(f"No sensor data for condition on {cond.get('topic')}")
            return

        actual = sensor_data.get(field, 0)

        if op == 'gt':
            result = actual > threshold
        elif op == 'lt':
            result = actual < threshold
        elif op == 'gte':
            result = actual >= threshold
        elif op == 'lte':
            result = actual <= threshold
        else:
            self.logger.warn(f"Unknown op '{op}'")
            return

        target_id = stage['then'] if result else stage['else']
        self._jump_to(target_id)

    def _jump_to(self, stage_id):
        """Jump to a stage by ID."""
        for i, s in enumerate(self.stages):
            if s.get('id') == stage_id:
                self.stage_index = i
                self._seq_step_index = 0
                self._parallel_active = []
                self._wait_duration = 0.0
                self.logger.info(f"Jumped to [{stage_id}]")
                return
        self.logger.warn(f"Stage '{stage_id}' not found, skipping")

    # ------------------------------------------------------------------
    # Stage: wait
    # ------------------------------------------------------------------

    def _update_wait(self, stage):
        duration = stage.get('duration_s', 0.0)
        if self._wait_duration == 0.0:
            self._wait_start = time.time()
            self._wait_duration = duration

        if time.time() - self._wait_start >= self._wait_duration:
            self._wait_duration = 0.0
            if self._seq_step_index > 0:
                # In a sequential, advance to next step
                self._seq_step_index += 1
            else:
                self._advance_stage()

    # ------------------------------------------------------------------
    # Stage: terminate
    # ------------------------------------------------------------------

    def _execute_terminate(self):
        self._pub_zero_driving()
        # Stop all arm motors (triplet format: motor_id, pos=0, speed=0)
        joint_triplets = []
        for act in self.actuators.values():
            if act.get('type') == 'motor':
                joint_triplets.extend([float(act['motor_id']), 0.0, 0.0])
        if joint_triplets and self.pub_joint is not None:
            msg = Float32MultiArray()
            msg.data = joint_triplets
            self.pub_joint.publish(msg)
        if self.pub_pneu is not None:
            msg = Float32MultiArray()
            msg.data = [0.0, 0.0, 0.0]
            self.pub_pneu.publish(msg)
        self.phase = 'terminated'
        self.logger.info("Mission terminated")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _advance_stage(self):
        self.stage_index += 1

    def set_pose(self, pose):
        self.current_pose = pose

    def reset(self):
        self.stage_index = 0
        self.phase = 'idle'
        self.arrived_counter = 0
        self._seq_step_index = 0
        self._parallel_active = []
        self._wait_duration = 0.0
