"""Mission executor that interprets mission YAML and drives all robot subsystems.

Loads a mission file containing waypoints, navigation profiles, actuator
mappings, and a stage list. Executes stages sequentially with support for
navigate, arm, sequential, parallel, conditional, stop_chassis,
red_area_weapon_cycle, wait, and terminate types.

All numeric values (coordinates, speeds, positions) live in the YAML.
Python code only does interpretation and publishing — no hardcoded values.
"""

import os
import time
import math
import numpy as np
import yaml

from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Pose2D

from .tracker import Tracker
from .speed_profiler import SpeedProfiler
from .utils_math import normalize_angle, get_distance


TRACKER_K_CTE_P = 0.5
DEFAULT_K_P_X = 0.5
DEFAULT_K_P_Y = 0.5
DEFAULT_K_I_X = 0.0
DEFAULT_K_I_Y = 0.0
DEFAULT_K_D_X = 0.0
DEFAULT_K_D_Y = 0.0
TRACKER_K_HEADING_P = 1.0
TRACKER_K_HEADING_D = 0.0
TRACKER_MAX_SPEED_MPS = 0.5
DEFAULT_MAX_LATERAL_MPS = 0.05
DEFAULT_MIN_SPEED_SCALE = 0.20
DEFAULT_MISSION_ANGLE_UNIT = 'rad'
SUPPORTED_MISSION_ANGLE_UNITS = {'deg', 'rad'}


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
        self.angle_unit = DEFAULT_MISSION_ANGLE_UNIT

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
        self._active_nav_stage_id = None
        self._nav_start_time = 0.0

        # XY split PID integral / derivative state
        self._xy_error_integral_x = 0.0
        self._xy_error_integral_y = 0.0
        self._xy_error_prev_x = 0.0
        self._xy_error_prev_y = 0.0
        self._xy_pid_initialized = False

        # Conditional state
        self.sensor_cache = {}
        self._condition_result = None

        # Weapon head pickup state
        self._weapon_stage_id = None
        self._weapon_state = 'idle'
        self._weapon_slot_index = 0
        self._weapon_pickup_step_index = 0
        self._weapon_step_target_pose = None
        self._weapon_scan_start_pose = None
        self._weapon_scan_start_time = 0.0
        self._weapon_wait_start = 0.0
        self._weapon_sequence_action_index = -1
        self._weapon_warned_missing_ir = False
        self._weapon_warned_ir_timeout = False
        self._weapon_ir_timeout_start = 0.0

        # Red Area weapon cycle state
        self._red_cycle_stage_id = None
        self._red_cycle_state = 'idle'
        self._red_cycle_slot_index = 0
        self._red_cycle_success_count = 0
        self._red_cycle_retry_count = 0
        self._red_cycle_sequence_name = None
        self._red_cycle_sequence_index = 0
        self._red_cycle_sequence_after = None
        self._red_cycle_wait_start = 0.0
        self._red_cycle_target_pose = None
        self._red_cycle_scan_start_pose = None
        self._red_cycle_scan_start_time = 0.0
        self._red_cycle_warned_missing_torque = False

        # Arm state tracking (deterministic FSM: every stage defines full robot state)
        self._current_arm_state = {}           # {actuator_name: semantic_value}
        self._last_arm_publish_time = 0.0
        self._arm_keepalive_interval_s = 0.1   # 10Hz default
        self._arm_keepalive_enabled = True
        self._active_stage_id = None           # for stage-entry detection

        # Publishers (set after init by global_navigation_node)
        self.pub_driving = None
        self.pub_joint = None   # arm/joint_navigation
        self.pub_pneu = None    # arm/pneu_navigation
        self.pub_target_pose = None  # /global_nav/target_pose

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

        try:
            with open(filepath, 'r') as f:
                raw_text = f.read()
        except FileNotFoundError:
            self.logger.error(f"Mission file not found: {filepath}")
            raise
        except OSError as e:
            self.logger.error(f"Cannot read mission file: {filepath} ({e})")
            raise

        try:
            data = yaml.safe_load(raw_text) or {}
        except yaml.YAMLError as e:
            self.logger.error(f"YAML parse error in mission file: {filepath}")
            if hasattr(e, 'problem_mark') and e.problem_mark is not None:
                m = e.problem_mark
                lines = raw_text.split('\n')
                ctx_start = max(m.line - 2, 0)
                ctx_end = min(m.line + 3, len(lines))
                self.logger.error(
                    f"  Line {m.line + 1}, column {m.column + 1}: {e.problem}"
                )
                self.logger.error(f"  Context ({ctx_start + 1}-{ctx_end}):")
                for i in range(ctx_start, ctx_end):
                    marker = '>>>' if i == m.line else '   '
                    self.logger.error(f"  {marker} {i + 1}: {lines[i]}")
            else:
                self.logger.error(f"  {e}")
            raise

        # 显式类型校验，防 YAML 静默解析为意外类型
        if not isinstance(data.get('stages', []), list):
            self.logger.error(
                f"Mission 'stages' must be a list, got {type(data.get('stages')).__name__}. "
                f"Check YAML indentation — a missing leading '- ' often causes this."
            )
            raise ValueError("stages is not a list")
        if not isinstance(data.get('waypoints', {}), dict):
            self.logger.error(f"Mission 'waypoints' must be a dict, got {type(data.get('waypoints')).__name__}")
            raise ValueError("waypoints is not a dict")
        if not isinstance(data.get('profiles', {}), dict):
            self.logger.error(f"Mission 'profiles' must be a dict, got {type(data.get('profiles')).__name__}")
            raise ValueError("profiles is not a dict")
        if not isinstance(data.get('actuators', {}), dict):
            self.logger.error(f"Mission 'actuators' must be a dict, got {type(data.get('actuators')).__name__}")
            raise ValueError("actuators is not a dict")

        self.angle_unit = self._read_angle_unit(data)
        self.waypoints = self._normalize_waypoints(
            data.get('waypoints', {}),
            self.angle_unit,
        )
        self.profiles = data.get('profiles', {})
        self.actuators = data.get('actuators', {})
        self.stages = data.get('stages', [])
        self.frame_id = data.get('frame_id', 'map')

        self.logger.info(
            f"Mission loaded: {len(self.waypoints)} waypoints, "
            f"{len(self.profiles)} profiles, "
            f"{len(self.actuators)} actuators, "
            f"{len(self.stages)} stages, "
            f"angle_unit={self.angle_unit}"
        )
        self._validate_stages()

    def _read_angle_unit(self, data):
        """Read the mission YAML angle unit used by waypoint yaw values."""
        raw_unit = data.get('angle_unit', data.get('yaw_unit', DEFAULT_MISSION_ANGLE_UNIT))
        unit = str(raw_unit).strip().lower()
        aliases = {
            'degree': 'deg',
            'degrees': 'deg',
            'radian': 'rad',
            'radians': 'rad',
        }
        unit = aliases.get(unit, unit)

        if unit not in SUPPORTED_MISSION_ANGLE_UNITS:
            self.logger.warn(
                f"Unknown mission angle_unit '{unit}', "
                f"falling back to '{DEFAULT_MISSION_ANGLE_UNIT}'"
            )
            return DEFAULT_MISSION_ANGLE_UNIT
        return unit

    def _angle_to_rad(self, value, angle_unit):
        """Convert a YAML angle value to radians for internal navigation math."""
        numeric_value = float(value)
        if angle_unit == 'deg':
            return math.radians(numeric_value)
        return numeric_value

    def _normalize_waypoints(self, waypoints, angle_unit):
        """Convert waypoint yaw/tolerance fields from mission units to radians."""
        normalized = {}
        for name, waypoint in waypoints.items():
            wp = dict(waypoint)
            pose = dict(wp.get('pose', {}))

            if 'yaw' in pose:
                pose['yaw'] = self._angle_to_rad(pose['yaw'], angle_unit)
            wp['pose'] = pose

            if 'yaw_tolerance_deg' in wp:
                wp['yaw_tolerance'] = math.radians(float(wp['yaw_tolerance_deg']))
            elif 'yaw_tolerance' in wp:
                wp['yaw_tolerance'] = self._angle_to_rad(wp['yaw_tolerance'], angle_unit)

            normalized[name] = wp
        return normalized

    # Known-key whitelists for YAML validation
    # 用于 _validate_stages() 中检测拼写错误 / 不存在的字段
    _KNOWN_WAYPOINT_KEYS = {'pose', 'pos_tolerance', 'yaw_tolerance', 'yaw_tolerance_deg'}
    _KNOWN_POSE_KEYS = {'x', 'y', 'yaw'}
    _KNOWN_PROFILE_KEYS = {
        'speed_mps', 'yaw_rate_rps', 'start_radius_m', 'end_radius_m',
        'min_speed_scale', 'curve', 'k_cte_p', 'k_heading_p', 'k_heading_d',
        'max_lateral_mps', 'k_p_x', 'k_p_y', 'k_i_x', 'k_i_y', 'k_d_x', 'k_d_y',
        'xy_integral_max', 'max_body_x_mps', 'max_body_y_mps',
    }
    _KNOWN_ACTUATOR_MOTOR_KEYS = {'type', 'motor_id', 'speed', 'positions'}
    _KNOWN_ACTUATOR_PNEU_KEYS = {'type', 'states'}
    _STAGE_COMMON_KEYS = {'id', 'type', 'arm'}
    _KNOWN_CHASSIS_KEYS = {'to', 'profile', 'timeout_s', 'torque_arrival', 'stop'}
    _KNOWN_TORQUE_ARRIVAL_KEYS = {
        'topic', 'field', 'op', 'abs_threshold_nm', 'threshold_nm',
        'value', 'max_age_s', 'min_elapsed_s', 'stamp_field',
    }
    _KNOWN_CONDITION_KEYS = {'topic', 'field', 'op', 'value', 'max_age_s', 'stamp_field'}
    _KNOWN_SCAN_KEYS = {'direction_rad', 'speed_mps', 'max_distance_m', 'timeout_s'}
    _KNOWN_STEP_KEYS = {'profile', 'settle_s', 'pos_tolerance', 'yaw_tolerance'}
    _KNOWN_MICRO_SWEEP_KEYS = {
        'direction_rad', 'back_distance_m', 'forward_distance_m', 'speed_mps',
        'timeout_s', 'max_distance_m', 'profile', 'pos_tolerance', 'yaw_tolerance',
    }
    _KNOWN_SEARCH_KEYS = _KNOWN_MICRO_SWEEP_KEYS | {'mode'}
    _KNOWN_VERIFY_IR_KEYS = {
        'type', 'label', 'expected', 'on_true', 'on_false', 'on_match', 'on_mismatch',
    }

    def _warn_unknown(self, label, data, known):
        """对 data 中不在 known 集合里的 key 各发一次 warn。"""
        if not isinstance(data, dict):
            return
        for key in data:
            if key not in known:
                self.logger.warn(
                    f"{label}: unknown key '{key}' "
                    f"(valid: {', '.join(sorted(known))})"
                )

    def _validate_stages(self):
        """Check that all referenced IDs / waypoints / profiles exist,
        AND warn about unrecognized YAML keys in every block."""

        actuator_names = set(self.actuators.keys())
        stage_ids = {s['id'] for s in self.stages}

        # ---- 0. Validate waypoints, profiles, actuators at block level ----
        for name, wp in self.waypoints.items():
            self._warn_unknown(f"waypoint '{name}'", wp, self._KNOWN_WAYPOINT_KEYS)
            pose = wp.get('pose')
            if isinstance(pose, dict):
                self._warn_unknown(f"waypoint '{name}'.pose", pose, self._KNOWN_POSE_KEYS)

        for name, prof in self.profiles.items():
            self._warn_unknown(f"profile '{name}'", prof, self._KNOWN_PROFILE_KEYS)

        for name, act in self.actuators.items():
            if act.get('type') == 'motor':
                self._warn_unknown(f"actuator '{name}'", act, self._KNOWN_ACTUATOR_MOTOR_KEYS)
            elif act.get('type') == 'pneumatic':
                self._warn_unknown(f"actuator '{name}'", act, self._KNOWN_ACTUATOR_PNEU_KEYS)

        # ---- 1. Stage-level unknown-key + reference checks ----
        for s in self.stages:
            sid = s.get('id', '?')
            stype = s.get('type', '?')

            # -- stage-level unknown keys --
            # Per declared type: known = common + type-specific extra
            if stype == 'navigate':
                self._warn_unknown(f"[{sid}]", s, self._STAGE_COMMON_KEYS | {'to', 'profile', 'timeout_s', 'torque_arrival'})
            elif stype == 'arm':
                self._warn_unknown(f"[{sid}]", s, self._STAGE_COMMON_KEYS | actuator_names)
            elif stype == 'action':
                self._warn_unknown(f"[{sid}]", s, self._STAGE_COMMON_KEYS | {'chassis'})
            elif stype in ('condition', 'conditional'):
                self._warn_unknown(f"[{sid}]", s, self._STAGE_COMMON_KEYS | {'condition', 'then', 'else'})
            elif stype == 'verify_ir':
                self._warn_unknown(f"[{sid}]", s, self._STAGE_COMMON_KEYS | self._KNOWN_VERIFY_IR_KEYS)
            elif stype == 'wait':
                self._warn_unknown(f"[{sid}]", s, self._STAGE_COMMON_KEYS | {'duration_s'})
            elif stype == 'stop_chassis':
                self._warn_unknown(f"[{sid}]", s, self._STAGE_COMMON_KEYS)
            elif stype == 'sequential':
                self._warn_unknown(f"[{sid}]", s, self._STAGE_COMMON_KEYS | {'steps'})
            elif stype == 'parallel':
                self._warn_unknown(f"[{sid}]", s, self._STAGE_COMMON_KEYS | {'actions', 'wait_until'})
            elif stype == 'weapon_head_pickup':
                self._warn_unknown(f"[{sid}]", s, self._STAGE_COMMON_KEYS | {
                    'search_mode', 'ir_topic', 'ir_field', 'ir_timeout_s',
                    'slot_count', 'slot_spacing_m', 'on_miss',
                    'scan', 'step', 'micro_sweep', 'pickup_sequence',
                })
            elif stype == 'red_area_weapon_cycle':
                self._warn_unknown(f"[{sid}]", s, self._STAGE_COMMON_KEYS - {'arm'} | {
                    'start_waypoint', 'slot_count', 'slot_spacing_m', 'slot_direction_rad',
                    'slot_profile', 'target_success_count', 'max_retry_per_slot',
                    'ir_topic', 'ir_field', 'ir_timeout_s',
                    'docking_torque_topic', 'docking_torque_field', 'docking_torque_abs_threshold_nm',
                    'slot_pos_tolerance', 'slot_yaw_tolerance', 'slot_yaw_tolerance_deg',
                    'prepare_sequence', 'pickup_sequence', 'miss_sequence',
                    'dock_release_sequence', 'final_sequence', 'search', 'micro_sweep',
                })
            elif stype == 'terminate':
                self._warn_unknown(f"[{sid}]", s, self._STAGE_COMMON_KEYS)
            else:
                self.logger.warn(f"[{sid}] unknown stage type '{stype}'")

            # -- arm block content validation (all stage types can have it) --
            arm_block = s.get('arm')
            if isinstance(arm_block, dict):
                self._warn_unknown(f"[{sid}] arm", arm_block, actuator_names)
                for act_name, act_value in arm_block.items():
                    act = self.actuators.get(act_name)
                    if act is None:
                        continue  # warned as unknown key above
                    if act.get('type') == 'motor':
                        positions = act.get('positions', {})
                        if act_value not in positions:
                            self.logger.warn(
                                f"[{sid}] arm.{act_name}: unknown position '{act_value}' "
                                f"(valid: {list(positions.keys())})"
                            )
                    elif act.get('type') == 'pneumatic':
                        states = act.get('states', [])
                        if act_value not in states:
                            self.logger.warn(
                                f"[{sid}] arm.{act_name}: unknown state '{act_value}' "
                                f"(valid: {states})"
                            )

            # -- sub-block unknown keys --
            chassis = s.get('chassis') or {}
            if isinstance(chassis, dict):
                self._warn_unknown(f"[{sid}] chassis", chassis, self._KNOWN_CHASSIS_KEYS)
                ta = chassis.get('torque_arrival')
                if isinstance(ta, dict):
                    self._warn_unknown(f"[{sid}] chassis.torque_arrival", ta, self._KNOWN_TORQUE_ARRIVAL_KEYS)
            ta2 = s.get('torque_arrival')
            if isinstance(ta2, dict):
                self._warn_unknown(f"[{sid}] torque_arrival", ta2, self._KNOWN_TORQUE_ARRIVAL_KEYS)

            cond = s.get('condition')
            if isinstance(cond, dict):
                self._warn_unknown(f"[{sid}] condition", cond, self._KNOWN_CONDITION_KEYS)

            scan = s.get('scan') or {}
            if isinstance(scan, dict):
                self._warn_unknown(f"[{sid}] scan", scan, self._KNOWN_SCAN_KEYS)
            step_cfg = s.get('step') or {}
            if isinstance(step_cfg, dict):
                self._warn_unknown(f"[{sid}] step", step_cfg, self._KNOWN_STEP_KEYS)
            ms = s.get('micro_sweep') or {}
            if isinstance(ms, dict):
                self._warn_unknown(f"[{sid}] micro_sweep", ms, self._KNOWN_MICRO_SWEEP_KEYS)
            search_cfg = s.get('search') or {}
            if isinstance(search_cfg, dict):
                self._warn_unknown(f"[{sid}] search", search_cfg, self._KNOWN_SEARCH_KEYS)

            # -- reference checks (duplicate existing logic, enhanced for new types) --
            if stype == 'navigate':
                if s.get('to') not in self.waypoints:
                    self.logger.warn(f"[{sid}] waypoint '{s.get('to')}' not found")
                if s.get('profile') not in self.profiles:
                    self.logger.warn(f"[{sid}] profile '{s.get('profile')}' not found")

            elif stype == 'action':
                ch = s.get('chassis') or {}
                if isinstance(ch, dict):
                    if ch.get('to') and ch['to'] not in self.waypoints:
                        self.logger.warn(f"[{sid}] chassis.to waypoint '{ch['to']}' not found")
                    if ch.get('profile') and ch['profile'] not in self.profiles:
                        self.logger.warn(f"[{sid}] chassis.profile '{ch['profile']}' not found")

            elif stype == 'arm':
                for key in s:
                    if key in ('type', 'id'):
                        continue
                    if key not in self.actuators:
                        self.logger.warn(f"[{sid}] actuator '{key}' not defined")

            elif stype in ('condition', 'conditional'):
                then_id = s.get('then')
                else_id = s.get('else')
                if then_id and then_id not in stage_ids:
                    self.logger.warn(f"[{sid}] then stage '{then_id}' not found")
                if else_id and else_id not in stage_ids:
                    self.logger.warn(f"[{sid}] else stage '{else_id}' not found")

            elif stype == 'verify_ir':
                for branch_key in ('on_true', 'on_false', 'on_match', 'on_mismatch'):
                    target_id = s.get(branch_key)
                    if target_id and target_id not in ('continue', 'advance', 'terminate') and target_id not in stage_ids:
                        self.logger.warn(f"[{sid}] {branch_key} stage '{target_id}' not found")

            elif stype == 'sequential':
                for step in s.get('steps', []):
                    stp_type = step.get('type', '?')
                    if stp_type == 'arm':
                        for key in step:
                            if key in ('type',):
                                continue
                            if key not in self.actuators:
                                self.logger.warn(f"[{sid}] steps arm: actuator '{key}' not defined")
                    elif stp_type == 'wait':
                        self._warn_unknown(f"[{sid}] steps wait", step, {'type', 'duration_s'})
                    elif stp_type == 'stop_chassis':
                        self._warn_unknown(f"[{sid}] steps stop_chassis", step, {'type'})
                    else:
                        self.logger.warn(f"[{sid}] steps: unknown step type '{stp_type}'")

            elif stype == 'parallel':
                for action in s.get('actions', []):
                    act_type = action.get('type', '?')
                    if act_type == 'arm':
                        for key in action:
                            if key in ('type',):
                                continue
                            if key not in self.actuators:
                                self.logger.warn(f"[{sid}] actions arm: actuator '{key}' not defined")
                    else:
                        self.logger.warn(f"[{sid}] actions: unknown action type '{act_type}'")

            elif stype == 'weapon_head_pickup':
                mode = s.get('search_mode', 'scan_until_ir')
                if mode not in ('scan_until_ir', 'step_0p2m', 'micro_sweep_10mm'):
                    self.logger.warn(f"[{sid}] unknown search_mode '{mode}'")
                for step in s.get('pickup_sequence', []):
                    stp_type = step.get('type', '?')
                    if stp_type == 'arm':
                        for key in step:
                            if key in ('type', 'id'):
                                continue
                            if key not in self.actuators:
                                self.logger.warn(f"[{sid}] pickup arm: actuator '{key}' not defined")
                        self._warn_unknown(f"[{sid}] pickup arm", step, {'type', 'id'} | actuator_names)
                    elif stp_type == 'wait':
                        self._warn_unknown(f"[{sid}] pickup wait", step, {'type', 'id', 'duration_s'})
                    elif stp_type == 'verify_ir':
                        self._warn_unknown(f"[{sid}] pickup verify_ir", step, self._KNOWN_VERIFY_IR_KEYS)
                        for branch_key in ('on_true', 'on_false', 'on_match', 'on_mismatch'):
                            target_id = step.get(branch_key)
                            if target_id and target_id not in ('continue', 'advance', 'terminate') and target_id not in stage_ids:
                                self.logger.warn(f"[{sid}] pickup verify_ir {branch_key} stage '{target_id}' not found")
                    elif stp_type in ('condition', 'conditional'):
                        self._warn_unknown(f"[{sid}] pickup condition", step, {'type', 'id', 'condition', 'then', 'else'})
                        cond_cfg = step.get('condition') or {}
                        if isinstance(cond_cfg, dict):
                            self._warn_unknown(f"[{sid}] pickup condition.condition", cond_cfg, self._KNOWN_CONDITION_KEYS)
                        for b in ('then', 'else'):
                            t = step.get(b)
                            if t and t not in stage_ids:
                                seq_ids = {x.get('id') for x in s.get('pickup_sequence', []) if x.get('id')}
                                if t not in seq_ids:
                                    self.logger.warn(f"[{sid}] pickup condition {b} '{t}' not found in pickup_sequence or stages")
                    elif stp_type == 'action':
                        self._warn_unknown(f"[{sid}] pickup action", step, {'type', 'id', 'arm', 'chassis'})
                        ch = step.get('chassis') or {}
                        if isinstance(ch, dict):
                            self._warn_unknown(f"[{sid}] pickup action chassis", ch, self._KNOWN_CHASSIS_KEYS)
                    elif stp_type in ('navigate',):
                        self._warn_unknown(f"[{sid}] pickup navigate", step, {'type', 'id', 'to', 'profile', 'timeout_s', 'torque_arrival'})
                    elif stp_type == 'stop_chassis':
                        self._warn_unknown(f"[{sid}] pickup stop_chassis", step, {'type', 'id'})
                    else:
                        self.logger.warn(f"[{sid}] pickup_sequence: unknown step type '{stp_type}'")

            elif stype == 'red_area_weapon_cycle':
                if s.get('start_waypoint') not in self.waypoints:
                    self.logger.warn(f"[{sid}] start_waypoint '{s.get('start_waypoint')}' not found")
                profile_name = s.get('slot_profile', 'head_rack_speed')
                if profile_name not in self.profiles:
                    self.logger.warn(f"[{sid}] slot_profile '{profile_name}' not found")
                srch = s.get('search', {}) or {}
                if isinstance(srch, dict) and srch.get('profile') and srch['profile'] not in self.profiles:
                    self.logger.warn(f"[{sid}] search profile '{srch['profile']}' not found")
                ms2 = s.get('micro_sweep') or {}
                if isinstance(ms2, dict) and ms2.get('profile') and ms2['profile'] not in self.profiles:
                    self.logger.warn(f"[{sid}] micro_sweep profile '{ms2['profile']}' not found")
                for seq_name in ('prepare_sequence', 'pickup_sequence', 'miss_sequence',
                                 'dock_release_sequence', 'final_sequence'):
                    for step in s.get(seq_name, []):
                        stp_type = step.get('type', 'wait')
                        if stp_type == 'arm':
                            for key in step:
                                if key in ('type',):
                                    continue
                                if key not in self.actuators:
                                    self.logger.warn(f"[{sid}] {seq_name} actuator '{key}' not defined")
                        elif stp_type == 'wait':
                            self._warn_unknown(f"[{sid}] {seq_name} wait", step, {'type', 'duration_s'})
                        elif stp_type == 'stop_chassis':
                            self._warn_unknown(f"[{sid}] {seq_name} stop_chassis", step, {'type'})
                        else:
                            self.logger.warn(f"[{sid}] {seq_name} has unknown step type '{stp_type}'")

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
            self._arm_keepalive_poll()
            return

        if self.stage_index >= len(self.stages):
            self.phase = 'done'
            self.logger.info("Mission complete")
            self._arm_keepalive_poll()
            return

        stage = self.stages[self.stage_index]
        stype = stage.get('type', 'wait')

        # ---- Stage entry: apply optional arm block from YAML ----
        current_id = stage.get('id', '?')
        if current_id != self._active_stage_id:
            self._on_stage_enter(stage)

        # ---- Dispatch (old type names are aliased to unified types) ----
        # action  ← action, navigate, arm, stop_chassis
        # condition ← condition, conditional, verify_ir
        if stype in ('action', 'arm', 'navigate', 'stop_chassis'):
            self._update_action(stage)
        elif stype in ('condition', 'conditional', 'verify_ir'):
            self._update_condition(stage)
        elif stype == 'wait':
            self._update_wait(stage)
        elif stype == 'weapon_head_pickup':
            self._update_weapon_head_pickup(stage)
        elif stype == 'red_area_weapon_cycle':
            self._update_red_area_weapon_cycle(stage)
        elif stype == 'sequential':
            self._update_sequential(stage)
        elif stype == 'parallel':
            self._update_parallel(stage)
        elif stype == 'terminate':
            self._execute_terminate()
        else:
            self.logger.warn(f"Unknown stage type '{stype}' at [{current_id}], skipping")
            self._advance_stage()

        # ---- Arm keep-alive: republish full arm state every N ms ----
        # This is the deterministic guarantee — no matter what stage we are in,
        # the arm state is continuously refreshed so the Arduino 200ms watchdog
        # can never fire on stale data, even if downstream serial reconnects.
        self._arm_keepalive_poll()

    # ------------------------------------------------------------------
    # Stage: navigate
    # ------------------------------------------------------------------

    def _update_navigate(self, stage):
        # Normalize: support new 'chassis' block and old flat format
        chassis = stage.get('chassis', {})
        wp_name = chassis.get('to', stage.get('to'))
        if wp_name is None:
            self.logger.warn("Navigate/Action with no 'to' waypoint; treating as pure arm")
            self._advance_stage()
            return
        wp = self.waypoints[wp_name]
        profile_name = chassis.get('profile', stage.get('profile', 'normal'))
        profile = self.profiles.get(profile_name, {})

        end_pose = wp['pose']

        if self.current_pose is None:
            self._pub_zero_driving()
            return

        self._begin_navigate_stage(stage, end_pose, profile)

        dist = get_distance(self.current_pose, end_pose)
        yaw_err = abs(normalize_angle(end_pose['yaw'] - self.current_pose['yaw']))

        pos_tol = wp.get('pos_tolerance', 0.05)
        yaw_tol = wp.get('yaw_tolerance', 0.1)

        if dist < pos_tol and yaw_err < yaw_tol:
            self.arrived_counter += 1
        else:
            self.arrived_counter = 0

        if self.arrived_counter >= self.arrived_stable_count:
            self.logger.info(f"Arrived at {wp_name}")
            self.arrived_counter = 0
            self._pub_zero_driving()
            self._advance_stage()
            return

        if self._navigate_torque_arrived(stage):
            self._pub_zero_driving()
            self._advance_stage()
            return

        if self._navigate_timed_out(stage):
            self._pub_zero_driving()
            self._advance_stage()
            return

        self._pub_target_pose(end_pose)

        # Compute driving command
        start_pose = self._nav_from_pose

        # 预计算旋转（XY 分立和 heading 共用）
        cos_yaw = math.cos(self.current_pose['yaw'])
        sin_yaw = math.sin(self.current_pose['yaw'])

        has_xy_split = ('k_p_x' in profile or 'k_p_y' in profile)

        # =================================================================
        # XY 分立模式：机体 PID 控制，不用路径前进层
        # vx_body = kp*e + ki*∫e + kd*de, vy_body 同理
        # 机体 X/Y 轴独立驱动，互不干扰
        # =================================================================
        if has_xy_split:
            k_p_x = float(profile.get('k_p_x', DEFAULT_K_P_X))
            k_p_y = float(profile.get('k_p_y', DEFAULT_K_P_Y))
            k_i_x = float(profile.get('k_i_x', DEFAULT_K_I_X))
            k_i_y = float(profile.get('k_i_y', DEFAULT_K_I_Y))
            k_d_x = float(profile.get('k_d_x', DEFAULT_K_D_X))
            k_d_y = float(profile.get('k_d_y', DEFAULT_K_D_Y))

            # 世界系位置误差 → 机体系误差
            ex_w = end_pose['x'] - self.current_pose['x']
            ey_w = end_pose['y'] - self.current_pose['y']
            ex_body =  ex_w * cos_yaw + ey_w * sin_yaw
            ey_body = -ex_w * sin_yaw + ey_w * cos_yaw

            # 首周期初始化：避免 D 项跳变
            if not self._xy_pid_initialized:
                self._xy_error_prev_x = ex_body
                self._xy_error_prev_y = ey_body
                self._xy_pid_initialized = True

            # 积分累积 + 抗饱和 (anti-windup)
            integral_max_x = float(profile.get('xy_integral_max', 0.0))
            integral_max_y = float(profile.get('xy_integral_max', 0.0))
            self._xy_error_integral_x += ex_body
            self._xy_error_integral_y += ey_body
            if integral_max_x > 0:
                self._xy_error_integral_x = max(-integral_max_x, min(integral_max_x, self._xy_error_integral_x))
            if integral_max_y > 0:
                self._xy_error_integral_y = max(-integral_max_y, min(integral_max_y, self._xy_error_integral_y))

            # 微分 (误差变化量，不经 dt 缩放 — dt 吸收进 kd)
            d_x = ex_body - self._xy_error_prev_x
            d_y = ey_body - self._xy_error_prev_y
            self._xy_error_prev_x = ex_body
            self._xy_error_prev_y = ey_body

            # 机体 PID 速度
            vx_body = k_p_x * ex_body + k_i_x * self._xy_error_integral_x + k_d_x * d_x
            vy_body = k_p_y * ey_body + k_i_y * self._xy_error_integral_y + k_d_y * d_y

            # Alpha: 基于目标欧氏距离的 cubic ease（替代沿路径投影）
            dist_to_target = math.sqrt(ex_w**2 + ey_w**2)
            start_r = float(profile.get('start_radius_m', 0.0))
            end_r = float(profile.get('end_radius_m', 0.0))
            alpha_start = 1.0
            alpha_end = 1.0
            if start_r > 0 or end_r > 0:
                dist_total = math.sqrt(
                    (end_pose['x'] - start_pose['x'])**2 +
                    (end_pose['y'] - start_pose['y'])**2
                )
                if start_r > 0 and dist_total > 0:
                    dist_from_start = math.sqrt(
                        (self.current_pose['x'] - start_pose['x'])**2 +
                        (self.current_pose['y'] - start_pose['y'])**2
                    )
                    s = min(dist_from_start / start_r, 1.0)
                    alpha_start = 3.0 * s**2 - 2.0 * s**3
                if end_r > 0:
                    s = min(dist_to_target / end_r, 1.0)
                    alpha_end = 3.0 * s**2 - 2.0 * s**3
            alpha = min(alpha_start, alpha_end)
            min_scale = float(profile.get('min_speed_scale', DEFAULT_MIN_SPEED_SCALE))
            if dist > pos_tol and min_scale > 0.0:
                alpha = max(alpha, min(min_scale, 1.0))

            # 机体分轴限幅 + alpha 缩放
            max_body_x = float(profile.get('max_body_x_mps', DEFAULT_MAX_LATERAL_MPS))
            max_body_y = float(profile.get('max_body_y_mps', DEFAULT_MAX_LATERAL_MPS))
            vx_body = max(-max_body_x, min(max_body_x, vx_body)) * alpha
            vy_body = max(-max_body_y, min(max_body_y, vy_body)) * alpha

            # Heading 控制（复用 tracker 的 heading PID，忽略平移输出）
            k_heading = float(profile.get('k_heading_p', TRACKER_K_HEADING_P))
            k_heading_d = float(profile.get('k_heading_d', TRACKER_K_HEADING_D))
            _, _, omega_raw, _, _ = self.tracker.compute_pid_cte(
                self.current_pose, start_pose, end_pose,
                {'method': 'pid_cte', 'k_cte_p': 0.0,
                 'k_heading_p': k_heading, 'k_heading_d': k_heading_d,
                 'speed_mps': 0.0},
            )
            max_omega = profile.get('yaw_rate_rps', 1.5)
            omega = max(-max_omega, min(max_omega, omega_raw))

            self._pub_driving_body(vx_body, vy_body, omega)
            return

        # =================================================================
        # 原有 CTE 模式：路径前进速度 + 横向 CTE 修正
        # =================================================================
        tracker_speed_mps = min(
            float(profile.get('speed_mps', TRACKER_MAX_SPEED_MPS)),
            TRACKER_MAX_SPEED_MPS,
        )
        k_heading = float(profile.get('k_heading_p', TRACKER_K_HEADING_P))
        k_heading_d = float(profile.get('k_heading_d', TRACKER_K_HEADING_D))

        fwd_mps, lat_mps, omega_raw, u_fwd, u_lat = self.tracker.compute_pid_cte(
            self.current_pose,
            start_pose,
            end_pose,
            {
                'method': 'pid_cte',
                'k_cte_p': float(profile.get('k_cte_p', TRACKER_K_CTE_P)),
                'k_heading_p': k_heading,
                'k_heading_d': k_heading_d,
                'speed_mps': tracker_speed_mps,
            },
        )

        # --- 横向修正独立限幅，不参与 XY 合并压缩 ---
        max_lat = float(profile.get('max_lateral_mps', DEFAULT_MAX_LATERAL_MPS))
        lat_mps = max(-max_lat, min(max_lat, lat_mps))

        # --- 速度曲线缩放平移分量 ---
        alpha = self.speed_profiler.compute_alpha(
            self.current_pose, start_pose, end_pose,
            {
                'start_radius_m': profile.get('start_radius_m', 0.3),
                'end_radius_m': profile.get('end_radius_m', 0.3),
                'curve': 'cubic_ease',
            }
        )
        min_speed_scale = float(profile.get('min_speed_scale', DEFAULT_MIN_SPEED_SCALE))
        if dist > pos_tol and min_speed_scale > 0.0:
            alpha = max(alpha, min(min_speed_scale, 1.0))

        fwd_mps *= alpha
        lat_mps *= alpha

        # --- 世界系速度：前向 + 横向分别重建，互不挤压 ---
        vx_world = fwd_mps * u_fwd[0] + lat_mps * u_lat[0]
        vy_world = fwd_mps * u_fwd[1] + lat_mps * u_lat[1]

        # --- heading 独立限幅 ---
        max_omega = profile.get('yaw_rate_rps', 1.5)
        omega = max(-max_omega, min(max_omega, omega_raw))

        # --- 世界系 → 机体系旋转变换 ---
        vx_body =  vx_world * cos_yaw + vy_world * sin_yaw
        vy_body = -vx_world * sin_yaw + vy_world * cos_yaw

        # --- 机体分轴限幅 ---
        max_body_x = float(profile.get('max_body_x_mps', max_lat))
        max_body_y = float(profile.get('max_body_y_mps', max_lat))
        vx_body = max(-max_body_x, min(max_body_x, vx_body))
        vy_body = max(-max_body_y, min(max_body_y, vy_body))

        self._pub_driving_body(vx_body, vy_body, omega)

    def _begin_navigate_stage(self, stage, end_pose, profile):
        """Latch navigation start state once per navigate stage.

        The speed profile is distance-based. If the start pose follows the
        live robot pose every cycle, distance-from-start remains zero and the
        cubic profile never produces translational speed.
        """
        stage_id = stage.get('id', '?')
        if self._active_nav_stage_id == stage_id:
            return

        self._active_nav_stage_id = stage_id
        self._nav_start_time = time.monotonic()
        self._nav_from_pose = dict(self.current_pose)
        self._nav_target_wp = dict(end_pose)
        self._nav_profile = dict(profile)
        self.arrived_counter = 0

        # 每进入新 navigate stage 时重置 XY PID 积分/微分状态
        self._xy_error_integral_x = 0.0
        self._xy_error_integral_y = 0.0
        self._xy_error_prev_x = 0.0
        self._xy_error_prev_y = 0.0
        self._xy_pid_initialized = False
        self.logger.info(
            f"Navigate [{stage_id}] started from "
            f"({self._nav_from_pose['x']:.3f}, {self._nav_from_pose['y']:.3f}) "
            f"to ({end_pose['x']:.3f}, {end_pose['y']:.3f})"
        )

    def _navigate_torque_arrived(self, stage):
        """Return True when a navigate stage should early-exit on motor torque.

        This is optional per stage. It lets a mission treat physical contact as
        arrival while keeping normal waypoint arrival as the default behavior.
        """
        chassis = stage.get('chassis', {})
        cfg = chassis.get('torque_arrival') or stage.get('torque_arrival') or {}
        if not cfg:
            return False

        min_elapsed_s = float(cfg.get('min_elapsed_s', 0.0))
        if self._nav_start_time > 0.0 and time.monotonic() - self._nav_start_time < min_elapsed_s:
            return False

        topic = cfg.get('topic', '/damiao_feedback')
        field = cfg.get('field', 'chassis_motor_tau')
        sensor_data = self.sensor_cache.get(topic)
        if not sensor_data or field not in sensor_data:
            return False

        stamp_field = cfg.get('stamp_field')
        if stamp_field is None:
            if field.startswith('motor_') and field.endswith('_tau'):
                stamp_field = field.replace('_tau', '_stamp')
            elif field.startswith('chassis_motor_'):
                stamp_field = 'chassis_motor_stamp'
            else:
                stamp_field = '_stamp'
        stamp = sensor_data.get(stamp_field, sensor_data.get('_stamp'))
        max_age_s = float(cfg.get('max_age_s', 0.25))
        if stamp is None or time.monotonic() - float(stamp) > max_age_s:
            return False

        actual = float(sensor_data.get(field, 0.0))
        threshold = float(cfg.get('abs_threshold_nm', cfg.get('threshold_nm', cfg.get('value', 0.0))))
        op = cfg.get('op', 'abs_gte')
        if op == 'abs_gt':
            triggered = abs(actual) > threshold
        elif op == 'abs_gte':
            triggered = abs(actual) >= threshold
        elif op == 'gt':
            triggered = actual > threshold
        elif op == 'gte':
            triggered = actual >= threshold
        elif op == 'lt':
            triggered = actual < threshold
        elif op == 'lte':
            triggered = actual <= threshold
        else:
            self.logger.warn(f"Unknown torque_arrival op '{op}', ignoring")
            return False

        if triggered:
            self.logger.info(
                f"Navigate [{stage.get('id', '?')}] torque arrival: "
                f"{field}={actual:.3f}Nm, op={op}, threshold={threshold:.3f}Nm"
            )
        return triggered

    def _navigate_timed_out(self, stage):
        """Return True when a navigate stage exceeds its optional timeout_s."""
        chassis = stage.get('chassis', {})
        timeout_s = chassis.get('timeout_s', stage.get('timeout_s'))
        if timeout_s is None:
            return False
        timeout_s = float(timeout_s)
        if timeout_s <= 0.0 or self._nav_start_time <= 0.0:
            return False
        elapsed = time.monotonic() - self._nav_start_time
        if elapsed < timeout_s:
            return False

        self.logger.warn(
            f"Navigate [{stage.get('id', '?')}] timeout: "
            f"elapsed={elapsed:.2f}s >= {timeout_s:.2f}s; advancing"
        )
        return True

    def _pub_driving_body(self, vx_body, vy_body, omega):
        """Publish body-frame velocity command to /local_driving.

        vx_body, vy_body must already be in body frame (世界系已旋转到机体系).
        """
        if self.pub_driving is None:
            return

        direction = math.atan2(vy_body, vx_body)
        speed_mps = math.sqrt(vx_body ** 2 + vy_body ** 2)

        msg = Float32MultiArray()
        msg.data = [float(direction), float(speed_mps), float(omega)]
        self.pub_driving.publish(msg)

    def _pub_zero_driving(self):
        if self.pub_driving is None:
            return
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0, 0.0]
        self.pub_driving.publish(msg)

    def _pub_target_pose(self, pose):
        """Publish the active navigation target for plot/debug nodes."""
        if self.pub_target_pose is None:
            return
        msg = Pose2D()
        msg.x = float(pose.get('x', 0.0))
        msg.y = float(pose.get('y', 0.0))
        msg.theta = math.degrees(float(pose.get('yaw', 0.0)))
        self.pub_target_pose.publish(msg)

    # ------------------------------------------------------------------
    # Stage: arm (retained for backward compat; see also _update_action)
    # ------------------------------------------------------------------

    def _execute_arm(self, stage):
        """Apply arm commands from a YAML stage/step and track state.

        Supports both new 'arm' block and old inline actuator keys.
        Called from pickup_sequence, red_area_cycle sequences, sequential
        sub-steps, and top-level arm stages (via _on_stage_enter + keep-alive).

        Updates _current_arm_state so the keep-alive republish always sends
        the full deterministic arm configuration.
        """
        arm_state = self._extract_arm_state(stage)
        if arm_state:
            self._current_arm_state.update(arm_state)
        self._publish_full_arm_state()

    # ------------------------------------------------------------------
    # Arm state extraction, publishing, keep-alive, and unified dispatch
    # ------------------------------------------------------------------

    def _extract_arm_state(self, stage):
        """Extract arm state dict from a stage/step, supporting both formats.

        New format:
          arm:
            arm_gripper: close
            arm_lift: low
            ...

        Old format (inline actuator keys on arm-type stages):
          type: arm
          arm_gripper: close
          arm_lift: low
          ...
        """
        # New format: explicit 'arm' block
        arm_block = stage.get('arm')
        if isinstance(arm_block, dict) and arm_block:
            result = {}
            for name, value in arm_block.items():
                if name in self.actuators:
                    result[name] = value
                else:
                    self.logger.warn(f"arm block references unknown actuator '{name}'")
            return result if result else None

        # Old format: inline actuator keys on arm-type stages
        stype = stage.get('type', '')
        if stype in ('arm',):
            result = {}
            for key, value in stage.items():
                if key in ('type', 'id', 'wait_motors',
                           'motor_arrival_timeout_s', 'motor_arrival_tolerance_rad'):
                    continue
                if key in self.actuators:
                    result[key] = value
            return result if result else None

        return None

    def _publish_full_arm_state(self):
        """Resolve _current_arm_state → joint/pneu messages and publish.

        Sends every tracked actuator so arm_ctrl_node receives a complete
        command frame. Called on stage entry and by the keep-alive timer.
        """
        if not self._current_arm_state:
            return

        joint_triplets = []          # [motor_id, pos, speed, ...]
        pneu_pairs = []              # ["name:index", ...]

        for name, value in self._current_arm_state.items():
            act = self.actuators.get(name)
            if act is None:
                continue

            if act['type'] == 'motor':
                motor_id = int(act['motor_id'])
                position = float(act['positions'][value])
                speed = float(act.get('speed', 3.0))
                joint_triplets.extend([float(motor_id), position, speed])

            elif act['type'] == 'pneumatic':
                target_val = act['states'].index(value)
                pneu_pairs.append(f"{name}:{target_val}")

        if joint_triplets:
            self._pub_joint_cmd(joint_triplets)
        if pneu_pairs:
            self._pub_pneu_cmd(",".join(pneu_pairs))

        self._last_arm_publish_time = time.monotonic()

    def _on_stage_enter(self, stage):
        """Apply optional arm block when entering a new stage.

        Called exactly once per stage by update() when _active_stage_id changes.
        If the stage declares an 'arm' block, those values merge into
        _current_arm_state and are published immediately. Actuators NOT listed
        in the arm block retain their previous values.
        """
        sid = stage.get('id', '?')
        self._active_stage_id = sid
        self.logger.info(f"Entering stage [{sid}] type={stage.get('type', '?')}")

        arm_state = self._extract_arm_state(stage)
        if arm_state:
            self._current_arm_state.update(arm_state)
            self._publish_full_arm_state()
            self.logger.info(f"[{sid}] arm state: {list(arm_state.keys())}")

    def _arm_keepalive_poll(self):
        """Republish full arm state at configured interval to keep watchdog alive.

        Called unconditionally from update() every cycle (50Hz). Only actually
        publishes when enabled and the interval has elapsed. No-op when
        _current_arm_state is empty (no arm commands issued yet).

        The 100ms interval beats the Arduino's 200ms firmware watchdog, so even
        if the downstream serial chain restarts or reconnects, the next
        keep-alive beat restores commanded state before the watchdog fires.
        """
        if not self._arm_keepalive_enabled:
            return
        if not self._current_arm_state:
            return
        if (time.monotonic() - self._last_arm_publish_time
                < self._arm_keepalive_interval_s):
            return

        self._publish_full_arm_state()

    # ------------------------------------------------------------------
    # Unified stage types: action, condition
    # ------------------------------------------------------------------

    def _update_action(self, stage):
        """Unified handler for action / navigate / arm / stop_chassis.

        - Has chassis.to  → navigate to waypoint (arm state maintained by keep-alive)
        - Has chassis.stop → publish zero driving and advance
        - No chassis      → pure arm action (arm state already applied by _on_stage_enter),
                             advance immediately
        """
        chassis = stage.get('chassis', {})
        if chassis:
            if chassis.get('to'):
                self._update_navigate(stage)
                return
            if chassis.get('stop'):
                self._pub_zero_driving()
                self._advance_stage()
                return

        # Pure arm action (or old arm/navigate with inline keys and chassis in top-level)
        stype = stage.get('type', '')
        if stype == 'arm':
            self._advance_stage()
            return
        if stype == 'navigate':
            self._update_navigate(stage)
            return
        if stype == 'stop_chassis':
            self._pub_zero_driving()
            self._advance_stage()
            return

        # New-style action with no chassis block = pure arm → advance
        self._advance_stage()

    def _update_condition(self, stage):
        """Unified handler for condition / conditional / verify_ir.

        Polls sensor_cache and branches via _jump_to. The arm state declared
        in the stage's arm block is maintained by the keep-alive during the
        polling loop.
        """
        result = self._evaluate_condition(stage.get('condition', {}))
        if result is None:
            return

        target_id = stage['then'] if result else stage['else']
        self._jump_to(target_id)

    def _evaluate_condition(self, cond, default_max_age_s=None):
        """Evaluate a sensor_cache condition.

        Returns True/False when the condition can be evaluated. Returns None
        when required data is missing or stale so the caller can hold state.
        Torque feedback defaults to a freshness check because stale motor
        contact data can release a gripper before the current docking attempt.
        """
        topic = cond.get('topic')
        field = cond.get('field')
        op = cond.get('op', 'gt')
        threshold = float(cond.get('value', 0.0))

        sensor_data = self.sensor_cache.get(topic)
        if sensor_data is None or field not in sensor_data:
            return None

        max_age_s = cond.get('max_age_s', default_max_age_s)
        if max_age_s is None and self._condition_field_needs_freshness(topic, field):
            max_age_s = 0.25
        if max_age_s is not None and not self._condition_data_is_fresh(cond, sensor_data, field, float(max_age_s)):
            return None

        actual = float(sensor_data.get(field, 0.0))
        if op == 'gt':
            return actual > threshold
        if op == 'lt':
            return actual < threshold
        if op == 'gte':
            return actual >= threshold
        if op == 'lte':
            return actual <= threshold
        if op == 'abs_gt':
            return abs(actual) > threshold
        if op == 'abs_gte':
            return abs(actual) >= threshold

        self.logger.warn(f"Unknown condition op '{op}'")
        return None

    def _condition_field_needs_freshness(self, topic, field):
        """Return True for feedback fields that must not use stale cache."""
        return topic == '/damiao_feedback' or str(field).endswith('_tau')

    def _condition_data_is_fresh(self, cond, sensor_data, field, max_age_s):
        """Check timestamp freshness for condition data."""
        stamp_field = cond.get('stamp_field')
        if stamp_field is None:
            if str(field).startswith('motor_') and str(field).endswith('_tau'):
                stamp_field = str(field).replace('_tau', '_stamp')
            elif str(field).startswith('chassis_motor_'):
                stamp_field = 'chassis_motor_stamp'
            else:
                stamp_field = '_stamp'

        stamp = sensor_data.get(stamp_field, sensor_data.get('_stamp'))
        if stamp is None:
            return False
        return time.monotonic() - float(stamp) <= max_age_s

    # ------------------------------------------------------------------
    # Low-level publish helpers
    # ------------------------------------------------------------------

    def _pub_joint_cmd(self, targets):
        """Publish joint triplets to arm/joint_navigation.

        targets format: [motor_id, pos_rad, speed_rad_s, ...]
        """
        if self.pub_joint is None:
            return
        msg = Float32MultiArray()
        msg.data = [float(t) for t in targets]
        self.pub_joint.publish(msg)

    def _pub_pneu_cmd(self, targets):
        """Publish pneumatic name:value pairs to arm/pneu_navigation.

        targets format: "name1:val1,name2:val2,..."
        """
        if self.pub_pneu is None:
            return
        msg = String()
        msg.data = targets
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
        elif stype == 'stop_chassis':
            self._execute_stop_chassis()
            self._seq_step_index += 1
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
    # Stage: weapon_head_pickup
    # ------------------------------------------------------------------

    def _update_weapon_head_pickup(self, stage):
        """Search weapon head rack with IR, then run YAML-defined pickup steps.

        This stage belongs in navigation because it must coordinate chassis
        motion, sensor feedback, and arm actions in one ordered whole-robot
        decision. Arm details still stay parameterized in the YAML actuators and
        pickup_sequence blocks.
        """
        stage_id = stage.get('id', 'weapon_head_pickup')
        if self.current_pose is None:
            self._pub_zero_driving()
            return

        if self._weapon_stage_id != stage_id:
            self._begin_weapon_head_pickup(stage_id)

        if self._weapon_state == 'checking':
            self._update_weapon_checking(stage)
        elif self._weapon_state == 'step_move':
            self._update_weapon_step_move(stage)
        elif self._weapon_state == 'scan':
            self._update_weapon_scan(stage)
        elif self._weapon_state == 'micro_sweep_prepare':
            self._update_weapon_micro_sweep_prepare(stage)
        elif self._weapon_state == 'micro_sweep_scan':
            self._update_weapon_micro_sweep_scan(stage)
        elif self._weapon_state == 'pickup':
            self._update_weapon_pickup_sequence(stage)
        else:
            self.logger.warn(f"Unknown weapon pickup state '{self._weapon_state}', stopping")
            self._finish_weapon_pickup(stage, success=False)

    def _begin_weapon_head_pickup(self, stage_id):
        """Initialize per-stage search state for a new weapon pickup stage."""
        self._clear_navigation_state()
        self._weapon_stage_id = stage_id
        self._weapon_state = 'checking'
        self._weapon_slot_index = 0
        self._weapon_pickup_step_index = 0
        self._weapon_step_target_pose = None
        self._weapon_scan_start_pose = None
        self._weapon_scan_start_time = 0.0
        self._weapon_wait_start = 0.0
        self._weapon_sequence_action_index = -1
        self._weapon_warned_missing_ir = False
        self._weapon_warned_ir_timeout = False
        self._weapon_ir_timeout_start = 0.0
        self.logger.info(f"Weapon pickup [{stage_id}] started")

    def _update_weapon_checking(self, stage):
        """Check the current rack slot and choose the configured search mode."""
        ir_value = self._read_weapon_ir(stage)
        if ir_value is None:
            self._pub_zero_driving()
            if self._check_weapon_ir_timeout_fallback(stage):
                self._finish_weapon_pickup(stage, success=False)
            return

        if ir_value:
            self._pub_zero_driving()
            self._weapon_state = 'pickup'
            self._weapon_pickup_step_index = 0
            self._weapon_wait_start = 0.0
            self.logger.info(
                f"Weapon head detected at slot {self._weapon_slot_index + 1}; running pickup_sequence"
            )
            return

        mode = stage.get('search_mode', 'scan_until_ir')
        if mode == 'step_0p2m':
            self._start_next_weapon_step(stage)
        elif mode == 'scan_until_ir':
            self._start_weapon_scan(stage)
        elif mode == 'micro_sweep_10mm':
            self._start_weapon_micro_sweep(stage)
        else:
            self.logger.warn(f"Unknown weapon search_mode '{mode}'")
            self._finish_weapon_pickup(stage, success=False)

    def _start_next_weapon_step(self, stage):
        """Create a dynamic pose target one slot spacing ahead in body +X."""
        slot_count = int(stage.get('slot_count', 6))
        if self._weapon_slot_index >= max(slot_count - 1, 0):
            self.logger.warn(f"No weapon head found after checking {slot_count} slots")
            self._finish_weapon_pickup(stage, success=False)
            return

        spacing = float(stage.get('slot_spacing_m', 0.2))
        yaw = self.current_pose['yaw']
        self._weapon_step_target_pose = {
            'x': self.current_pose['x'] + spacing * math.cos(yaw),
            'y': self.current_pose['y'] + spacing * math.sin(yaw),
            'yaw': yaw,
        }
        self._weapon_slot_index += 1
        self._weapon_state = 'step_move'
        self._weapon_wait_start = 0.0
        self._clear_navigation_state()
        self.logger.info(
            f"IR false; stepping to weapon slot {self._weapon_slot_index + 1} "
            f"(+{spacing:.3f} m body X)"
        )

    def _update_weapon_step_move(self, stage):
        """Drive to the dynamic step target, then settle and re-check IR."""
        step_cfg = stage.get('step', {}) or {}
        profile_name = step_cfg.get('profile', 'slow')
        profile = self.profiles.get(profile_name, {})
        pos_tol = float(step_cfg.get('pos_tolerance', stage.get('step_pos_tolerance', 0.03)))
        yaw_tol = float(step_cfg.get('yaw_tolerance', stage.get('step_yaw_tolerance', 0.1)))

        settle_s = float(step_cfg.get('settle_s', 0.15))
        if self._weapon_wait_start != 0.0:
            self._pub_zero_driving()
            if time.time() - self._weapon_wait_start >= settle_s:
                self._weapon_wait_start = 0.0
                self._weapon_state = 'checking'
            return

        arrived = self._drive_to_dynamic_pose(
            f"{stage.get('id', 'weapon_head_pickup')}_slot_{self._weapon_slot_index}",
            self._weapon_step_target_pose,
            profile,
            pos_tol,
            yaw_tol,
        )
        if arrived:
            self._pub_zero_driving()
            self._weapon_wait_start = time.time()

    def _start_weapon_scan(self, stage):
        """Begin continuous low-speed scan until IR detects a weapon head."""
        self._weapon_scan_start_pose = dict(self.current_pose)
        self._weapon_scan_start_time = time.time()
        self._weapon_state = 'scan'
        self.logger.info("IR false; scanning forward until weapon head is detected")

    def _update_weapon_scan(self, stage):
        """Move slowly while watching IR, timeout, and max scan distance."""
        ir_value = self._read_weapon_ir(stage)
        if ir_value is None:
            self._pub_zero_driving()
            return

        if ir_value:
            self._pub_zero_driving()
            self._weapon_state = 'pickup'
            self._weapon_pickup_step_index = 0
            self._weapon_wait_start = 0.0
            self.logger.info("Weapon head detected during scan; running pickup_sequence")
            return

        scan_cfg = stage.get('scan', {}) or {}
        timeout_s = float(scan_cfg.get('timeout_s', 5.0))
        max_distance_m = float(scan_cfg.get('max_distance_m', stage.get('slot_spacing_m', 0.2) * max(int(stage.get('slot_count', 6)) - 1, 1)))
        elapsed = time.time() - self._weapon_scan_start_time
        moved = get_distance(self.current_pose, self._weapon_scan_start_pose)
        if elapsed > timeout_s or moved > max_distance_m:
            self.logger.warn(
                f"Weapon scan failed: elapsed={elapsed:.2f}s/{timeout_s:.2f}s, "
                f"distance={moved:.3f}m/{max_distance_m:.3f}m"
            )
            self._finish_weapon_pickup(stage, success=False)
            return

        direction = float(scan_cfg.get('direction_rad', 0.0))
        speed = float(scan_cfg.get('speed_mps', 0.05))
        vx_body = speed * math.cos(direction)
        vy_body = speed * math.sin(direction)
        self._pub_driving_body(vx_body, vy_body, 0.0)

    def _start_weapon_micro_sweep(self, stage):
        """Start a local -10mm to +10mm sweep around the current rack point.

        This policy is intended for the moment after the chassis has navigated
        to a rack point and the arm has already been commanded into its pickup
        pose. If IR is still false, the chassis first backs up a small distance
        along the configured body-frame direction, then slowly scans through the
        point to the forward side while continuously checking IR.
        """
        sweep_cfg = stage.get('micro_sweep', {}) or {}
        direction = float(sweep_cfg.get('direction_rad', 0.0))
        back_distance = float(sweep_cfg.get('back_distance_m', 0.01))

        yaw = self.current_pose['yaw'] + direction
        self._weapon_scan_start_pose = None
        self._weapon_scan_start_time = 0.0
        self._weapon_step_target_pose = {
            'x': self.current_pose['x'] - back_distance * math.cos(yaw),
            'y': self.current_pose['y'] - back_distance * math.sin(yaw),
            'yaw': self.current_pose['yaw'],
        }
        self._weapon_state = 'micro_sweep_prepare'
        self._weapon_wait_start = 0.0
        self._clear_navigation_state()
        self.logger.info(
            f"IR false; micro_sweep_10mm backing {back_distance:.3f}m before slow scan"
        )

    def _update_weapon_micro_sweep_prepare(self, stage):
        """Move to the pre-point side of the micro sweep, then start scanning."""
        ir_value = self._read_weapon_ir(stage)
        if ir_value is None:
            self._pub_zero_driving()
            if self._check_weapon_ir_timeout_fallback(stage):
                self._finish_weapon_pickup(stage, success=False)
            return

        if ir_value:
            self._pub_zero_driving()
            self._weapon_state = 'pickup'
            self._weapon_pickup_step_index = 0
            self._weapon_wait_start = 0.0
            self.logger.info("Weapon head detected before micro sweep; running pickup_sequence")
            return

        sweep_cfg = stage.get('micro_sweep', {}) or {}
        profile_name = sweep_cfg.get('profile', stage.get('step', {}).get('profile', 'slow'))
        profile = self.profiles.get(profile_name, {})
        pos_tol = float(sweep_cfg.get('pos_tolerance', 0.003))
        yaw_tol = float(sweep_cfg.get('yaw_tolerance', 0.05))

        arrived = self._drive_to_dynamic_pose(
            f"{stage.get('id', 'weapon_head_pickup')}_micro_sweep_back",
            self._weapon_step_target_pose,
            profile,
            pos_tol,
            yaw_tol,
        )
        if arrived:
            self._pub_zero_driving()
            self._weapon_scan_start_pose = dict(self.current_pose)
            self._weapon_scan_start_time = time.time()
            self._weapon_state = 'micro_sweep_scan'
            self.logger.info("micro_sweep_10mm reached back side; scanning through point")

    def _update_weapon_micro_sweep_scan(self, stage):
        """Scan slowly from the pre-point side to the post-point side."""
        ir_value = self._read_weapon_ir(stage)
        if ir_value is None:
            self._pub_zero_driving()
            if self._check_weapon_ir_timeout_fallback(stage):
                self._finish_weapon_pickup(stage, success=False)
            return

        if ir_value:
            self._pub_zero_driving()
            self._weapon_state = 'pickup'
            self._weapon_pickup_step_index = 0
            self._weapon_wait_start = 0.0
            self.logger.info("Weapon head detected during micro sweep; running pickup_sequence")
            return

        sweep_cfg = stage.get('micro_sweep', {}) or {}
        back_distance = float(sweep_cfg.get('back_distance_m', 0.01))
        forward_distance = float(sweep_cfg.get('forward_distance_m', 0.01))
        max_distance_m = float(sweep_cfg.get('max_distance_m', back_distance + forward_distance))
        timeout_s = float(sweep_cfg.get('timeout_s', 2.0))

        elapsed = time.time() - self._weapon_scan_start_time
        moved = get_distance(self.current_pose, self._weapon_scan_start_pose)
        if elapsed > timeout_s or moved > max_distance_m:
            self.logger.warn(
                f"micro_sweep_10mm failed: elapsed={elapsed:.2f}s/{timeout_s:.2f}s, "
                f"distance={moved:.3f}m/{max_distance_m:.3f}m"
            )
            self._finish_weapon_pickup(stage, success=False)
            return

        direction = float(sweep_cfg.get('direction_rad', 0.0))
        speed = float(sweep_cfg.get('speed_mps', 0.015))
        vx_body = speed * math.cos(direction)
        vy_body = speed * math.sin(direction)
        self._pub_driving_body(vx_body, vy_body, 0.0)

    def _update_weapon_pickup_sequence(self, stage):
        """Execute YAML pickup_sequence with arm, wait, and IR verification steps."""
        sequence = stage.get('pickup_sequence', [])
        if not sequence:
            self.logger.warn("weapon_head_pickup has no pickup_sequence; finishing")
            self._finish_weapon_pickup(stage, success=True)
            return

        if self._weapon_pickup_step_index >= len(sequence):
            self._finish_weapon_pickup(stage, success=True)
            return

        step = sequence[self._weapon_pickup_step_index]
        stype = step.get('type', 'wait')
        if stype == 'arm':
            self._execute_arm(step)
            self._weapon_pickup_step_index += 1
            self._weapon_wait_start = 0.0
        elif stype == 'wait':
            duration = float(step.get('duration_s', 0.0))
            if self._weapon_wait_start == 0.0:
                self._weapon_wait_start = time.time()
                return
            if time.time() - self._weapon_wait_start >= duration:
                self._weapon_wait_start = 0.0
                self._weapon_pickup_step_index += 1
        elif stype == 'verify_ir':
            self._update_weapon_verify_ir_step(stage, step)
        elif stype in ('condition', 'conditional'):
            self._update_weapon_condition_step(stage, step, sequence)
        elif stype in ('action', 'navigate', 'stop_chassis'):
            self._update_weapon_action_step(stage, step)
        else:
            self.logger.warn(f"Unknown pickup_sequence step type '{stype}', skipping")
            self._weapon_pickup_step_index += 1
            self._weapon_wait_start = 0.0

    def _update_weapon_action_step(self, stage, step):
        """Execute action/navigate/stop_chassis inside pickup_sequence.

        Navigation sub-steps reuse the normal navigate controller, but their
        completion advances only the pickup sequence step instead of leaving the
        surrounding weapon_head_pickup stage.
        """
        self._enter_weapon_action_step_once(step)
        chassis = step.get('chassis', {}) or {}
        stype = step.get('type', '')

        if chassis.get('to') or stype == 'navigate':
            self._update_weapon_navigate_step(stage, step)
            return

        if chassis.get('stop') or stype == 'stop_chassis':
            self._pub_zero_driving()
            self._advance_weapon_sequence_step(clear_navigation=True)
            return

        # Pure action with only an arm block: apply once and continue.
        self._advance_weapon_sequence_step(clear_navigation=False)

    def _enter_weapon_action_step_once(self, step):
        """Apply an action step's arm state once when entering that step."""
        if self._weapon_sequence_action_index == self._weapon_pickup_step_index:
            return
        self._weapon_sequence_action_index = self._weapon_pickup_step_index
        if self._extract_arm_state(step):
            self._execute_arm(step)

    def _update_weapon_navigate_step(self, stage, step):
        """Run a navigate step without letting _update_navigate advance stages."""
        nav_step = dict(step)
        if not nav_step.get('id'):
            nav_step['id'] = (
                f"{stage.get('id', 'weapon_head_pickup')}"
                f":pickup_step_{self._weapon_pickup_step_index}"
            )

        original_advance_stage = self._advance_stage

        def advance_pickup_step():
            self._advance_weapon_sequence_step(clear_navigation=True)

        self._advance_stage = advance_pickup_step
        try:
            self._update_navigate(nav_step)
        finally:
            self._advance_stage = original_advance_stage

    def _advance_weapon_sequence_step(self, clear_navigation=False):
        """Advance one pickup_sequence step and clear per-step state."""
        if clear_navigation:
            self._clear_navigation_state()
        self._weapon_pickup_step_index += 1
        self._weapon_wait_start = 0.0
        self._weapon_sequence_action_index = -1

    def _update_weapon_condition_step(self, stage, step, sequence):
        """Poll a pickup_sequence condition and branch inside the sequence.

        Blue point 1/2 uses this for motor 5 torque docking. A false branch can
        target the same step id to form a non-blocking 50Hz polling loop while
        the global arm keep-alive continues to refresh the gripper state.
        """
        result = self._evaluate_condition(step.get('condition', {}), default_max_age_s=0.25)
        if result is None:
            return

        target = step.get('then' if result else 'else')
        if not target:
            self._weapon_pickup_step_index += 1
            self._weapon_wait_start = 0.0
            return

        self._jump_within_weapon_sequence_or_stage(stage, sequence, str(target))

    def _jump_within_weapon_sequence_or_stage(self, stage, sequence, target):
        """Jump to a pickup_sequence step id, falling back to mission stage ids."""
        for i, step in enumerate(sequence):
            if step.get('id') == target:
                self._weapon_pickup_step_index = i
                self._weapon_wait_start = 0.0
                return

        self._jump_to(target)

    def _update_weapon_verify_ir_step(self, stage, step):
        """Re-check IR during pickup_sequence and optionally branch the mission.

        This keeps point-specific recovery in YAML: the executor only reads the
        configured IR field, applies the same timeout/CRC protection as the
        search phase, and follows the branch target selected by the mission.
        """
        ir_value = self._read_weapon_ir(stage)
        if ir_value is None:
            self._pub_zero_driving()
            if self._check_weapon_ir_timeout_fallback(stage):
                target = step.get('on_false', 'advance')
                self.logger.warn(
                    f"Weapon IR verify timeout fallback → {target}"
                )
                self._handle_weapon_ir_branch(stage, str(target), matched=False)
            return

        expected_raw = step.get('expected')
        matched = True
        if expected_raw is not None:
            matched = bool(ir_value) == bool(expected_raw)

        target = step.get('on_true' if ir_value else 'on_false')
        if target is None:
            target = step.get('on_match' if matched else 'on_mismatch')

        label = step.get('label', f"pickup_step_{self._weapon_pickup_step_index}")
        self.logger.info(
            f"Weapon IR verify [{label}]: actual={bool(ir_value)}, "
            f"expected={expected_raw if expected_raw is not None else 'any'}, "
            f"matched={matched}"
        )

        if target:
            self._handle_weapon_ir_branch(stage, str(target), matched)
            return

        if expected_raw is not None and not matched:
            self.logger.warn(f"Weapon IR verify [{label}] mismatched; finishing pickup as failed")
            self._finish_weapon_pickup(stage, success=False)
            return

        self._weapon_pickup_step_index += 1
        self._weapon_wait_start = 0.0

    def _handle_weapon_ir_branch(self, stage, target, matched):
        """Apply a verify_ir branch target."""
        if target == 'continue':
            self._weapon_pickup_step_index += 1
            self._weapon_wait_start = 0.0
        elif target == 'advance':
            self._finish_weapon_pickup(stage, success=matched)
        elif target == 'terminate':
            self._execute_terminate()
        else:
            self._jump_to(target)

    def _read_weapon_ir(self, stage):
        """Read configured IR boolean from the cached sensor topic.

        Returns True/False when the value is fresh enough, or None when the
        sensor data is missing/stale and the chassis must not move.

        CRC is NOT checked here: arm_arduino_node already validates serial
        frames via XOR-LRC before publishing to /arm/ir_status, so bad data
        never reaches this function. Timeout protection (ir_timeout_s) is
        the only gate — if sensor data stops arriving, the chassis holds.
        """
        topic = stage.get('ir_topic', '/arm/ir_status')
        field = stage.get('ir_field', 'ir')
        sensor_data = self.sensor_cache.get(topic)
        if sensor_data is None:
            if not self._weapon_warned_missing_ir:
                self.logger.warn(f"No IR sensor data on {topic}; weapon pickup is holding position")
                self._weapon_warned_missing_ir = True
            return None

        stamp = sensor_data.get('_stamp')
        timeout_s = float(stage.get('ir_timeout_s', 0.5))
        if stamp is not None and time.monotonic() - float(stamp) > timeout_s:
            if not self._weapon_warned_ir_timeout:
                self.logger.warn(f"IR sensor timeout ({timeout_s:.2f}s); weapon pickup is holding position")
                self._weapon_warned_ir_timeout = True
            # Don't reset _weapon_ir_timeout_start here — let _check_weapon_ir_timeout_fallback accumulate
            return None

        # Data is fresh — reset all timeout tracking
        self._weapon_warned_missing_ir = False
        self._weapon_warned_ir_timeout = False
        self._weapon_ir_timeout_start = 0.0
        return bool(sensor_data.get(field, False))

    def _check_weapon_ir_timeout_fallback(self, stage):
        """Return True when IR has been timing out continuously long enough to fallback.

        Uses ir_timeout_s as the accumulated grace period: once the sensor has
        been stale for longer than ir_timeout_s total, the caller should stop
        holding and fallback (on_miss for checking, on_false for verify_ir).
        """
        now = time.monotonic()
        if self._weapon_ir_timeout_start == 0.0:
            self._weapon_ir_timeout_start = now
        timeout_s = float(stage.get('ir_timeout_s', 0.5))
        elapsed = now - self._weapon_ir_timeout_start
        if elapsed > timeout_s:
            self.logger.warn(
                f"Weapon IR timeout fallback after {elapsed:.1f}s "
                f"(limit: {timeout_s:.1f}s)"
            )
            return True
        return False

    def _drive_to_dynamic_pose(self, stage_id, target_pose, profile, pos_tol, yaw_tol):
        """Drive to a generated pose using the same XY PID fields as navigate.

        Dynamic pose targets are used by weapon rack slot moves and Red Area
        micro-sweep pre-positioning. They need the same I/D tuning surface as
        normal navigate stages so field PID adjustments transfer consistently.
        """
        if target_pose is None or self.current_pose is None:
            self._pub_zero_driving()
            return False

        dist = get_distance(self.current_pose, target_pose)
        yaw_err_signed = normalize_angle(target_pose.get('yaw', self.current_pose['yaw']) - self.current_pose['yaw'])
        if dist < pos_tol and abs(yaw_err_signed) < yaw_tol:
            self.arrived_counter += 1
        else:
            self.arrived_counter = 0

        if self.arrived_counter >= self.arrived_stable_count:
            self.arrived_counter = 0
            self._clear_navigation_state()
            return True

        self._begin_navigate_stage({'id': stage_id}, target_pose, profile)
        self._pub_target_pose(target_pose)

        cos_yaw = math.cos(self.current_pose['yaw'])
        sin_yaw = math.sin(self.current_pose['yaw'])
        ex_w = target_pose['x'] - self.current_pose['x']
        ey_w = target_pose['y'] - self.current_pose['y']
        ex_body = ex_w * cos_yaw + ey_w * sin_yaw
        ey_body = -ex_w * sin_yaw + ey_w * cos_yaw

        k_p_x = float(profile.get('k_p_x', DEFAULT_K_P_X))
        k_p_y = float(profile.get('k_p_y', DEFAULT_K_P_Y))
        k_i_x = float(profile.get('k_i_x', DEFAULT_K_I_X))
        k_i_y = float(profile.get('k_i_y', DEFAULT_K_I_Y))
        k_d_x = float(profile.get('k_d_x', DEFAULT_K_D_X))
        k_d_y = float(profile.get('k_d_y', DEFAULT_K_D_Y))

        if not self._xy_pid_initialized:
            self._xy_error_prev_x = ex_body
            self._xy_error_prev_y = ey_body
            self._xy_pid_initialized = True

        self._xy_error_integral_x += ex_body
        self._xy_error_integral_y += ey_body
        integral_max_x = float(profile.get('xy_integral_max', 0.0))
        integral_max_y = float(profile.get('xy_integral_max', 0.0))
        if integral_max_x > 0:
            self._xy_error_integral_x = max(-integral_max_x, min(integral_max_x, self._xy_error_integral_x))
        if integral_max_y > 0:
            self._xy_error_integral_y = max(-integral_max_y, min(integral_max_y, self._xy_error_integral_y))

        d_x = ex_body - self._xy_error_prev_x
        d_y = ey_body - self._xy_error_prev_y
        self._xy_error_prev_x = ex_body
        self._xy_error_prev_y = ey_body

        vx_body = k_p_x * ex_body + k_i_x * self._xy_error_integral_x + k_d_x * d_x
        vy_body = k_p_y * ey_body + k_i_y * self._xy_error_integral_y + k_d_y * d_y

        max_body_x = float(profile.get('max_body_x_mps', DEFAULT_MAX_LATERAL_MPS))
        max_body_y = float(profile.get('max_body_y_mps', DEFAULT_MAX_LATERAL_MPS))
        vx_body = max(-max_body_x, min(max_body_x, vx_body))
        vy_body = max(-max_body_y, min(max_body_y, vy_body))

        k_heading = float(profile.get('k_heading_p', TRACKER_K_HEADING_P))
        k_heading_d = float(profile.get('k_heading_d', TRACKER_K_HEADING_D))
        _, _, omega_raw, _, _ = self.tracker.compute_pid_cte(
            self.current_pose,
            self._nav_from_pose,
            target_pose,
            {
                'method': 'pid_cte',
                'k_cte_p': 0.0,
                'k_heading_p': k_heading,
                'k_heading_d': k_heading_d,
                'speed_mps': 0.0,
            },
        )
        max_omega = float(profile.get('yaw_rate_rps', 1.5))
        omega = max(-max_omega, min(max_omega, omega_raw))
        self._pub_driving_body(vx_body, vy_body, omega)
        return False

    def _finish_weapon_pickup(self, stage, success):
        """Stop chassis and leave the weapon pickup stage."""
        self._pub_zero_driving()
        if success:
            self.logger.info("Weapon pickup sequence complete")
        else:
            self.logger.warn("Weapon pickup finished without detection/pickup")
            if stage.get('on_miss') == 'terminate':
                self._execute_terminate()
                return
        self._clear_weapon_state()
        self._advance_stage()


    # ------------------------------------------------------------------
    # Stage: red_area_weapon_cycle
    # ------------------------------------------------------------------

    def _update_red_area_weapon_cycle(self, stage):
        """Run the Red Area rack cycle over multiple weapon head positions.

        The stage keeps competition-specific flow in YAML while this executor
        provides reusable control primitives: slot navigation, IR verification,
        one retry per slot, torque-triggered docking release, and success-count
        termination.
        """
        stage_id = stage.get('id', 'red_area_weapon_cycle')
        if self.current_pose is None:
            self._pub_zero_driving()
            return

        if self._red_cycle_stage_id != stage_id:
            self._begin_red_area_weapon_cycle(stage)

        state = self._red_cycle_state
        if state == 'navigate_slot':
            self._update_red_cycle_navigate_slot(stage)
        elif state == 'sequence':
            self._update_red_cycle_sequence(stage)
        elif state == 'check_ir':
            self._update_red_cycle_check_ir(stage)
        elif state == 'micro_sweep_prepare':
            self._update_red_cycle_micro_sweep_prepare(stage)
        elif state == 'micro_sweep_scan':
            self._update_red_cycle_micro_sweep_scan(stage)
        elif state == 'verify_pickup':
            self._update_red_cycle_verify_pickup(stage)
        elif state == 'docking':
            self._update_red_cycle_docking(stage)
        elif state == 'complete':
            self._finish_red_cycle(stage)
        else:
            self.logger.warn(f"Unknown red area cycle state '{state}', stopping chassis")
            self._pub_zero_driving()
            self._start_red_cycle_final_sequence(stage)

    def _begin_red_area_weapon_cycle(self, stage):
        """Initialize per-stage counters for the Red Area rack cycle."""
        self._clear_navigation_state()
        self._clear_weapon_state()
        self._red_cycle_stage_id = stage.get('id', 'red_area_weapon_cycle')
        self._red_cycle_state = 'navigate_slot'
        self._red_cycle_slot_index = 0
        self._red_cycle_success_count = 0
        self._red_cycle_retry_count = 0
        self._red_cycle_sequence_name = None
        self._red_cycle_sequence_index = 0
        self._red_cycle_sequence_after = None
        self._red_cycle_wait_start = 0.0
        self._red_cycle_target_pose = None
        self._red_cycle_scan_start_pose = None
        self._red_cycle_scan_start_time = 0.0
        self._red_cycle_warned_missing_torque = False
        self.logger.info(
            f"Red Area weapon cycle started: slots={int(stage.get('slot_count', 6))}, "
            f"target_success={int(stage.get('target_success_count', 5))}"
        )

    def _red_cycle_slot_pose(self, stage):
        """Return the absolute pose of the current weapon slot."""
        start_wp = self.waypoints[stage['start_waypoint']]
        start_pose = start_wp['pose']
        spacing = float(stage.get('slot_spacing_m', 0.2))
        direction = float(start_pose.get('yaw', 0.0)) + float(stage.get('slot_direction_rad', 0.0))
        offset = self._red_cycle_slot_index * spacing
        return {
            'x': float(start_pose['x']) + offset * math.cos(direction),
            'y': float(start_pose['y']) + offset * math.sin(direction),
            'yaw': float(start_pose.get('yaw', 0.0)),
        }

    def _red_cycle_slot_yaw_tolerance(self, stage):
        if 'slot_yaw_tolerance_deg' in stage:
            return math.radians(float(stage.get('slot_yaw_tolerance_deg')))
        return float(stage.get('slot_yaw_tolerance', 0.05))

    def _update_red_cycle_navigate_slot(self, stage):
        """Navigate to the current weapon slot, then run prepare_sequence."""
        if self._red_cycle_target_pose is None:
            self._red_cycle_target_pose = self._red_cycle_slot_pose(stage)
            self._clear_navigation_state()
            self.logger.info(
                f"Red Area slot {self._red_cycle_slot_index + 1}: navigating to "
                f"({self._red_cycle_target_pose['x']:.3f}, {self._red_cycle_target_pose['y']:.3f})"
            )

        profile = self.profiles.get(stage.get('slot_profile', 'head_rack_speed'), {})
        pos_tol = float(stage.get('slot_pos_tolerance', 0.005))
        yaw_tol = self._red_cycle_slot_yaw_tolerance(stage)
        self._pub_target_pose(self._red_cycle_target_pose)
        arrived = self._drive_to_dynamic_pose(
            f"{stage.get('id', 'red_area_weapon_cycle')}_slot_{self._red_cycle_slot_index + 1}",
            self._red_cycle_target_pose,
            profile,
            pos_tol,
            yaw_tol,
        )
        if arrived:
            self._pub_zero_driving()
            self._red_cycle_target_pose = None
            self._start_red_cycle_sequence(stage, 'prepare_sequence', 'check_ir')

    def _start_red_cycle_sequence(self, stage, sequence_name, after):
        """Start a YAML action sequence used by the Red Area cycle."""
        sequence = stage.get(sequence_name, []) or []
        if not sequence:
            self._handle_red_cycle_sequence_after(stage, after)
            return
        self._red_cycle_sequence_name = sequence_name
        self._red_cycle_sequence_index = 0
        self._red_cycle_sequence_after = after
        self._red_cycle_wait_start = 0.0
        self._red_cycle_state = 'sequence'

    def _update_red_cycle_sequence(self, stage):
        sequence = stage.get(self._red_cycle_sequence_name, []) or []
        if self._red_cycle_sequence_index >= len(sequence):
            after = self._red_cycle_sequence_after
            self._red_cycle_sequence_name = None
            self._red_cycle_sequence_index = 0
            self._red_cycle_sequence_after = None
            self._red_cycle_wait_start = 0.0
            self._handle_red_cycle_sequence_after(stage, after)
            return

        step = sequence[self._red_cycle_sequence_index]
        stype = step.get('type', 'wait')
        if stype == 'arm':
            self._execute_arm(step)
            self._red_cycle_sequence_index += 1
            self._red_cycle_wait_start = 0.0
        elif stype == 'wait':
            duration = float(step.get('duration_s', 0.0))
            if self._red_cycle_wait_start == 0.0:
                self._red_cycle_wait_start = time.time()
                return
            if time.time() - self._red_cycle_wait_start >= duration:
                self._red_cycle_wait_start = 0.0
                self._red_cycle_sequence_index += 1
        elif stype == 'stop_chassis':
            self._execute_stop_chassis()
            self._red_cycle_sequence_index += 1
            self._red_cycle_wait_start = 0.0
        else:
            self.logger.warn(f"Unknown red cycle sequence step type '{stype}', skipping")
            self._red_cycle_sequence_index += 1
            self._red_cycle_wait_start = 0.0

    def _handle_red_cycle_sequence_after(self, stage, after):
        if after == 'check_ir':
            self._red_cycle_state = 'check_ir'
        elif after == 'verify_pickup':
            self._red_cycle_state = 'verify_pickup'
        elif after == 'retry_slot':
            self._weapon_ir_timeout_start = 0.0
            self._red_cycle_target_pose = None
            self._red_cycle_state = 'navigate_slot'
        elif after == 'next_slot':
            self._red_cycle_advance_slot_or_finish(stage)
        elif after == 'after_docking':
            if self._red_cycle_success_count >= int(stage.get('target_success_count', 5)):
                self.logger.info(
                    f"Target success count reached ({self._red_cycle_success_count}); finishing Red Area cycle"
                )
                self._start_red_cycle_final_sequence(stage)
            else:
                self._red_cycle_advance_slot_or_finish(stage)
        elif after == 'complete':
            self._red_cycle_state = 'complete'
        else:
            self.logger.warn(f"Unknown red cycle sequence continuation '{after}', finishing")
            self._start_red_cycle_final_sequence(stage)

    def _update_red_cycle_check_ir(self, stage):
        """Check IR at the current slot before pickup or local search."""
        ir_value = self._read_weapon_ir(stage)
        if ir_value is None:
            self._pub_zero_driving()
            if self._check_weapon_ir_timeout_fallback(stage):
                self._handle_red_cycle_miss(stage, 'IR timeout before pickup')
            return

        if ir_value:
            self.logger.info(f"Slot {self._red_cycle_slot_index + 1}: IR true, starting pickup")
            self._start_red_cycle_sequence(stage, 'pickup_sequence', 'verify_pickup')
            return

        search_cfg = stage.get('search', {}) or {}
        mode = search_cfg.get('mode', 'micro_sweep_10mm')
        if mode == 'micro_sweep_10mm':
            self._start_red_cycle_micro_sweep(stage)
        elif mode == 'none':
            self._handle_red_cycle_miss(stage, 'IR false and search disabled')
        else:
            self.logger.warn(f"Unknown red cycle search mode '{mode}', treating slot as missed")
            self._handle_red_cycle_miss(stage, f"unknown search mode {mode}")

    def _red_cycle_search_cfg(self, stage):
        return stage.get('micro_sweep', None) or stage.get('search', {}) or {}

    def _start_red_cycle_micro_sweep(self, stage):
        """Back up slightly, then scan through the current slot while reading IR."""
        cfg = self._red_cycle_search_cfg(stage)
        direction = float(cfg.get('direction_rad', 0.0))
        back_distance = float(cfg.get('back_distance_m', 0.01))
        yaw = self.current_pose['yaw'] + direction
        self._red_cycle_target_pose = {
            'x': self.current_pose['x'] - back_distance * math.cos(yaw),
            'y': self.current_pose['y'] - back_distance * math.sin(yaw),
            'yaw': self.current_pose['yaw'],
        }
        self._clear_navigation_state()
        self._red_cycle_state = 'micro_sweep_prepare'
        self.logger.info(
            f"Slot {self._red_cycle_slot_index + 1}: IR false, micro sweep back {back_distance:.3f}m"
        )

    def _update_red_cycle_micro_sweep_prepare(self, stage):
        ir_value = self._read_weapon_ir(stage)
        if ir_value is None:
            self._pub_zero_driving()
            if self._check_weapon_ir_timeout_fallback(stage):
                self._handle_red_cycle_miss(stage, 'IR timeout during micro sweep prepare')
            return
        if ir_value:
            self._pub_zero_driving()
            self._start_red_cycle_sequence(stage, 'pickup_sequence', 'verify_pickup')
            return

        cfg = self._red_cycle_search_cfg(stage)
        profile = self.profiles.get(cfg.get('profile', stage.get('slot_profile', 'head_rack_speed')), {})
        pos_tol = float(cfg.get('pos_tolerance', 0.003))
        yaw_tol = float(cfg.get('yaw_tolerance', 0.05))
        self._pub_target_pose(self._red_cycle_target_pose)
        arrived = self._drive_to_dynamic_pose(
            f"{stage.get('id', 'red_area_weapon_cycle')}_slot_{self._red_cycle_slot_index + 1}_micro_back",
            self._red_cycle_target_pose,
            profile,
            pos_tol,
            yaw_tol,
        )
        if arrived:
            self._pub_zero_driving()
            self._red_cycle_scan_start_pose = dict(self.current_pose)
            self._red_cycle_scan_start_time = time.time()
            self._red_cycle_state = 'micro_sweep_scan'

    def _update_red_cycle_micro_sweep_scan(self, stage):
        ir_value = self._read_weapon_ir(stage)
        if ir_value is None:
            self._pub_zero_driving()
            if self._check_weapon_ir_timeout_fallback(stage):
                self._handle_red_cycle_miss(stage, 'IR timeout during micro sweep scan')
            return
        if ir_value:
            self._pub_zero_driving()
            self.logger.info(f"Slot {self._red_cycle_slot_index + 1}: weapon found during micro sweep")
            self._start_red_cycle_sequence(stage, 'pickup_sequence', 'verify_pickup')
            return

        cfg = self._red_cycle_search_cfg(stage)
        back_distance = float(cfg.get('back_distance_m', 0.01))
        forward_distance = float(cfg.get('forward_distance_m', 0.01))
        max_distance_m = float(cfg.get('max_distance_m', back_distance + forward_distance))
        timeout_s = float(cfg.get('timeout_s', 2.0))
        elapsed = time.time() - self._red_cycle_scan_start_time
        moved = get_distance(self.current_pose, self._red_cycle_scan_start_pose)
        if elapsed > timeout_s or moved > max_distance_m:
            self.logger.warn(
                f"Slot {self._red_cycle_slot_index + 1}: micro sweep missed "
                f"elapsed={elapsed:.2f}s/{timeout_s:.2f}s, distance={moved:.3f}m/{max_distance_m:.3f}m"
            )
            self._handle_red_cycle_miss(stage, 'micro sweep missed')
            return

        direction = float(cfg.get('direction_rad', 0.0))
        speed = float(cfg.get('speed_mps', 0.015))
        self._pub_driving_body(speed * math.cos(direction), speed * math.sin(direction), 0.0)

    def _update_red_cycle_verify_pickup(self, stage):
        """Verify that lift-high pickup still sees IR before counting success."""
        ir_value = self._read_weapon_ir(stage)
        if ir_value is None:
            self._pub_zero_driving()
            if self._check_weapon_ir_timeout_fallback(stage):
                self._handle_red_cycle_miss(stage, 'IR timeout after lift high')
            return

        if ir_value:
            self._red_cycle_success_count += 1
            self._red_cycle_retry_count = 0
            self.logger.info(
                f"Slot {self._red_cycle_slot_index + 1}: pickup verified, "
                f"success_count={self._red_cycle_success_count}"
            )
            self._red_cycle_state = 'docking'
        else:
            self._handle_red_cycle_miss(stage, 'IR false after lift high')

    def _handle_red_cycle_miss(self, stage, reason):
        """Handle a failed pickup attempt with one retry per configured slot."""
        self._pub_zero_driving()
        max_retry = int(stage.get('max_retry_per_slot', 1))
        slot_no = self._red_cycle_slot_index + 1
        if self._red_cycle_retry_count < max_retry:
            self._red_cycle_retry_count += 1
            self.logger.warn(
                f"Slot {slot_no}: pickup missed ({reason}); retry "
                f"{self._red_cycle_retry_count}/{max_retry}"
            )
            self._start_red_cycle_sequence(stage, 'miss_sequence', 'retry_slot')
        else:
            self.logger.warn(f"Slot {slot_no}: pickup missed after retry ({reason}); moving on")
            self._start_red_cycle_sequence(stage, 'miss_sequence', 'next_slot')

    def _update_red_cycle_docking(self, stage):
        """Wait for torque contact, then release gripper for docking handoff."""
        self._pub_zero_driving()
        topic = stage.get('docking_torque_topic', '/damiao_feedback')
        field = stage.get('docking_torque_field', 'motor_5_tau')
        threshold = float(stage.get('docking_torque_abs_threshold_nm', 1.3))
        sensor_data = self.sensor_cache.get(topic)
        if sensor_data is None:
            if not self._red_cycle_warned_missing_torque:
                self.logger.warn(f"No torque data on {topic}; Red Area docking is waiting")
                self._red_cycle_warned_missing_torque = True
            return
        self._red_cycle_warned_missing_torque = False
        actual = float(sensor_data.get(field, 0.0))
        if abs(actual) > threshold:
            self.logger.info(
                f"Docking torque detected: abs({field})={abs(actual):.3f}Nm > {threshold:.3f}Nm"
            )
            self._start_red_cycle_sequence(stage, 'dock_release_sequence', 'after_docking')

    def _red_cycle_advance_slot_or_finish(self, stage):
        target_success = int(stage.get('target_success_count', 5))
        slot_count = int(stage.get('slot_count', 6))
        if self._red_cycle_success_count >= target_success:
            self._start_red_cycle_final_sequence(stage)
            return
        if self._red_cycle_slot_index >= slot_count - 1:
            self.logger.warn(
                f"Red Area processed {slot_count} slots with success_count="
                f"{self._red_cycle_success_count}/{target_success}; finishing"
            )
            self._start_red_cycle_final_sequence(stage)
            return

        self._red_cycle_slot_index += 1
        self._red_cycle_retry_count = 0
        self._red_cycle_target_pose = None
        self._weapon_ir_timeout_start = 0.0
        self._clear_navigation_state()
        self._red_cycle_state = 'navigate_slot'

    def _start_red_cycle_final_sequence(self, stage):
        self._pub_zero_driving()
        self._start_red_cycle_sequence(stage, 'final_sequence', 'complete')

    def _finish_red_cycle(self, stage):
        """Finish without generic terminate so the final arm pose is preserved."""
        self._pub_zero_driving()
        self.phase = 'done'
        self.logger.info(
            f"Red Area weapon cycle complete: success_count={self._red_cycle_success_count}, "
            f"last_slot={self._red_cycle_slot_index + 1}"
        )
        self._clear_red_cycle_state()

    # ------------------------------------------------------------------
    # Stage: conditional
    # ------------------------------------------------------------------

    # _update_conditional() is superseded by _update_condition() above.
    # The old name is aliased in update() dispatch for backward compat.

    def _jump_to(self, stage_id):
        """Jump to a stage by ID."""
        for i, s in enumerate(self.stages):
            if s.get('id') == stage_id:
                self.stage_index = i
                self._seq_step_index = 0
                self._parallel_active = []
                self._wait_duration = 0.0
                self._active_stage_id = None  # force _on_stage_enter on target
                self._clear_navigation_state()
                self._clear_weapon_state()
                self._clear_red_cycle_state()
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
    # Stage: stop_chassis
    # ------------------------------------------------------------------

    def _execute_stop_chassis(self):
        """Publish an explicit zero chassis command."""
        self._pub_zero_driving()

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
            pneu_pairs = []
            for name, act in self.actuators.items():
                if act.get('type') == 'pneumatic':
                    pneu_pairs.append(f"{name}:0")
            if pneu_pairs:
                msg = String()
                msg.data = ",".join(pneu_pairs)
                self.pub_pneu.publish(msg)
        self._current_arm_state = {}        # clear tracked state
        self._last_arm_publish_time = 0.0
        self.phase = 'terminated'
        self.logger.info("Mission terminated")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _advance_stage(self):
        self._clear_navigation_state()
        self._clear_weapon_state()
        self._clear_red_cycle_state()
        self._active_stage_id = None  # force _on_stage_enter on next stage
        self.stage_index += 1

    def _clear_navigation_state(self):
        """Clear per-stage navigation state when leaving or jumping stages."""
        self._nav_target_wp = None
        self._nav_profile = {}
        self._nav_from_pose = None
        self._active_nav_stage_id = None
        self._nav_start_time = 0.0
        self.arrived_counter = 0

    def _clear_weapon_state(self):
        """Clear per-stage weapon pickup state when leaving or jumping stages."""
        self._weapon_stage_id = None
        self._weapon_state = 'idle'
        self._weapon_slot_index = 0
        self._weapon_pickup_step_index = 0
        self._weapon_step_target_pose = None
        self._weapon_scan_start_pose = None
        self._weapon_scan_start_time = 0.0
        self._weapon_wait_start = 0.0
        self._weapon_sequence_action_index = -1
        self._weapon_warned_missing_ir = False
        self._weapon_warned_ir_timeout = False
        self._weapon_ir_timeout_start = 0.0

    def _clear_red_cycle_state(self):
        """Clear per-stage Red Area weapon cycle state."""
        self._red_cycle_stage_id = None
        self._red_cycle_state = 'idle'
        self._red_cycle_slot_index = 0
        self._red_cycle_success_count = 0
        self._red_cycle_retry_count = 0
        self._red_cycle_sequence_name = None
        self._red_cycle_sequence_index = 0
        self._red_cycle_sequence_after = None
        self._red_cycle_wait_start = 0.0
        self._red_cycle_target_pose = None
        self._red_cycle_scan_start_pose = None
        self._red_cycle_scan_start_time = 0.0
        self._red_cycle_warned_missing_torque = False

    def set_pose(self, pose):
        self.current_pose = pose

    def reset(self):
        self.stage_index = 0
        self.phase = 'idle'
        self.arrived_counter = 0
        self._seq_step_index = 0
        self._parallel_active = []
        self._wait_duration = 0.0
        self._current_arm_state = {}
        self._last_arm_publish_time = 0.0
        self._active_stage_id = None
        self._clear_navigation_state()
        self._clear_weapon_state()
        self._clear_red_cycle_state()
