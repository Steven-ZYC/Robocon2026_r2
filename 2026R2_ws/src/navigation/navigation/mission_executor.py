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
        self._weapon_warned_missing_ir = False
        self._weapon_warned_ir_timeout = False

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

            elif stype == 'weapon_head_pickup':
                mode = s.get('search_mode', 'scan_until_ir')
                if mode not in ('scan_until_ir', 'step_0p2m'):
                    self.logger.warn(f"[{sid}] unknown search_mode '{mode}'")
                for step in s.get('pickup_sequence', []):
                    if step.get('type') == 'arm':
                        for key in step:
                            if key == 'type':
                                continue
                            if key not in self.actuators:
                                self.logger.warn(f"[{sid}] actuator '{key}' not defined")
                    elif step.get('type') == 'verify_ir':
                        for branch_key in ('on_true', 'on_false', 'on_match', 'on_mismatch'):
                            target_id = step.get(branch_key)
                            if target_id and target_id not in ('continue', 'advance', 'terminate') and target_id not in stage_ids:
                                self.logger.warn(f"[{sid}] {branch_key} stage '{target_id}' not found")

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
        elif stype == 'weapon_head_pickup':
            self._update_weapon_head_pickup(stage)
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

        self._begin_navigate_stage(stage, end_pose, profile)
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
    # Stage: arm
    # ------------------------------------------------------------------

    def _execute_arm(self, stage):
        """Translate semantic arm commands → arm/joint_navigation + arm/pneu_navigation.

        arm/joint_navigation uses triplet format:
          [motor_id, position_rad, speed_rad_s, ...]

        motor_id comes directly from the actuator definition in YAML.
        position is looked up from the actuator's positions table.
        speed is read from the actuator's speed field (default 3.0 rad/s).
        """
        joint_triplets = []          # [motor_id, pos, speed, ...]
        pneu_pairs = []              # ["name:value", ...]

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
                target_val = act['states'].index(value)
                pneu_pairs.append(f"{name}:{target_val}")

        if joint_triplets:
            self._pub_joint_cmd(joint_triplets)
        if pneu_pairs:
            self._pub_pneu_cmd(",".join(pneu_pairs))

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
        self._weapon_warned_missing_ir = False
        self._weapon_warned_ir_timeout = False
        self.logger.info(f"Weapon pickup [{stage_id}] started")

    def _update_weapon_checking(self, stage):
        """Check the current rack slot and choose the configured search mode."""
        ir_value = self._read_weapon_ir(stage)
        if ir_value is None:
            self._pub_zero_driving()
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
        else:
            self.logger.warn(f"Unknown pickup_sequence step type '{stype}', skipping")
            self._weapon_pickup_step_index += 1
            self._weapon_wait_start = 0.0

    def _update_weapon_verify_ir_step(self, stage, step):
        """Re-check IR during pickup_sequence and optionally branch the mission.

        This keeps point-specific recovery in YAML: the executor only reads the
        configured IR field, applies the same timeout/CRC protection as the
        search phase, and follows the branch target selected by the mission.
        """
        ir_value = self._read_weapon_ir(stage)
        if ir_value is None:
            self._pub_zero_driving()
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
        sensor data is missing/stale/CRC-invalid and the chassis must not move.
        """
        topic = stage.get('ir_topic', '/arduino/raw_sensor_data')
        field = stage.get('ir_field', 'weapon_head_detected')
        sensor_data = self.sensor_cache.get(topic)
        if sensor_data is None:
            if not self._weapon_warned_missing_ir:
                self.logger.warn(f"No IR sensor data on {topic}; weapon pickup is holding position")
                self._weapon_warned_missing_ir = True
            return None

        require_crc_valid = bool(stage.get('require_crc_valid', True))
        if require_crc_valid and not bool(sensor_data.get('crc_valid', False)):
            if not self._weapon_warned_ir_timeout:
                self.logger.warn("Latest IR packet is CRC-invalid; weapon pickup is holding position")
                self._weapon_warned_ir_timeout = True
            return None

        stamp = sensor_data.get('_stamp')
        timeout_s = float(stage.get('ir_timeout_s', 0.5))
        if stamp is not None and time.monotonic() - float(stamp) > timeout_s:
            if not self._weapon_warned_ir_timeout:
                self.logger.warn(f"IR sensor timeout ({timeout_s:.2f}s); weapon pickup is holding position")
                self._weapon_warned_ir_timeout = True
            return None

        self._weapon_warned_missing_ir = False
        self._weapon_warned_ir_timeout = False
        return bool(sensor_data.get(field, False))

    def _drive_to_dynamic_pose(self, stage_id, target_pose, profile, pos_tol, yaw_tol):
        """Small-pose PID used by step_0p2m dynamic slot targets."""
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
        vx_body = k_p_x * ex_body
        vy_body = k_p_y * ey_body

        max_body_x = float(profile.get('max_body_x_mps', DEFAULT_MAX_LATERAL_MPS))
        max_body_y = float(profile.get('max_body_y_mps', DEFAULT_MAX_LATERAL_MPS))
        vx_body = max(-max_body_x, min(max_body_x, vx_body))
        vy_body = max(-max_body_y, min(max_body_y, vy_body))

        k_heading = float(profile.get('k_heading_p', TRACKER_K_HEADING_P))
        max_omega = float(profile.get('yaw_rate_rps', 1.5))
        omega = max(-max_omega, min(max_omega, k_heading * yaw_err_signed))
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
        elif op == 'abs_gt':
            result = abs(actual) > threshold
        elif op == 'abs_gte':
            result = abs(actual) >= threshold
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
                self._clear_navigation_state()
                self._clear_weapon_state()
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
            pneu_pairs = []
            for name, act in self.actuators.items():
                if act.get('type') == 'pneumatic':
                    pneu_pairs.append(f"{name}:0")
            if pneu_pairs:
                msg = String()
                msg.data = ",".join(pneu_pairs)
                self.pub_pneu.publish(msg)
        self.phase = 'terminated'
        self.logger.info("Mission terminated")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _advance_stage(self):
        self._clear_navigation_state()
        self._clear_weapon_state()
        self.stage_index += 1

    def _clear_navigation_state(self):
        """Clear per-stage navigation state when leaving or jumping stages."""
        self._nav_target_wp = None
        self._nav_profile = {}
        self._nav_from_pose = None
        self._active_nav_stage_id = None
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
        self._weapon_warned_missing_ir = False
        self._weapon_warned_ir_timeout = False

    def set_pose(self, pose):
        self.current_pose = pose

    def reset(self):
        self.stage_index = 0
        self.phase = 'idle'
        self.arrived_counter = 0
        self._seq_step_index = 0
        self._parallel_active = []
        self._wait_duration = 0.0
        self._clear_navigation_state()
        self._clear_weapon_state()
