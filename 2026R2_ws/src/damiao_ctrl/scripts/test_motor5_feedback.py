#!/usr/bin/env python3
"""
Test damiao motor 5 POS_VEL feedback at 50 Hz.

Sends pos=0, random speed [-0.3, 0.3] to motor 5 in POS_VEL mode,
reads back feedback after each command, and prints the raw + decoded
values for verification against docs/damiao_can_feedback_protocol.md.
"""

import os
import sys
import time
import random
import struct
import numpy as np

# Add ROS2 workspace to path so we can import damiao_ctrl
WS = os.path.expanduser("~/Robocon2026_r2/2026R2_ws")
sys.path.insert(0, os.path.join(WS, "install", "damiao_ctrl", "lib", "python3.12", "site-packages"))

import serial
from damiao_ctrl.DM_CAN import Motor, MotorControl, Control_Type, DM_Motor_Type

DEVICE = "/dev/damiao_can"
MOTOR_ID = 6
TEST_DURATION_S = 10
RATE_HZ = 50
INTERVAL_S = 1.0 / RATE_HZ

# Motor 5 torque range from Limit_Param
# DM3519 = type 9 → Limit_Param[9] = [12.5, 280, 1]
# Per docs: T_MIN=-8, T_MAX=8 for DM3519 output torque
# We'll print both for comparison
LIMIT_PARAM = [12.5, 280, 1]
P_MIN, P_MAX = -LIMIT_PARAM[0], LIMIT_PARAM[0]
V_MIN, V_MAX = -LIMIT_PARAM[1], LIMIT_PARAM[1]
T_MIN_CURRENT, T_MAX_CURRENT = -LIMIT_PARAM[2], LIMIT_PARAM[2]
# Per docs:
T_MIN_DOCS, T_MAX_DOCS = -8.0, 8.0


def uint_to_float(x: int, x_min: float, x_max: float, bits: int) -> float:
    span = x_max - x_min
    max_int = float((1 << bits) - 1)
    return float(x) * span / max_int + x_min


def parse_feedback_manual(data: bytes):
    """Parse raw 8-byte feedback per docs/damiao_can_feedback_protocol.md."""
    if len(data) < 6:
        return None

    motor_id = data[0] & 0x0F
    err = (data[0] >> 4) & 0x0F

    pos_raw = (data[1] << 8) | data[2]
    vel_raw = (data[3] << 4) | (data[4] >> 4)
    tq_raw = ((data[4] & 0x0F) << 8) | data[5]

    t_mos = data[6] if len(data) > 6 else 0
    t_rotor = data[7] if len(data) > 7 else 0

    pos_cur = uint_to_float(pos_raw, P_MIN, P_MAX, 16)
    vel_cur = uint_to_float(vel_raw, V_MIN, V_MAX, 12)
    tq_cur = uint_to_float(tq_raw, T_MIN_CURRENT, T_MAX_CURRENT, 12)
    tq_docs = uint_to_float(tq_raw, T_MIN_DOCS, T_MAX_DOCS, 12)

    return {
        "id": motor_id, "err": err,
        "pos_raw": pos_raw, "vel_raw": vel_raw, "tq_raw": tq_raw,
        "pos_rad": pos_cur, "vel_rad_s": vel_cur,
        "tq_cur_nm": tq_cur, "tq_docs_nm": tq_docs,
        "t_mos": t_mos, "t_rotor": t_rotor,
        "data_hex": data[:8].hex(" "),
    }


def main():
    if not os.path.exists(DEVICE):
        print(f"ERROR: {DEVICE} not found")
        sys.exit(1)

    print(f"Opening {DEVICE}...")
    ser = serial.Serial(DEVICE, 921600, timeout=0.01)
    time.sleep(0.5)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    mc = MotorControl(ser)
    motor = Motor(DM_Motor_Type.DM3519, MOTOR_ID, 0x00)
    mc.addMotor(motor)

    # --- Init: switch to POS_VEL, set zero, enable ---
    print(f"--- Init motor {MOTOR_ID} ---")

    # Read CTRL_MODE
    motor.temp_param_dict.pop(0x0A, None)
    mc.read_param(motor, 0x0A)
    time.sleep(0.3)
    mc.recv()
    mode = motor.temp_param_dict.get(0x0A)
    print(f"CTRL_MODE read: {mode} (int={int(mode) if mode else 'None'})")

    if mode != 2:
        print(f"Switching to POS_VEL...")
        mc.switchControlMode(motor, Control_Type.POS_VEL)
        time.sleep(0.3)
        mc.recv()
        motor.temp_param_dict.pop(0x0A, None)
        mc.read_param(motor, 0x0A)
        time.sleep(0.3)
        mc.recv()
        mode = motor.temp_param_dict.get(0x0A)
        print(f"CTRL_MODE after switch: {mode}")

    mc.set_zero_position(motor)
    time.sleep(0.1)
    mc.enable(motor)
    time.sleep(0.1)

    # Warm-up: send a stop command and read feedback
    mc.control_Pos_Vel(motor, 0.0, 0.0)
    time.sleep(0.1)
    mc.recv()
    print(f"Enable state: {motor.isEnable}, state_code: {motor.state_code}")

    if not motor.isEnable:
        print("WARNING: motor not enabled! Sending commands anyway...")

    # --- Test loop ---
    print(f"\n--- Test: 50 Hz random speed to motor {MOTOR_ID} for {TEST_DURATION_S}s ---")
    print(f"{'t_s':>6s} {'cmd_spd':>8s} {'data_hex':>30s} {'id':>3s} {'err':>3s} {'pos_r':>8s} {'vel_r':>8s} {'tq_cur':>8s} {'tq_docs':>8s} {'en':>3s} {'raw8':>30s}")
    print("-" * 160)

    start = time.monotonic()
    loop_count = 0
    got_feedback = 0

    while time.monotonic() - start < TEST_DURATION_S:
        loop_start = time.monotonic()

        speed = random.uniform(-0.3, 0.3)
        mc.control_Pos_Vel(motor, 0.0, speed)
        loop_count += 1

        # Check for CAN frames in last_can_frames
        if mc.last_can_frames:
            got_feedback += 1
            for frame in mc.last_can_frames[-2:]:  # show last 2 frames max
                data = frame["data"]
                raw = frame["raw_data"]
                fb = parse_feedback_manual(data[:8])

                t_elapsed = time.monotonic() - start
                if fb:
                    print(
                        f"{t_elapsed:6.2f} {speed:+8.4f} {fb['data_hex']:>30s} {fb['id']:3d} {fb['err']:3d} "
                        f"{fb['pos_rad']:+8.4f} {fb['vel_rad_s']:+8.4f} {fb['tq_cur_nm']:+8.4f} {fb['tq_docs_nm']:+8.4f} "
                        f"{'Y' if motor.isEnable else 'N':>3s} {raw.hex(' '):>30s}"
                    )

        # Maintain 50 Hz
        elapsed = time.monotonic() - loop_start
        if elapsed < INTERVAL_S:
            time.sleep(INTERVAL_S - elapsed)

    # --- Summary ---
    print(f"\n--- Done ---")
    print(f"Loops: {loop_count}, Got feedback: {got_feedback}/{loop_count}")
    print(f"Motor state: q={motor.state_q:.4f}, dq={motor.state_dq:.4f}, tau={motor.state_tau:.4f}, enabled={motor.isEnable}")

    # Final comparison: what motor.state_tau says vs manual parse
    print(f"\n--- Decode check ---")
    print(f"motor.state_q   = {motor.state_q:.6f}  (via DM_CAN, Limit_Param[9])")
    print(f"motor.state_dq  = {motor.state_dq:.6f}")
    print(f"motor.state_tau = {motor.state_tau:.6f}")
    print(f"Limit_Param[9] for DM3519: P=±{LIMIT_PARAM[0]}, V=±{LIMIT_PARAM[1]}, T=±{LIMIT_PARAM[2]}")
    print(f"Docs suggest:         P=±12.5, V=±45, T=±8")

    ser.close()


if __name__ == "__main__":
    main()
