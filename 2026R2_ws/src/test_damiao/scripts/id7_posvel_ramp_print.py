#!/usr/bin/env python3
"""Hold one Damiao motor at a fixed POS_VEL target and print feedback.

This hardware test bypasses ROS2 topics and talks to the USB-CAN bridge through
the package's DM_CAN helper.  It sends one POS_VEL command per second:
the target position is fixed at 35 rad by default, while velocity stays at
1 rad/s.  Each cycle prints returned CAN data bytes plus the decoded motor
state stored by DM_CAN.
"""

import argparse
import os
import sys
import time
from pathlib import Path

import serial


SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_SRC = SCRIPT_DIR.parent
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from test_damiao.DM_CAN import Control_Type, DM_Motor_Type, Motor, MotorControl  # noqa: E402


DEVICE_ID = "usb-HDSC_CDC_Device_00000000050C-if00"
BAUDRATE = 921600
SERIAL_TIMEOUT = 0.01
DEFAULT_MOTOR_ID = 5
TARGET_POSITION_RAD = 0.0
VELOCITY_RAD_S = 1.0
COMMAND_PERIOD_S = 1.0
RECV_SETTLE_S = 0.05
ENABLE_FEEDBACK_TIMEOUT_S = 0.5
NO_FEEDBACK_WARN_CYCLES = 3
SERIAL_OPEN_SETTLE_S = 1.0


def find_device_port(device_id):
    """Return the real serial path for the configured USB-CAN by-id name."""
    by_id_dir = "/dev/serial/by-id/"
    try:
        for entry in os.listdir(by_id_dir):
            if device_id in entry:
                return os.path.realpath(os.path.join(by_id_dir, entry))
    except FileNotFoundError:
        return None
    return None


def parse_args():
    """Parse small hardware-test overrides without turning this into a full node."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motor-id", type=int, default=DEFAULT_MOTOR_ID, help="Damiao CAN ID to test.")
    parser.add_argument("--cycles", type=int, default=0, help="0 means run until Ctrl-C.")
    parser.add_argument("--target-position", type=float, default=TARGET_POSITION_RAD, help="Fixed target position in rad.")
    parser.add_argument(
        "--switch-mode",
        action="store_true",
        help="Write CTRL_MODE=POS_VEL before enable. Default skips register writes.",
    )
    return parser.parse_args()


def print_return_data(cycle, target_position, motor_control, motor):
    """Print raw returned CAN data and the latest decoded DM_CAN motor state."""
    if motor_control.last_can_frames:
        for frame in motor_control.last_can_frames:
            raw_note = ""
            if frame.get("raw_data") and frame["raw_data"] != frame["data"]:
                raw_note = f" raw={frame['raw_data'].hex(' ')}"
            print(
                f"[{cycle:04d}] return can_id=0x{frame['can_id']:03X} "
                f"data={frame['data'].hex(' ')}{raw_note}"
            )
    else:
        print(f"[{cycle:04d}] return data=<none>")

    print(
        f"[{cycle:04d}] target: position={target_position:.3f} rad, "
        f"velocity={VELOCITY_RAD_S:.3f} rad/s | decoded: "
        f"q={motor.state_q:.6f} rad, dq={motor.state_dq:.6f} rad/s, "
        f"tau={motor.state_tau:.6f} Nm, enable={motor.isEnable}"
    )


def collect_feedback(motor_control, duration_s):
    """Drain USB-CAN feedback for a short window and keep all decoded frames."""
    deadline = time.monotonic() + duration_s
    frames = []
    while time.monotonic() < deadline:
        motor_control.recv()
        frames.extend(motor_control.last_can_frames)
        time.sleep(SERIAL_TIMEOUT)
    motor_control.last_can_frames = frames
    return frames


def warn_no_feedback(motor_id):
    """Print the practical checks when serial is open but CAN gives no frames."""
    print(
        "WARNING: USB-CAN serial is open, but no CAN feedback was received from "
        f"motor ID {motor_id}."
    )
    print("Check motor power, CANH/CANL wiring, common GND, CAN bitrate, termination, and actual motor CAN ID.")


def main():
    """Open USB-CAN, send POS_VEL commands, and print raw plus decoded feedback."""
    args = parse_args()
    port = find_device_port(DEVICE_ID)
    if port is None:
        raise RuntimeError(f"USB-CAN device containing '{DEVICE_ID}' was not found.")

    print(f"Using serial port: {port}")
    print(f"Press Ctrl-C to stop. The script disables motor ID {args.motor_id} before exit.")

    motor = Motor(DM_Motor_Type.DMH3510, args.motor_id, 0x00)
    position = args.target_position

    with serial.Serial(port, BAUDRATE, timeout=SERIAL_TIMEOUT) as ser:
        print(f"Waiting {SERIAL_OPEN_SETTLE_S:.1f}s for USB-CAN serial startup...")
        time.sleep(SERIAL_OPEN_SETTLE_S)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        motor_control = MotorControl(ser)
        motor_control.addMotor(motor)
        if args.switch_mode:
            print("Writing CTRL_MODE=POS_VEL before enable...")
            motor_control.switchControlMode(motor, Control_Type.POS_VEL)
        else:
            print(f"Skipping CTRL_MODE write. Assuming motor ID {args.motor_id} is already in POS_VEL mode.")
        motor_control.enable(motor)
        print(f"Motor ID {args.motor_id} enable command sent. Waiting for first feedback frame...")

        # Send one POS_VEL command after enable. Damiao motors usually
        # return state feedback after a control command, which lets us verify
        # that CAN wiring and motor ID are actually responding before holding.
        motor_control.control_Pos_Vel(motor, position, VELOCITY_RAD_S)
        initial_frames = collect_feedback(motor_control, ENABLE_FEEDBACK_TIMEOUT_S)
        if initial_frames:
            print("First feedback frame received. Starting fixed POS_VEL command loop.")
            print_return_data(0, position, motor_control, motor)
        else:
            warn_no_feedback(args.motor_id)
            print("Continuing command loop so wiring fixes can be tested without restarting the script.")

        cycle = 0
        no_feedback_cycles = 0
        try:
            while args.cycles <= 0 or cycle < args.cycles:
                motor_control.last_can_frames = []
                motor_control.control_Pos_Vel(motor, position, VELOCITY_RAD_S)
                returned_frames = list(motor_control.last_can_frames)

                # Some USB-CAN replies arrive shortly after the write. Drain once
                # more so the printed decoded state follows the latest response.
                time.sleep(RECV_SETTLE_S)
                motor_control.recv()
                returned_frames.extend(motor_control.last_can_frames)
                motor_control.last_can_frames = returned_frames

                print_return_data(cycle, position, motor_control, motor)
                if returned_frames:
                    no_feedback_cycles = 0
                else:
                    no_feedback_cycles += 1
                    if no_feedback_cycles == NO_FEEDBACK_WARN_CYCLES:
                        warn_no_feedback(args.motor_id)
                cycle += 1
                time.sleep(COMMAND_PERIOD_S)
        except KeyboardInterrupt:
            print("\nInterrupted by user.")
        finally:
            motor_control.disable(motor)
            print(f"Motor ID {args.motor_id} disabled.")


if __name__ == "__main__":
    main()
