#!/usr/bin/env python3
"""
ROS 2 bridge for the Arduino Mega pneumatics + IR sensor controller.

Expected Arduino INO protocol
-----------------------------
Host -> Arduino:
    [0,0,0]\n
    [1,0,1]\n
    STATUS\n
    OFF\n
Arduino -> Host:
    <STATE,t:123456,pneu:[1,0,1],ir:1,*5A>\n
The checksum is XOR/LRC over the payload between '<' and ',*'.
Example payload:
    STATE,t:123456,pneu:[1,0,1],ir:1

ROS 2 interface
---------------
Subscribe:
    arm/pneu_ctrl    std_msgs/msg/Int8MultiArray
                         data: [arm_gripper, arm_lift, arm_stopper]

Publish:
    arm/pneu_ack        std_msgs/msg/Int8MultiArray
    arm/ir_status       std_msgs/msg/Bool
    arm/pneu_raw_frame  std_msgs/msg/String
"""

import os
import re
import time
import termios
from typing import Iterable, List, Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import String

import serial
from serial import SerialException


DEFAULT_PNEU_NAMES = ["arm_gripper", "arm_lift", "arm_stopper"]
NUM_PNEU = 3

STATE_RE = re.compile(r"^STATE,t:(\d+),pneu:\[([01]),([01]),([01])\],ir:([01])$")
ACK_RE = re.compile(r"^ACK,pneu:\[([01]),([01]),([01])\]$")
ERR_RE = re.compile(r"^ERR,(.+)$")
BOOT_RE = re.compile(r"^BOOT,ready$")


def find_device_port(device_id: str) -> Optional[str]:
    """Resolve a device path or search /dev/serial/by-id/ by substring."""
    if device_id.startswith("/dev/"):
        return device_id if os.path.exists(device_id) else None

    by_id_dir = "/dev/serial/by-id/"
    try:
        for entry in os.listdir(by_id_dir):
            if device_id in entry:
                return os.path.realpath(os.path.join(by_id_dir, entry))
    except FileNotFoundError:
        return None
    return None


class ArmArduinoNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_arduino_interface")

        # Serial parameters. INO baud rate is 115200.
        self.declare_parameter("port", "/dev/arm_arduino")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("arduino_reset_wait_s", 2.0)

        # Topic parameters.
        self.declare_parameter("command_topic", "arm/pneu_ctrl")
        self.declare_parameter("pneu_ack_topic", "arm/pneu_ack")
        self.declare_parameter("ir_status_topic", "arm/ir_status")
        self.declare_parameter("raw_frame_topic", "arm/pneu_raw_frame")

        # INO has COMMAND_TIMEOUT_MS = 200.
        # Therefore this node keeps sending the latest valid command repeatedly.
        # 20 Hz sends every 50 ms, safely faster than 200 ms.
        self.declare_parameter("send_rate_hz", 20.0)
        self.declare_parameter("read_rate_hz", 100.0)

        # Match the original INO default: all pneumatics OFF.
        self.declare_parameter("default_pneu", [0, 0, 0])

        self.port = str(self.get_parameter("port").value)
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.arduino_reset_wait_s = float(
            self.get_parameter("arduino_reset_wait_s").value
        )

        command_topic = str(self.get_parameter("command_topic").value)
        pneu_ack_topic = str(self.get_parameter("pneu_ack_topic").value)
        ir_status_topic = str(self.get_parameter("ir_status_topic").value)
        raw_frame_topic = str(self.get_parameter("raw_frame_topic").value)

        send_rate_hz = float(self.get_parameter("send_rate_hz").value)
        read_rate_hz = float(self.get_parameter("read_rate_hz").value)

        default_pneu = self.get_parameter("default_pneu").value
        default_bits = self.int8_array_to_bits(default_pneu, warn=False)
        if default_bits is None:
            default_bits = [0, 0, 0]
        self.current_cmd = default_bits

        self.serial_port: Optional[serial.Serial] = None
        self.rx_buffer = bytearray()

        self.pneu_ack_pub = self.create_publisher(
            Int8MultiArray,
            pneu_ack_topic,
            10,
        )
        self.ir_status_pub = self.create_publisher(Bool, ir_status_topic, 10)
        self.raw_frame_pub = self.create_publisher(String, raw_frame_topic, 10)

        self.command_sub = self.create_subscription(
            Int8MultiArray,
            command_topic,
            self.on_pneu_command,
            10,
        )

        self.open_serial()

        self.send_timer = self.create_timer(
            1.0 / send_rate_hz,
            self.send_current_command,
        )
        self.read_timer = self.create_timer(
            1.0 / read_rate_hz,
            self.read_serial,
        )
        self.reconnect_timer = self.create_timer(1.0, self.reconnect_if_needed)

        self.get_logger().info(
            "arm_arduino_interface started. "
            f"Subscribe: {command_topic}. "
            f"Publish: {pneu_ack_topic}, {ir_status_topic}, {raw_frame_topic}. "
            f"Default command: {self.current_cmd}. "
            f"Serial: {self.port} @ {self.baud_rate}."
        )

    def open_serial(self) -> None:
        if self.serial_port is not None and self.serial_port.is_open:
            return

        actual_port = find_device_port(self.port)
        if actual_port is None:
            self.get_logger().warn(
                f"Device {self.port} not found. "
                "Set -p port:=/dev/xxx or install udev rule for /dev/arm_arduino"
            )
            return

        self.get_logger().info(
            f"Opening {actual_port} for {self.port} @ {self.baud_rate}"
        )

        try:
            self.serial_port = serial.Serial(
                port=actual_port,
                baudrate=self.baud_rate,
                timeout=0,
                write_timeout=0.05,
            )

            # Match arduino_sensor_driver: do not drop DTR on close, so quick
            # relaunches are less likely to force another Arduino reset.
            try:
                attrs = termios.tcgetattr(self.serial_port.fd)
                attrs[2] &= ~termios.HUPCL
                termios.tcsetattr(self.serial_port.fd, termios.TCSANOW, attrs)
            except (termios.error, AttributeError, OSError) as exc:
                self.get_logger().warn(
                    f"Could not clear HUPCL for {actual_port}: {exc}",
                    throttle_duration_sec=5.0,
                )

            # Opening Arduino USB serial often resets the Mega. Wait for boot,
            # then discard any half STATE frame or startup timeout already queued.
            if self.arduino_reset_wait_s > 0:
                time.sleep(self.arduino_reset_wait_s)

            self.serial_port.reset_input_buffer()
            self.rx_buffer.clear()
            self._write_current_command_once()

            self.get_logger().info(
                f"Opened serial port: {actual_port} @ {self.baud_rate} baud"
            )

        except SerialException as exc:
            self.serial_port = None
            self.get_logger().warn(f"Could not open serial port {actual_port}: {exc}")

    def reconnect_if_needed(self) -> None:
        if self.serial_port is None or not self.serial_port.is_open:
            self.open_serial()

    def on_pneu_command(self, msg: Int8MultiArray) -> None:
        bits = self.int8_array_to_bits(msg.data, warn=True)
        if bits is None:
            return

        self.current_cmd = bits

    def int8_array_to_bits(
        self,
        data: Iterable[int],
        warn: bool = True,
    ) -> Optional[List[int]]:
        values = list(data)

        if len(values) < NUM_PNEU:
            if warn:
                self.get_logger().warn(
                    "pneu_command needs 3 values in this order: "
                    "[arm_gripper, arm_lift, arm_stopper]. "
                    f"Got: {values}"
                )
            return None

        if len(values) > NUM_PNEU and warn:
            self.get_logger().warn(
                f"pneu_command has extra values. Using first 3 only. Got: {values}"
            )

        bits: List[int] = []

        for index in range(NUM_PNEU):
            value = int(values[index])

            if warn and value not in (0, 1):
                self.get_logger().warn(
                    f"{DEFAULT_PNEU_NAMES[index]} command is {value}, "
                    "expected 0 or 1. Clamping to 0/1."
                )

            bits.append(1 if value > 0 else 0)

        return bits

    def send_current_command(self) -> None:
        if self.serial_port is None or not self.serial_port.is_open:
            return

        self._write_current_command_once()

    def _write_current_command_once(self) -> None:
        # Arduino parser accepts only [0,0,0] style, not [0.0,0.0,0.0].
        line = f"[{self.current_cmd[0]},{self.current_cmd[1]},{self.current_cmd[2]}]\n"

        try:
            self.serial_port.write(line.encode("ascii"))
        except SerialException as exc:
            self.get_logger().warn(f"Serial write failed: {exc}")
            self.close_serial()

    def read_serial(self) -> None:
        """Read serial bytes and extract complete '<...>' Arduino frames.

        This mirrors arduino_sensor_driver's frame-boundary scanner. It avoids
        treating a startup half-frame as a full line when ROS connects while the
        Arduino is already streaming STATE frames.
        """
        if self.serial_port is None or not self.serial_port.is_open:
            return

        try:
            waiting = self.serial_port.in_waiting
            if waiting <= 0:
                return

            data = self.serial_port.read(waiting)
            if not data:
                return

            self.rx_buffer.extend(data)

            while True:
                start = self.rx_buffer.find(b"<")
                if start == -1:
                    if len(self.rx_buffer) > 0 and bytes(self.rx_buffer).strip():
                        self.get_logger().warn(
                            f"Discarding {len(self.rx_buffer)} non-frame bytes",
                            throttle_duration_sec=2.0,
                        )
                    self.rx_buffer.clear()
                    break

                if start > 0:
                    del self.rx_buffer[:start]

                end = self.rx_buffer.find(b">")
                if end == -1:
                    break

                frame_bytes = bytes(self.rx_buffer[: end + 1])
                del self.rx_buffer[: end + 1]

                try:
                    frame = frame_bytes.decode("ascii", errors="ignore").strip()
                except UnicodeDecodeError as exc:
                    self.get_logger().warn(
                        f"Frame decode error: {exc}",
                        throttle_duration_sec=2.0,
                    )
                    continue

                if frame:
                    self.handle_serial_line(frame)

            # If noise/corruption arrives without a frame tail, avoid unbounded growth.
            if len(self.rx_buffer) > 512:
                self.get_logger().error(
                    f"Frame buffer overflow ({len(self.rx_buffer)} bytes), "
                    'no closing ">" received; clearing buffer.',
                    throttle_duration_sec=2.0,
                )
                self.rx_buffer.clear()

        except SerialException as exc:
            self.get_logger().error(f"Serial disconnected: {exc}. Will reconnect.")
            self.close_serial()
        except OSError as exc:
            self.get_logger().error(
                f"OS error on serial (device removed?): {exc}. Closing and reconnecting."
            )
            self.close_serial()
        except Exception as exc:
            self.get_logger().error(
                f"Unexpected serial read error: {exc}. Closing and reconnecting."
            )
            self.close_serial()

    def handle_serial_line(self, line: str) -> None:
        raw_msg = String()
        raw_msg.data = line
        self.raw_frame_pub.publish(raw_msg)

        payload = self.extract_and_verify_payload(line)
        if payload is None:
            return

        state_match = STATE_RE.match(payload)
        if state_match:
            pneu_bits = [
                int(state_match.group(2)),
                int(state_match.group(3)),
                int(state_match.group(4)),
            ]
            ir_bit = int(state_match.group(5))

            pneu_msg = Int8MultiArray()
            pneu_msg.data = pneu_bits
            self.pneu_ack_pub.publish(pneu_msg)

            ir_msg = Bool()
            ir_msg.data = bool(ir_bit)
            self.ir_status_pub.publish(ir_msg)
            return

        ack_match = ACK_RE.match(payload)
        if ack_match:
            # The original INO has ACK_EACH_VALID_COMMAND = false.
            # Parser remains here in case ACK is enabled later.
            return

        err_match = ERR_RE.match(payload)
        if err_match:
            reason = err_match.group(1)
            self.get_logger().warn(
                f"Arduino error frame: {reason}",
                throttle_duration_sec=2.0,
            )
            return

        if BOOT_RE.match(payload):
            self.get_logger().info("Arduino boot frame received.")
            return

        self.get_logger().warn(f"Unknown Arduino frame payload: {payload}")

    def extract_and_verify_payload(self, frame: str) -> Optional[str]:
        """Return payload if the Arduino frame wrapper and XOR checksum are valid."""
        if not frame.startswith("<") or not frame.endswith(">"):
            self.get_logger().warn(
                f"Invalid frame wrapper: {frame}",
                throttle_duration_sec=2.0,
            )
            return None

        body = frame[1:-1]

        if ",*" not in body:
            self.get_logger().warn(
                f"Frame has no checksum separator: {frame}",
                throttle_duration_sec=2.0,
            )
            return None

        payload, checksum_text = body.rsplit(",*", 1)

        try:
            received_lrc = int(checksum_text, 16)
        except ValueError:
            self.get_logger().warn(
                f"Invalid checksum text: {checksum_text}",
                throttle_duration_sec=2.0,
            )
            return None

        calculated_lrc = self.calc_xor_lrc(payload)

        if calculated_lrc != received_lrc:
            self.get_logger().warn(
                "Checksum mismatch. "
                f"payload={payload}, "
                f"received=0x{received_lrc:02X}, "
                f"calculated=0x{calculated_lrc:02X}",
                throttle_duration_sec=2.0,
            )
            return None

        return payload

    @staticmethod
    def calc_xor_lrc(payload: str) -> int:
        lrc = 0
        for ch in payload:
            lrc ^= ord(ch)
        return lrc & 0xFF

    def close_serial(self) -> None:
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except SerialException:
                pass
        self.serial_port = None

    def destroy_node(self) -> None:
        self.close_serial()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmArduinoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
