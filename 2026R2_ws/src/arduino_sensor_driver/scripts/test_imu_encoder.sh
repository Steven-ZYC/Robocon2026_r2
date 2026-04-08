#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

find_arduino_port() {
    local by_id_dir="/dev/serial/by-id"
    if [ -d "$by_id_dir" ]; then
        for entry in "$by_id_dir"/*; do
            [ -e "$entry" ] || continue
            if echo "$entry" | grep -qi "arduino"; then
                realpath "$entry"
                return 0
            fi
        done
    fi

    for dev in /dev/ttyACM* /dev/ttyUSB*; do
        [ -e "$dev" ] || continue
        echo "$dev"
        return 0
    done

    return 1
}

SERIAL_PORT="${1:-}"
if [ -z "$SERIAL_PORT" ]; then
    SERIAL_PORT="$(find_arduino_port)" || {
        echo "No Arduino serial device found." >&2
        exit 1
    }
fi

python3 - "$SERIAL_PORT" <<'PYEOF'
import re
import sys
import time
import serial

port = sys.argv[1]


def crc8_atm(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc = crc << 1
            crc &= 0xFF
    return crc


pattern = re.compile(
    r'ID=(\d+) T=(\d+) IMU=([\d.\-]+),([\d.\-]+),([\d.\-]+),([\d.\-]+),([\d.\-]+) ENC=([\-\d]+),([\-\d]+)'
)

print(f"Reading raw Arduino packets from {port}. Press Ctrl+C to stop.")

ser = serial.Serial(port, 115200, timeout=1.0)
time.sleep(0.3)
ser.reset_input_buffer()

try:
    while True:
        raw = ser.readline()
        if not raw.endswith(b'\n'):
            continue

        line = raw.decode('ascii', errors='ignore').strip()
        if not line:
            continue

        match_crc = re.search(r' crc=([0-9A-Fa-f]{2})$', line)
        if not match_crc:
            print(f"RAW  {line}")
            print("CRC  missing")
            print("-")
            continue

        crc_expected = int(match_crc.group(1), 16)
        payload = line[:match_crc.start()]
        crc_actual = crc8_atm(payload.encode('ascii'))
        crc_ok = crc_actual == crc_expected

        match = pattern.match(payload)
        if not match:
            print(f"RAW  {line}")
            print(f"CRC  {'OK' if crc_ok else 'FAIL'}")
            print("PARSE failed")
            print("-")
            continue

        packet_id = int(match.group(1))
        timestamp_ms = int(match.group(2))
        heading = float(match.group(3))
        rate = float(match.group(4))
        ax = float(match.group(5))
        ay = float(match.group(6))
        az = float(match.group(7))
        enc_x = int(match.group(8))
        enc_y = int(match.group(9))

        print(
            f"ID={packet_id} T={timestamp_ms}ms CRC={'OK' if crc_ok else 'FAIL'} "
            f"HDG={heading:.3f} RATE={rate:.3f} AX={ax:.3f} AY={ay:.3f} AZ={az:.3f} "
            f"ENC_X={enc_x} ENC_Y={enc_y}"
        )
except KeyboardInterrupt:
    pass
finally:
    ser.close()
PYEOF
