#!/bin/bash
# Arduino Sensor Driver Test Script (ROS2 Native)
# 功能：
#   1. 使用当前 venv 编译 arduino_sensor_driver 包
#   2. 启动 arduino_sensor_parser 节点（自动设备发现）
#   3. 实时解析并打印 /arduino/raw_sensor_data topic 内容
#   4. 持续监控直到 Ctrl+C
#
# 适用：arduino_sensor_driver package 的硬件在环测试
# 放置路径：2026R2_ws/src/arduino_sensor_driver/scripts/

# ==================== 路径 ====================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# ==================== 颜色 ====================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}============================================${NC}"
echo -e "${CYAN}   IMU + Encoder 数据读取测试脚本${NC}"
echo -e "${CYAN}============================================${NC}"
echo ""
echo "测试内容："
echo "  [1] Arduino 串口连接检查"
echo "  [2] 直接串口读取（不启动 ROS2，验证原始数据格式）"
echo "  [3] CRC8-ATM 校验验证"
echo "  [4] ROS2 节点启动与数据订阅"
echo "  [5] 实时 IMU / 编码器数据监控（持续，Ctrl+C 退出）"
echo ""

# ==================== Step 1: 环境检查 ====================
echo -e "${YELLOW}[Step 1/5] 检查运行环境...${NC}"

# 检查 ROS2 workspace 是否已 build
if [ ! -f "$WS_DIR/install/setup.bash" ]; then
    echo -e "${RED}  ✗ Workspace 尚未 build！${NC}"
    echo "  请先执行："
    echo "    cd $WS_DIR && colcon build --packages-select arduino_sensor_msgs arduino_sensor_driver"
    exit 1
fi

# 检查 pyserial
if ! python3 -c "import serial" 2>/dev/null; then
    echo -e "${RED}  ✗ pyserial 未安装！${NC}"
    echo "  请执行：pip3 install pyserial"
    exit 1
fi

echo -e "${GREEN}  ✓ 环境检查通过${NC}"

# ==================== Step 2: 串口设备检测 ====================
echo ""
echo -e "${YELLOW}[Step 2/5] 检测 Arduino 串口设备...${NC}"

# 优先使用 /dev/serial/by-id/ 进行设备发现（与 Python node 一致）
BY_ID_DIR="/dev/serial/by-id/"
SERIAL_PORT=""

if [ -d "$BY_ID_DIR" ]; then
    # 查找包含 "Arduino" 的设备
    for entry in "$BY_ID_DIR"*; do
        if [ -e "$entry" ] && echo "$entry" | grep -qi "arduino"; then
            SERIAL_PORT=$(realpath "$entry")
            echo "  通过 by-id 发现 Arduino: $entry -> $SERIAL_PORT"
            break
        fi
    done
fi

# 回退到传统设备路径
if [ -z "$SERIAL_PORT" ]; then
    SERIAL_DEVICES=$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true)
    if [ -z "$SERIAL_DEVICES" ]; then
        echo -e "${RED}  ✗ 未找到 Arduino 串口设备！${NC}"
        echo ""
        echo "  排查步骤："
        echo "    1. 确认 Arduino 已通过 USB 连接"
        echo "    2. 确认 Arduino 已上电运行"
        echo "    3. 执行 ls /dev/tty* 查看所有串口设备"
        echo "    4. 若设备存在但无权限：sudo chmod 666 /dev/ttyACM0"
        echo "       或永久授权：sudo usermod -aG dialout \$USER（需重新登录）"
        exit 1
    fi

    echo "  检测到以下串口设备："
    for dev in $SERIAL_DEVICES; do
        echo "    - $dev"
    done

    # 自动选择默认设备
    DEFAULT_PORT=$(echo "$SERIAL_DEVICES" | head -n1)
    echo ""
    read -p "  请选择串口设备 [$DEFAULT_PORT]: " SERIAL_PORT
    SERIAL_PORT=${SERIAL_PORT:-$DEFAULT_PORT}
fi

# 验证设备存在
if [ ! -e "$SERIAL_PORT" ]; then
    echo -e "${RED}  ✗ 设备 $SERIAL_PORT 不存在！${NC}"
    exit 1
fi

# 检查并修复权限
if [ ! -r "$SERIAL_PORT" ] || [ ! -w "$SERIAL_PORT" ]; then
    echo -e "${YELLOW}  ⚠ 串口权限不足，尝试修复...${NC}"
    sudo chmod 666 "$SERIAL_PORT" 2>/dev/null || {
        echo -e "${RED}  ✗ 权限修复失败${NC}"
        echo "  请执行：sudo usermod -aG dialout \$USER && newgrp dialout"
        exit 1
    }
    echo -e "${GREEN}  ✓ 权限已修复${NC}"
fi

echo -e "${GREEN}  ✓ 使用串口：$SERIAL_PORT${NC}"
echo "  （注：Python node 支持自动发现，启动时可不指定 serial_port 参数）"

# ==================== Step 3: 直接串口读取 + 格式 / CRC 验证 ====================
echo ""
echo -e "${YELLOW}[Step 3/5] 直接串口读取（5 行原始数据 + CRC 验证）...${NC}"
echo ""

RAW_CRC_SCRIPT=$(mktemp /tmp/test_imu_enc_raw_XXXXX.py)
cat > "$RAW_CRC_SCRIPT" << 'PYEOF'
#!/usr/bin/env python3
"""
直接串口读取 Arduino 数据，验证格式与 CRC8-ATM 校验。
不依赖 ROS2，可单独用于硬件排查。
"""
import serial
import sys
import time
import re

port = sys.argv[1]
TARGET_LINES = 8
TIMEOUT_SEC = 10.0

def crc8_atm(data: bytes) -> int:
    """CRC8-ATM 校验（多项式 0x07，初值 0x00）"""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc << 1) ^ 0x07 if (crc & 0x80) else crc << 1
            crc &= 0xFF
    return crc

def parse_and_display(line: str, idx: int):
    """解析一行数据，打印关键字段，返回 crc_ok"""
    # 提取 crc
    m_crc = re.search(r' crc=([0-9A-Fa-f]{2})$', line)
    if not m_crc:
        print(f"  [{idx}] 格式错误（无 crc 字段）: {line[:80]}")
        return False

    crc_expect = int(m_crc.group(1), 16)
    body = line[:m_crc.start()]
    crc_actual = crc8_atm(body.encode('ascii'))
    crc_ok = (crc_actual == crc_expect)

    # 解析字段（v2 协议：无 DEG= 字段）
    m = re.match(
        r'ID=(\d+) T=(\d+) IMU=([\d.\-]+),([\d.\-]+),([\d.\-]+),([\d.\-]+),([\d.\-]+) '
        r'ENC=([\-\d]+),([\-\d]+)',
        body
    )
    if not m:
        print(f"  [{idx}] 字段解析失败: {body[:80]}")
        return False

    pkg_id   = int(m.group(1))
    ts_ms    = int(m.group(2))
    hdg      = float(m.group(3))
    rate     = float(m.group(4))
    ax, ay, az = float(m.group(5)), float(m.group(6)), float(m.group(7))
    enc_x    = int(m.group(8))
    enc_y    = int(m.group(9))

    # 计算角度（与 Arduino 端一致：degrees = counts * 360 / CPR, CPR=8192）
    CPR = 8192
    deg_x = (enc_x * 360.0 / CPR) % 360.0
    deg_y = (enc_y * 360.0 / CPR) % 360.0

    crc_str  = f"\033[32m✓ CRC OK\033[0m" if crc_ok else f"\033[31m✗ CRC FAIL (expect {crc_expect:02X}, got {crc_actual:02X})\033[0m"

    print(f"  [{idx}] ID={pkg_id:<5}  T={ts_ms}ms")
    print(f"        IMU  ▸ heading={hdg:>7.2f}°  rate={rate:>6.3f} rad/s  acc=({ax:.3f},{ay:.3f},{az:.3f})g")
    print(f"        ENC  ▸ X={enc_x:>8} cnts ({deg_x:>7.2f}°)  Y={enc_y:>8} cnts ({deg_y:>7.2f}°)")
    print(f"        CRC  ▸ {crc_str}")
    print()
    return crc_ok

try:
    ser = serial.Serial(port, 115200, timeout=2.0)
    # 等待 Arduino 穩定：先讀取並丟棄前 10 行（避免串口重置後的殘缺包）
    time.sleep(0.3)
    for _ in range(10):
        ser.readline()
    time.sleep(0.2)

    lines_ok  = 0
    crc_pass  = 0
    crc_fail  = 0
    start     = time.time()

    while lines_ok < TARGET_LINES:
        if time.time() - start > TIMEOUT_SEC:
            print(f"\n  ✗ 超时（{TIMEOUT_SEC}s 内未收到足够数据）")
            break

        raw = ser.readline().decode('ascii', errors='ignore').strip()
        if not raw:
            continue

        # 只處理以 ID= 開頭的完整包（跳過殘缺/亂碼行）
        if not raw.startswith('ID='):
            continue

        ok = parse_and_display(raw, lines_ok + 1)
        lines_ok += 1
        if ok:
            crc_pass += 1
        else:
            crc_fail += 1

    ser.close()

    # 汇总
    print("  " + "─" * 48)
    print(f"  完整包：{lines_ok} 行    CRC 通过：{crc_pass}    CRC 失败：{crc_fail}")
    rate_pct = 100.0 * crc_pass / max(lines_ok, 1)
    if rate_pct >= 95.0:
        print(f"  CRC 成功率：\033[32m{rate_pct:.1f}%  通过\033[0m")
        sys.exit(0)
    else:
        print(f"  CRC 成功率：\033[31m{rate_pct:.1f}%  过低，请检查串口连接与波特率\033[0m")
        sys.exit(1)

except serial.SerialException as e:
    print(f"\n  ✗ 串口错误：{e}")
    sys.exit(1)
except KeyboardInterrupt:
    print("\n  （已中止）")
    sys.exit(0)
PYEOF

python3 "$RAW_CRC_SCRIPT" "$SERIAL_PORT"
RAW_RESULT=$?
rm -f "$RAW_CRC_SCRIPT"

if [ $RAW_RESULT -ne 0 ]; then
    echo -e "${RED}  ✗ 原始数据读取测试失败，中止后续步骤${NC}"
    echo ""
    echo "  常见原因："
    echo "    - 波特率不匹配（Arduino 需要 Serial.begin(115200)）"
    echo "    - 串口线缆接触不良"
    echo "    - Arduino 程序未正确烧录"
    exit 1
fi

echo -e "${GREEN}  ✓ 原始数据格式与 CRC 验证通过${NC}"

# ==================== Step 4: Build + 启动 ROS2 节点 ====================
echo ""
echo -e "${YELLOW}[Step 4/5] 构建 workspace 并启动 ROS2 节点...${NC}"

cd "$WS_DIR"
colcon build --packages-select arduino_sensor_msgs arduino_sensor_driver \
    2>&1 | grep -E "(Starting|Finished|Failed|Summary)" || echo "  构建中..."

if [ ${PIPESTATUS[0]} -ne 0 ]; then
    echo -e "${RED}  ✗ 构建失败！${NC}"
    exit 1
fi
echo -e "${GREEN}  ✓ 构建成功${NC}"

# source：先 ROS2 base，再 workspace（必须在同一 shell，不能子进程）
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source "$WS_DIR/install/setup.bash"

# 启动节点（后台）
# 注意：ros2 run 會重置 PYTHONPATH，導致找不到 arduino_sensor_msgs
# 改用 python3 直接執行 entry point 腳本
# 如果 Step 2 成功检测到设备，Python node 的自动发现功能也可以直接使用
PYTHONPATH="$WS_DIR/install/arduino_sensor_msgs/local/lib/python3.10/dist-packages:$WS_DIR/install/arduino_sensor_driver/lib/python3.10/site-packages:$PYTHONPATH" \
python3 "$WS_DIR/install/arduino_sensor_driver/lib/arduino_sensor_driver/arduino_sensor_parser" \
    --ros-args \
    -p serial_port:="$SERIAL_PORT" \
    -p device_id_pattern:="Arduino" \
    -p baud_rate:=115200 \
    -p timeout_sec:=1.0 \
    -p publish_tf:=false \
    > /tmp/arduino_node.log 2>&1 &
NODE_PID=$!
echo -e "${GREEN}  ✓ arduino_sensor_parser 已启动（PID: $NODE_PID）${NC}"
echo "  等待节点初始化..."
sleep 4

# 确认节点在线
if ! kill -0 $NODE_PID 2>/dev/null; then
    echo -e "${RED}  ✗ 节点已退出！错误日志：${NC}"
    cat /tmp/arduino_node.log
    exit 1
fi
if ! ros2 node list 2>/dev/null | grep -q "arduino_sensor_parser"; then
    echo -e "${RED}  ✗ 节点未出现在 ros2 node list，查看日志：cat /tmp/arduino_node.log${NC}"
    kill $NODE_PID 2>/dev/null || true
    exit 1
fi
echo -e "${GREEN}  ✓ 节点在线${NC}"

# ==================== Step 5: 实时数据监控 ====================
echo ""
echo -e "${YELLOW}[Step 5/5] 实时 IMU + 编码器数据监控（Ctrl+C 退出）...${NC}"
echo ""
echo "  订阅 topic：/arduino/raw_sensor_data"
echo "  ─────────────────────────────────────────────────────"
echo ""

MONITOR_SCRIPT=$(mktemp /tmp/test_imu_enc_monitor_XXXXX.py)
cat > "$MONITOR_SCRIPT" << 'PYEOF'
#!/usr/bin/env python3
"""
实时订阅 /arduino/raw_sensor_data，
在终端打印 IMU 与编码器关键字段，统计 CRC 状态。
"""
import rclpy
from rclpy.node import Node
from arduino_sensor_msgs.msg import ArduinoSensorData
import sys

class ImuEncoderMonitor(Node):
    """订阅原始传感器数据并在终端实时打印"""

    def __init__(self):
        super().__init__('imu_encoder_monitor')

        self.total      = 0
        self.crc_pass   = 0
        self.crc_fail   = 0
        self.last_pkg   = -1

        self.sub = self.create_subscription(
            ArduinoSensorData,
            '/arduino/raw_sensor_data',
            self.callback,
            10
        )
        self.get_logger().info("IMU + Encoder 实时监控已启动 — Ctrl+C 退出")

    def callback(self, msg: ArduinoSensorData):
        self.total += 1
        if msg.crc_valid:
            self.crc_pass += 1
        else:
            self.crc_fail += 1

        crc_tag = "\033[32m[CRC✓]\033[0m" if msg.crc_valid else "\033[31m[CRC✗]\033[0m"

        # 每 10 帧打印一次详情，避免刷屏
        if self.total % 10 == 1:
            print(
                f"  {crc_tag} "
                f"ID={msg.packet_id:<6} "
                f"T={msg.timestamp_ms}ms | "
                f"HDG={msg.imu_heading_deg:>7.2f}°  "
                f"RATE={msg.imu_rate_rad_s:>6.3f}rad/s | "
                f"ENC_X={msg.enc_x_counts:>8}  ENC_Y={msg.enc_y_counts:>8} | "
                f"PASS={self.crc_pass}/{self.total}"
            )

def main():
    rclpy.init()
    node = ImuEncoderMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 打印最终统计
        print()
        print("  ─────────────────────────────────────────────────")
        print(f"  监控结束  共收到：{node.total} 条  "
              f"CRC 通过：{node.crc_pass}  "
              f"CRC 失败：{node.crc_fail}  "
              f"成功率：{100.0*node.crc_pass/max(node.total,1):.1f}%")
        print("  ─────────────────────────────────────────────────")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
PYEOF

# 清理函数：退出时停止节点
cleanup() {
    echo ""
    echo -e "${YELLOW}  正在停止所有进程...${NC}"
    kill $NODE_PID 2>/dev/null || true
    rm -f "$MONITOR_SCRIPT"
    pkill -P $$ 2>/dev/null || true
    echo -e "${GREEN}  ✓ 清理完毕${NC}"
    echo ""
    echo "后续可用命令："
    echo "  ros2 topic echo /arduino/raw_sensor_data"
    echo "  ros2 topic echo /state_odom"
    echo "  ros2 topic hz  /arduino/raw_sensor_data   # 查看实际频率"
    exit 0
}
trap cleanup SIGINT SIGTERM

python3 "$MONITOR_SCRIPT"
