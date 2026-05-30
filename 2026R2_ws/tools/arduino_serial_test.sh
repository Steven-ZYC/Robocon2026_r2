#!/usr/bin/env bash
# ============================================================================
# arduino_serial_test.sh — Arduino 串口稳定性诊断脚本
# ============================================================================
# 用途：在不启动 ROS2 的情况下，直接测试 Arduino 串口通信质量
# 检测：数据损坏率、断连频率、USB 电源状态、CRC 校验通过率
# 用法：bash arduino_serial_test.sh [duration_sec] [device]
# ============================================================================

DURATION=${1:-30}
DEVICE=${2:-/dev/sensor_arduino}
BAUD=115200

echo "=============================================="
echo " Arduino 串口稳定性测试"
echo "=============================================="
echo " 设备: $DEVICE"
echo " 波特率: $BAUD"
echo " 测试时长: ${DURATION}s"
echo " 时间: $(date)"
echo "=============================================="

# ---- 1. USB 电源管理状态 ----
echo ""
echo "[1] USB 电源管理状态"
echo "----------------------------------------"
# 找到 Arduino 对应的 USB 设备
ARDUINO_USB_DEV=$(basename "$(readlink -f /sys/class/tty/$(basename "$(readlink -f "$DEVICE")")/device/..)" 2>/dev/null)
if [ -z "$ARDUINO_USB_DEV" ]; then
    # fallback: 直接找包含 Arduino 的 USB 设备
    for dev in /sys/bus/usb/devices/*/manufacturer; do
        if grep -q "Arduino" "$dev" 2>/dev/null; then
            ARDUINO_USB_DEV=$(basename "$(dirname "$dev")")
            break
        fi
    done
fi

if [ -n "$ARDUINO_USB_DEV" ]; then
    echo "  USB 设备: $ARDUINO_USB_DEV"
    POWER_DIR="/sys/bus/usb/devices/$ARDUINO_USB_DEV/power"
    if [ -d "$POWER_DIR" ]; then
        CTRL=$(cat "$POWER_DIR/control" 2>/dev/null || echo "N/A")
        STATUS=$(cat "$POWER_DIR/runtime_status" 2>/dev/null || echo "N/A")
        echo "  power/control:       $CTRL"
        echo "  power/runtime_status: $STATUS"
        # 检查是否可写（修改 autosuspend 需要 root）
        if [ -w "$POWER_DIR/control" ]; then
            echo "  [OK] 有权限修改 power/control"
        else
            echo "  [注意] 无权限修改 power/control (需要 root)"
        fi
    else
        echo "  [注意] 找不到 power 目录"
    fi
else
    echo "  [注意] 找不到 Arduino USB 设备"
fi

# 显示所有 USB 总线的 autosuspend 状态
echo ""
echo "  所有 USB bus autosuspend 状态:"
for ctrl in /sys/bus/usb/devices/usb*/power/control; do
    bus=$(basename "$(dirname "$(dirname "$ctrl")")")
    val=$(cat "$ctrl" 2>/dev/null || echo "?")
    echo "    $bus: $val"
done

# ---- 2. 设备节点检查 ----
echo ""
echo "[2] 设备节点"
echo "----------------------------------------"
if [ -e "$DEVICE" ]; then
    TARGET=$(readlink -f "$DEVICE")
    PERMS=$(stat -c "%a %U:%G" "$DEVICE" 2>/dev/null)
    echo "  $DEVICE -> $TARGET ($PERMS)"
else
    echo "  [错误] $DEVICE 不存在!"
    exit 1
fi

# 检查是否有其他进程占用
USERS=$(lsof "$TARGET" 2>/dev/null | tail -n +2)
if [ -n "$USERS" ]; then
    echo "  [警告] 以下进程正在使用该串口:"
    echo "$USERS"
else
    echo "  [OK] 无其他进程占用"
fi

# ---- 3. CRC8-ATM 校验函数 ----
crc8_atm() {
    python3 -c "
import sys
data = sys.argv[1].encode('ascii')
crc = 0
for byte in data:
    crc ^= byte
    for _ in range(8):
        crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
        crc &= 0xFF
print(f'{crc:02X}')
" "$1"
}

# ---- 4. 串口数据采集与校验 ----
echo ""
echo "[3] 串口数据采集 (${DURATION}s)"
echo "----------------------------------------"

TMPFILE=$(mktemp /tmp/arduino_test_XXXXXX)

# 配置串口参数 (cat 不会自动设置波特率，必须先 stty)
stty -F "$DEVICE" "$BAUD" raw -echo -echoe -echok 2>/dev/null
if [ $? -ne 0 ]; then
    echo "  [错误] 无法配置串口 $DEVICE (stty 失败)"
    rm -f "$TMPFILE"
    exit 1
fi
echo "  [OK] 串口已配置: $BAUD baud, raw mode"

# 用 timeout 限制采集时间，直接 cat 原始数据
timeout "$DURATION" cat "$DEVICE" > "$TMPFILE" 2>/dev/null
FILESIZE=$(stat -c%s "$TMPFILE" 2>/dev/null || echo 0)
echo "  采集字节数: $FILESIZE"

if [ "$FILESIZE" -lt 100 ]; then
    echo "  [错误] 数据量过少，请检查串口连接"
    rm -f "$TMPFILE"
    exit 1
fi

# 用 Python 进行详细分析
echo ""
echo "[4] 数据包分析"
echo "----------------------------------------"
python3 << PYEOF
import re
import sys

# CRC8-ATM
def crc8_atm(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc

# 读取原始数据
with open("$TMPFILE", "rb") as f:
    raw = f.read()

# 按行分割（处理 \\r\\n 和 \\n）
lines = raw.replace(b'\r\n', b'\n').split(b'\n')

total_lines = 0
has_crc = 0
crc_ok = 0
crc_fail = 0
parse_ok = 0
parse_fail = 0
merged_lines = 0  # 两包黏连

# 协议格式: ID=<pkg_id> T=<ms> IMU=<hdg>,<rate>,<ax>,<ay>,<az> ENC=<x_cnt>,<y_cnt> crc=<hex>
pattern = re.compile(
    rb"ID=(\d+) T=(\d+) "
    rb"IMU=([\d\.\-]+),([\d\.\-]+),([\d\.\-]+),([\d\.\-]+),([\d\.\-]+) "
    rb"ENC=([\-\d]+),([\-\d]+)"
)

prev_id = None
id_jumps = 0

for line in lines:
    if not line:
        continue
    total_lines += 1

    # 检查是否包含 crc=
    crc_match = re.search(rb" crc=([0-9A-Fa-f]{2})", line)
    if not crc_match:
        # 可能是黏连包 — 找最后一个 crc=
        crc_matches = list(re.finditer(rb" crc=([0-9A-Fa-f]{2})", line))
        if len(crc_matches) > 1:
            merged_lines += 1
        elif len(crc_matches) == 0:
            pass  # 纯垃圾行
        continue

    has_crc += 1
    crc_hex = crc_match.group(1).decode()
    expected = int(crc_hex, 16)
    data_to_check = line[:crc_match.start()]
    actual = crc8_atm(data_to_check)

    if actual == expected:
        crc_ok += 1
        # 尝试解析
        m = pattern.search(line)
        if m:
            parse_ok += 1
            pkg_id = int(m.group(1))
            if prev_id is not None and pkg_id != prev_id + 1:
                id_jumps += 1
            prev_id = pkg_id
        else:
            parse_fail += 1
    else:
        crc_fail += 1
        # 检查是否为黏连包
        if len(re.findall(rb"ID=\d+", line)) > 1:
            merged_lines += 1

print(f"  总行数:       {total_lines}")
print(f"  含 CRC 行:    {has_crc}")
print(f"  CRC 通过:     {crc_ok}  ({crc_ok/has_crc*100:.1f}%)" if has_crc else "  CRC 通过:     0")
print(f"  CRC 失败:     {crc_fail}")
print(f"  解析成功:     {parse_ok}")
print(f"  解析失败:     {parse_fail}")
print(f"  黏连包:       {merged_lines}")
print(f"  ID 跳跃次数:  {id_jumps}")
print(f"  断连次数:     (需配合 dmesg 查看)")

# 估算实际数据率
if total_lines > 0:
    duration = $DURATION
    rate = has_crc / duration
    valid_rate = crc_ok / duration
    print(f"\n  含CRC行速率:  ~{rate:.0f} packets/s")
    print(f"  CRC通过速率:  ~{valid_rate:.0f} packets/s")
    if valid_rate < 80:
        print(f"  [警告] 有效速率偏低 (期望 ~100 packets/s)")
    elif valid_rate > 95:
        print(f"  [OK] 速率正常")

# 断连检测
print(f"\n[4.1] 断连检测")
print("  (断连表现为 cat 输出中断，可通过以下方式确认:)")
print(f"  1. 查看 dmesg:  sudo dmesg | grep -E 'USB|ttyACM|disconnect'")
print(f"  2. 比较采集字节数 ({$FILESIZE}) 与预期 ({100*$DURATION}~{120*$DURATION})")
expected_min = 100 * $DURATION
if $FILESIZE < expected_min:
    print(f"  [警告] 采集字节数少于预期，可能存在断连")

PYEOF

# ---- 5. 时间间隙分析 (检测是否有数据中断) ----
echo ""
echo "[5] 时间间隙分析"
echo "----------------------------------------"
python3 << PYEOF
import re

with open("$TMPFILE", "rb") as f:
    raw = f.read()

# 提取所有 T= 时间戳
timestamps = []
for m in re.finditer(rb"T=(\d+)", raw):
    timestamps.append(int(m.group(1)))

if len(timestamps) < 2:
    print("  时间戳不足，无法分析")
else:
    # Arduino T 单位是 ms，计算相邻差值
    gaps = []
    for i in range(1, len(timestamps)):
        gap = timestamps[i] - timestamps[i-1]
        if gap < 0:
            gap += 65536  # millis() 溢出回绕 (约65s)
        if gap < 200:  # 过滤明显异常值
            gaps.append(gap)

    if gaps:
        avg_gap = sum(gaps) / len(gaps)
        max_gap = max(gaps)
        min_gap = min(gaps)
        # 统计 > 20ms 的长间隙 (可能是数据中断)
        long_gaps = [g for g in gaps if g > 20]
        very_long_gaps = [g for g in gaps if g > 100]

        print(f"  时间戳样本数:  {len(timestamps)}")
        print(f"  平均间隔:      {avg_gap:.1f} ms (期望 ~10ms)")
        print(f"  最小间隔:      {min_gap} ms")
        print(f"  最大间隔:      {max_gap} ms")
        print(f"  长间隙(>20ms): {len(long_gaps)} 次")
        print(f"  超长间隙(>100ms): {len(very_long_gaps)} 次" +
              (" [断连迹象!]" if very_long_gaps else ""))

        if very_long_gaps:
            print(f"\n  前10个超长间隙:")
            for g in very_long_gaps[:10]:
                print(f"    {g} ms")

PYEOF

# ---- 6. 原始数据样本 ----
echo ""
echo "[6] 原始数据样本 (前 10 行非空)"
echo "----------------------------------------"
python3 << PYEOF
with open("$TMPFILE", "rb") as f:
    raw = f.read()

lines = raw.replace(b'\r\n', b'\n').split(b'\n')
shown = 0
for line in lines:
    stripped = line.strip()
    if stripped:
        try:
            decoded = stripped.decode('ascii', errors='replace')
            status = "OK" if "crc=" in decoded else "??"
            print(f"  [{status}] {decoded[:100]}")
        except:
            print(f"  [??] {stripped[:80]}")
        shown += 1
        if shown >= 10:
            break
PYEOF

# ---- 7. 建议 ----
echo ""
echo "=============================================="
echo " 诊断建议"
echo "=============================================="
echo ""
echo "  [如果 CRC 通过率 < 95%]:"
echo "    → 换 USB 线 (短、带磁环、质量好)"
echo "    → 检查 Arduino 电源 (USB供电可能不足)"
echo ""
echo "  [如果存在超长间隙 (>100ms)]:"
echo "    → 运行: sudo dmesg -w"
echo "    → 重新执行本脚本，观察是否有 USB disconnect 事件"
echo "    → 如果有 disconnect: 尝试禁用 autosuspend:"
echo "        echo 'on' | sudo tee /sys/bus/usb/devices/$ARDUINO_USB_DEV/power/control"
echo ""
echo "  [如果黏连包很多]:"
echo "    → Arduino 串口缓冲区可能溢出"
echo "    → 检查 Arduino 端是否有阻塞操作 (delay, 大量 Serial.print)"
echo "    → 考虑降低数据输出频率或增加串口波特率"
echo ""
echo "  原始数据保存在: $TMPFILE"
echo "=============================================="
