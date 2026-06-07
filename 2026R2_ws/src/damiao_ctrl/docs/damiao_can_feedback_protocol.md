# 达妙 CAN 反馈帧解算逻辑

达妙电机 CAN 反馈帧为 **8 byte**：

```
D0 D1 D2 D3 D4 D5 D6 D7
```

## 字段拆解

| 字节 | 位 | 字段 | 说明 |
|---|---|---|---|
| D0[3:0] | 0-3 | ID | 电机 ID |
| D0[7:4] | 4-7 | ERR | 错误/状态码 |
| D1 D2 | - | POS_raw | 位置，16 bit |
| D3 D4[7:4] | - | VEL_raw | 速度，12 bit |
| D4[3:0] D5 | - | T_raw | 力矩，12 bit |
| D6 | - | T_MOS | MOS 温度 |
| D7 | - | T_Rotor | 转子温度 |

---

## 1. ID 和错误码

```
id  = data[0] & 0x0F
err = (data[0] >> 4) & 0x0F
```

例：`D0 = 0x13 = 0001 0011` → `ERR = 1, ID = 3`

---

## 2. 位置 POS（16 bit）

```
pos_raw = (data[1] << 8) | data[2]
```

范围 `0 ~ 65535`，线性映射到真实位置：

```
pos = uint_to_float(pos_raw, P_MIN, P_MAX, 16)
```

---

## 3. 速度 VEL（12 bit）

```
D3        = VEL[11:4]
D4[7:4]   = VEL[3:0]

vel_raw = (data[3] << 4) | (data[4] >> 4)
```

范围 `0 ~ 4095`，线性映射：

```
vel = uint_to_float(vel_raw, V_MIN, V_MAX, 12)
```

---

## 4. 力矩 Torque（12 bit）

```
D4[3:0]   = T[11:8]
D5        = T[7:0]

torque_raw = ((data[4] & 0x0F) << 8) | data[5]
```

范围 `0 ~ 4095`，线性映射：

```
torque = uint_to_float(torque_raw, T_MIN, T_MAX, 12)
```

### 力矩换算说明

若 datasheet 最大输出 torque 为 `8 Nm`，按达妙反馈范围理解为 `-8 ~ +8 Nm`：

```
T_MIN = -8.0
T_MAX =  8.0

torque_nm = torque_raw / 4095.0 * 16.0 - 8.0
          = (torque_raw - 2047.5) * 16.0 / 4095.0
```

注意：此 torque 为达妙定义的输出力矩，**通常不要再乘 gear_ratio**。

---

## 5. 温度

```
t_mos   = data[6]   // MOS 温度, °C
t_rotor = data[7]   // 转子温度, °C
```

---

## 通用线性映射函数

```python
def uint_to_float(x: int, x_min: float, x_max: float, bits: int) -> float:
    max_int = float((1 << bits) - 1)
    return float(x) * (x_max - x_min) / max_int + x_min
```

---

## 解析示例（Python）

```python
from dataclasses import dataclass

@dataclass
class DamiaoFeedback:
    id: int
    err: int
    pos_raw: int
    vel_raw: int
    torque_raw: int
    pos_rad: float
    vel_rad_s: float
    torque_nm: float
    t_mos: int
    t_rotor: int


def parse_damiao_feedback(data: bytes, p_min=-12.5, p_max=12.5,
                          v_min=-45.0, v_max=45.0,
                          t_min=-8.0, t_max=8.0):
    fb_id   = data[0] & 0x0F
    fb_err  = (data[0] >> 4) & 0x0F
    pos_raw = (data[1] << 8) | data[2]
    vel_raw = (data[3] << 4) | (data[4] >> 4)
    tq_raw  = ((data[4] & 0x0F) << 8) | data[5]
    t_mos   = data[6]
    t_rotor = data[7]

    return DamiaoFeedback(
        id=fb_id,
        err=fb_err,
        pos_raw=pos_raw,
        vel_raw=vel_raw,
        torque_raw=tq_raw,
        pos_rad=uint_to_float(pos_raw, p_min, p_max, 16),
        vel_rad_s=uint_to_float(vel_raw, v_min, v_max, 12),
        torque_nm=uint_to_float(tq_raw, t_min, t_max, 12),
        t_mos=t_mos,
        t_rotor=t_rotor,
    )
```

`P_MIN / P_MAX / V_MIN / V_MAX / T_MIN / T_MAX` 需按具体达妙型号 datasheet 确认。

---

## 实测帧验证

反馈帧：

```
13 9F 51 81 E9 84 21 1B
```

拆解：

```
D0 = 0x13  →  ID = 3, ERR = 1
POS_raw    = 0x9F51 = 40785
VEL_raw    = 0x81E  = 2078
Torque_raw = 0x984  = 2436
T_MOS      = 0x21  = 33°C
T_Rotor    = 0x1B  = 27°C

torque_nm  = 2436 / 4095 * 16 - 8 ≈ +1.52 Nm
```

---

> 核心逻辑：先按 bit 位拆出原始整数，再用达妙给定的物理范围做线性映射。
