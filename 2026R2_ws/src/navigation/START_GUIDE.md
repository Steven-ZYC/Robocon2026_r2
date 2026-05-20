# Navigation 套件啟動指南

## 系統架構

```
[Simplified Localization] -> /state_pose2d -> [Global Navigation Node] -> /local_driving -> [Local Navigation Node] -> [Base Controller]
```

`/state_pose2d` 使用 `geometry_msgs/Pose2D`，字段約定如下：
- `x`: 向前為正
- `y`: 向左為正
- `theta`: yaw，逆時針為正，單位為弧度

## 啟動步驟

### 方法 1: 使用 Launch File（推薦）

```bash
# 1. Source workspace
source ~/robotics/Robocon2026_r2/2026R2_ws/install/setup.bash

# 2. 啟動全局導航節點
ros2 launch navigation navigation.launch.py

# 可選：指定自定義路徑文件
ros2 launch navigation navigation.launch.py route_file:=/path/to/custom_route.yaml
```

### 方法 2: 手動啟動節點

```bash
# Source workspace
source ~/robotics/Robocon2026_r2/2026R2_ws/install/setup.bash

# 啟動節點（需指定參數文件）
ros2 run navigation global_navigation_node --ros-args \
  --params-file src/navigation/config/global_nav_params.yaml
```

## 完整系統啟動流程

### 1. 啟動底盤控制
```bash
# Terminal 1: 啟動底盤節點（damiao + local_navigation）
source ~/robotics/Robocon2026_r2/2026R2_ws/install/setup.bash
ros2 launch base_omniwheel_r2_600 base.launch.py
```

### 2. 啟動感測器 / 二維定位
```bash
# Terminal 2: 啟動傳感器解析節點（發布 /state_pose2d）
source ~/robotics/Robocon2026_r2/2026R2_ws/install/setup.bash
ros2 launch arduino_sensor_driver arduino_sensor.launch.py
```

### 3. 啟動全局導航
```bash
# Terminal 3: 啟動全局導航
source ~/robotics/Robocon2026_r2/2026R2_ws/install/setup.bash
ros2 launch navigation navigation.launch.py
```

## 測試與調試

### 檢查 Topics
```bash
# 查看所有話題
ros2 topic list

# 應該看到：
# /state_pose2d       (輸入 - 機器人平面位姿)
# /local_driving      (輸出 - 運動指令)
# /global_nav/status  (調試 - 導航狀態)
# /global_nav/target_pose (調試 - 目標位置)
```

### 監控狀態
```bash
# 監控導航狀態
ros2 topic echo /global_nav/status

# 監控輸入位姿
ros2 topic echo /state_pose2d

# 監控運動指令
ros2 topic echo /local_driving

# 監控目標位姿
ros2 topic echo /global_nav/target_pose
```

### 模擬測試（無硬件）
```bash
# 發布模擬的二維位姿數據
ros2 topic pub /state_pose2d geometry_msgs/Pose2D "{
  x: 0.0,
  y: 0.0,
  theta: 0.0
}"
```

## 路徑配置

當前路徑 (route_A.yaml) 配置：
- **起點**: (0, 0), 朝向: 0° → 等待 0.5s
- **第一點**: (1.0, 0), 朝向: 180° → 等待 1s
- **第二點**: (1.5, 0), 朝向: 0° → 等待 1s

修改路徑：編輯 `src/navigation/routes/route_A.yaml`

## 參數調整

編輯 `src/navigation/config/global_nav_params.yaml`:
- `control_rate_hz`: 控制頻率（默認 50Hz）
- `max_cmd_speed_mps`: 最大速度
- `max_cmd_omega_rps`: 最大角速度
- `arrived_stable_count`: 到點穩定計數閾值

## 常見問題

1. **節點無法啟動**: 檢查是否已 source workspace
2. **無運動指令輸出**: 檢查是否有 `/state_pose2d` 輸入
3. **機器人方向判斷異常**: 檢查 `theta` 是否使用弧度，且逆時針為正
4. **座標方向不對**: 檢查輸入是否符合 `x` 向前、`y` 向左 的 REP 103 平面約定
5. **到點不穩定**: 增大 `arrived_stable_count` 或調整 waypoint 容差
6. **速度不平滑**: 調整 segment 的 `start_radius_m` 和 `end_radius_m`