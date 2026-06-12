# Red Area FSM Briefing

本文档用自然语言说明当前 Red Area FSM 的整体理解，供实车调试前统一队内认知。它描述的是任务流程和安全逻辑，不是 YAML 字段说明。

## 任务目标

Red Area 的任务目标是在 weapon head rack 上依次处理 1 到 6 号 weapon head position。机器人从 `weapon1/#1` 开始，每个 position 都执行同一套高层流程：到达当前点、寻找 weapon、尝试夹取、抬升后用 IR 复检、成功后执行 torque docking，然后前进到下一个 position。

整个 FSM 的成功计数以“真正夹到并完成 docking”为准。也就是说，只有在 lift high 后 IR 仍然为 true，并且后续 docking 扭矩触发释放完成，才算成功夹取一个 weapon。成功数量达到 5 个后，机器人不再继续处理后续 position，而是进入最终停机与安全姿态。

## Position 定义

`#1` 就是 `weapon1` 的位置，不是额外的参考点。FSM 只把它当作六点循环的起点。后续 weapon2 到 weapon6 按 rack 方向依次前进，每个点之间的默认间距为 0.2 m。

因此 FSM 不是“在 #1 和 weapon1 之间来回切换”，而是从 weapon1 开始沿 rack 逐点推进：weapon1、weapon2、weapon3、weapon4、weapon5、weapon6。

## 单个 Position 的流程

机器人到达当前 weapon position 后，先进入取物准备姿态：yaw 朝前、roll 向上、夹爪闭合、lift 保持 low、stopper 保持 down。随后读取 arm 侧 IR。

如果 IR 为 true，说明当前位置已经检测到 weapon，FSM 直接进入夹取流程。如果 IR 为 false，FSM 不会立刻判定失败，而是在当前位置附近执行小范围寻找。当前理解是使用 10 mm 级别的微扫：先退到当前点前侧，再慢速扫过当前位置，尽量让 IR 有机会触发。

一旦在寻找过程中 IR 变为 true，底盘立刻停止，进入夹取流程。

## 夹取与 IR 复检

夹取流程的核心动作是夹爪闭合，然后 lift high。lift high 之后必须再次读取 IR。

如果 lift high 后 IR 仍然为 true，说明 weapon 大概率已经被夹住并抬起，本次夹取判定为成功，随后进入 docking。

如果 lift high 后 IR 为 false，说明这次没有夹到，或者抬升后 weapon 已经脱离传感器有效状态。该情况一律视为本 position 夹取失败，不进入 docking。

## 失败与重试

每个 weapon position 允许重试一次。第一次 lift high 后 IR=false 时，FSM 会执行安全回位：底盘停止、夹爪打开、lift 回 low、stopper down、roll up，然后重新回到当前 position 的寻找与夹取流程。

如果同一个 position 第二次仍然 lift high 后 IR=false，则该 position 判定为失败或为空。FSM 不再继续在本点耗时，而是执行安全回位后前进到下一个 weapon position。

## Docking 逻辑

Docking 不是一个导航点，而是 torque 感应加释放动作。夹取成功后，FSM 停止底盘并等待 `/damiao_feedback.motor_5_tau` 的绝对值超过 1.3 Nm。

当 `abs(motor_5_tau) > 1.3 Nm` 时，认为 docking 接触已经发生，FSM 执行 gripper open 释放 weapon。释放完成后，本次 weapon 才计入成功数量，并进入下一步判断。

如果 torque 数据没有到达，FSM 不会释放 gripper，也不会继续前进，而是保持底盘停止并等待 torque 数据恢复或触发。

## 成功计数与停机条件

FSM 维护一个成功计数。每完成一次“夹取成功 + torque docking + gripper open 释放”，成功计数加一。

当成功计数达到 5 时，FSM 立即停止继续处理后续 position，进入最终安全姿态和停机状态。

如果 1 到 6 号 position 全部处理完，但成功计数仍然小于 5，FSM 也会进入最终安全姿态和停机状态，并在日志中提示六个点已处理完成但成功数量不足。

## 最终安全姿态

任务结束时，底盘明确停止。手臂进入最终安全姿态：yaw left、roll up、gripper open、lift low、stopper down。

这里不使用通用 terminate 的电机归零行为，因为通用 terminate 会把电机目标归到 0，可能覆盖最终需要保持的 yaw left 姿态。Red Area FSM 结束时应保留上述安全姿态，并让任务状态进入 DONE。

## 超时与安全保护

如果 `/state_pose2d` 超时，global navigation 会停止发布运动命令，底盘保持停止。

如果 IR 数据缺失或超过配置时间没有更新，FSM 会发布零 `/local_driving`。持续超时后，该次尝试按 miss 处理，避免机器人在传感器失效时继续盲动。

如果 docking torque 数据缺失，FSM 会停在 docking 等待状态，不会提前释放 gripper。

## 调试重点

实车调试时应重点观察 `/global_nav/status`、`/state_pose2d`、`/arm/ir_status` 和 `/damiao_feedback.motor_5_tau`。

需要逐项确认：slot 前进方向是否正确，0.2 m 间距是否匹配实际 rack，lift high 后 IR=false 是否只重试一次，`abs(motor_5_tau) > 1.3 Nm` 是否适合作为 docking 释放阈值，以及成功计数达到 5 后是否立即停机。
