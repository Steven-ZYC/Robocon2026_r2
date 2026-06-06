# Damiao motor feedback published after every control command.
# All values are output-side (after gear ratio conversion).

uint8 motor_id    # Damiao CAN motor ID
float64 q_rad     # output-side position (rad)
float64 dq_rad_s  # output-side velocity (rad/s)
float64 tau_nm    # output-side torque (Nm)
uint8 enabled     # 1 = enabled, 0 = disabled
