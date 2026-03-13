# Raw sensor data from Arduino
# Format: ID=XXX T=XXX IMU=... ENC=... DEG=... crc=XX

std_msgs/Header header

uint32 packet_id        # Package ID from Arduino
uint32 timestamp_ms     # Arduino millis() timestamp

# IMU data (LPBUS format)
float32 imu_heading_deg    # Heading in degrees [0, 360)
float32 imu_rate_rad_s     # Angular rate in rad/s
float32 imu_ax             # Acceleration X (normalized)
float32 imu_ay             # Acceleration Y (normalized)
float32 imu_az             # Acceleration Z (normalized)

# Encoder data (AMT103, CPR=8192)
int64 enc_x_counts         # X-axis encoder counts (horizontal motion)
int64 enc_y_counts         # Y-axis encoder counts (vertical motion)
float32 enc_x_deg          # X-axis position in degrees [0, 360)
float32 enc_y_deg          # Y-axis position in degrees [0, 360)

bool crc_valid             # CRC8-ATM validation result
