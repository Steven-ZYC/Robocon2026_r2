# Raw sensor data from Arduino.
# Format: ID=XXX T=XXX IMU=... ENC=... crc=XX

std_msgs/Header header

uint32 packet_id        # Package ID from Arduino, unitless
uint32 timestamp_ms     # Arduino millis() timestamp, unit: ms

# IMU data (LPBUS format)
float32 imu_heading_deg    # Heading, unit: deg
float32 imu_rate_rad_s     # Angular rate around Z, unit: rad/s
float32 imu_ax             # Acceleration X, normalized by g
float32 imu_ay             # Acceleration Y, normalized by g
float32 imu_az             # Acceleration Z, normalized by g

# Encoder data in the robot body frame, following ROS REP 103.
int64 enc_x_counts         # REP X forward-positive accumulated counts, unit: counts
int64 enc_y_counts         # REP Y left-positive accumulated counts, unit: counts

# Reserved compatibility fields. The current Arduino firmware does not emit DEG.
float32 enc_x_deg          # Unit: deg, currently published as 0.0
float32 enc_y_deg          # Unit: deg, currently published as 0.0

bool crc_valid             # CRC8-ATM validation result, unitless
bool imu_ok                 # IMU health flag from Arduino (IMU_OK=1 means connected)
