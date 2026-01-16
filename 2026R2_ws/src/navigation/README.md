# navigation

Navigation package for R2.

## Nodes

- local_navigation_node
- general_navigation_node
- omni_wheel_speed_node

## Topics

- Subscribes: ps4 (sensor_msgs/Joy)
- Publishes: local_driving (Float32MultiArray)
- Publishes: general_driving (Float32MultiArray)
- Subscribes: local_driving (Float32MultiArray)
- Publishes: damiao_control (Float32MultiArray)

## General Path Map

- config/general_path.yaml
- Segment format: [x0, y0, yaw0_deg, x1, y1, yaw1_deg]
