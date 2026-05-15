#!/usr/bin/env bash

set -e

ros2 topic pub --once /damiao_control std_msgs/msg/Float32MultiArray \
  "{data: [7.0, 2.0, 1.0, 0.0]}"
