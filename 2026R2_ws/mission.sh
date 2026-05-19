#!/bin/bash

# Launch FSM mission nodes manually with one gnome-terminal window per node.
# 用法: ./mission.sh [mission_file]
#       默认 mission_file: routes/red_area.yaml

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MISSION_FILE="${1:-$SCRIPT_DIR/src/navigation/routes/red_area.yaml}"

# ----------------------------
# Start base and motion drivers
# ----------------------------
echo "Starting chassis damiao_node..."
gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash && source $SCRIPT_DIR/install/setup.bash && ros2 run base_omniwheel_r2_700 damiao_node; exec bash"

echo "Starting arduino_sensor_parser..."
gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash && source $SCRIPT_DIR/install/setup.bash && ros2 run arduino_sensor_driver arduino_sensor_parser; exec bash"

echo "Starting local_navigation_node..."
gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash && source $SCRIPT_DIR/install/setup.bash && ros2 run base_omniwheel_r2_700 local_navigation_node; exec bash"

echo "Starting arm launch..."
gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash && source $SCRIPT_DIR/install/setup.bash && ros2 launch arm arm.launch.py; exec bash"

echo "Starting pneu_ctrl_node..."
gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash && source $SCRIPT_DIR/install/setup.bash && ros2 run pneumatics pneu_ctrl_node; exec bash"

# ----------------------------
# Start FSM navigation (替代 joystick_control_node)
# ----------------------------
echo "Starting global_navigation_node (mission: $MISSION_FILE)..."
gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash && source $SCRIPT_DIR/install/setup.bash && ros2 run navigation global_navigation_node --ros-args -p mission_file:=$MISSION_FILE; exec bash"

# ----------------------------
# Monitor navigation status
# ----------------------------
echo "Starting /global_nav/status monitor..."
gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash && source $SCRIPT_DIR/install/setup.bash && sleep 0.5 && ros2 topic echo /global_nav/status; exec bash"

echo "✅ All nodes launched successfully."
