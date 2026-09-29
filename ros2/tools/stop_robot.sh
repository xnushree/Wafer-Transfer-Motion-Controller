#!/bin/bash
# Stop every process started by the wafer_transfer_robot launch file.
# Stopping only `ros2 launch` can leave its nodes running, and two simulators
# then publish conflicting /clock messages.

pkill -TERM -f 'ros2 launch wafer_transfer_robot'
pkill -TERM -f 'wafer_transfer_robot/lib'
pkill -TERM rviz2
pkill -TERM -f robot_state_publisher
sleep 3
echo stopped
