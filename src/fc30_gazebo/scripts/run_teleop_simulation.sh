#!/bin/bash

# Set Gazebo model paths
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/lcf/ros2_hoisting_system/src/fc30_gazebo/models/
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/usr/share/gazebo-*/models/
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/.gazebo/models/

echo "Gazebo model path: $GAZEBO_MODEL_PATH"

# Start the teleop simulation
gz sim /home/lcf/ros2_hoisting_system/src/fc30_gazebo/worlds/quadcopter_teleop.sdf