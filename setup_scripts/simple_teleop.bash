#!/usr/bin/bash

#for this one, you plug the controller into the pi and it controls the robot. movement only.
source /opt/ros/humble/setup.bash

#start up the teleop_twist_joy node, which starts the teleop_joy node
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
