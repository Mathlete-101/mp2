# mp2
the second iteration of the moonpie repository

but will it be better?



## moonpie-lari
This directory holds the nodes for the raspberry pi "Lari", which handles communications with all of the sensors and motors in the robot.

# dependencies
i'm going to try to list all of the packages that need to be installed via apt in apt-packages.txt.
you will have to install ros2 humble following the instructions at this link [ROS Humble Installation on Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

# installation and running
Every time you start a terminal, you need to activate ros with the following command: `source /opt/ros/humble/setup.bash`
To automate this, add that line to the end of your `~\.bashrc`.
The mp2 directory is a ros2 workspace. From it, run `colcon build`. Then, to install the nodes, run `source install/setup.bash`.
Start the arduino communication node with `ros2 run moonpie-lari arduino_control`
In a different terminal, start the joy nodes with `ros2 launch teleop_twist_joy teleop-launch.py --joy-config:='xbox'`. This can be started on the same pi as the arduino for development.

# development
You only have to source the setup file once. Then, just running colcon build will rebuild the script, and then you redo the `ros2 run` command.

## ardu 
This directory is the arduino project for the arduino connected to "Lari". The two should be connected via usb
