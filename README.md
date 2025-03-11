# mp2
the second iteration of the moonpie repository

but will it be better?

#dependencies
i'm going to try to list all of the packages that need to be installed via apt in apt-packages.txt.
you will have to install ros2 humble following the instructions at this link [ROS Humble Installation on Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
also I think there are a bunch of python packages that need to be installed


#moonpie-lari
This directory holds the nodes for the raspberry pi "Lari", which handles communications with all of the sensors and motors in the robot.

#moonpie-osamu
How to install the librealsense ros:
follow the instructions on the github, but do two other things
1. pip install colcon-common-extensions
2. pip install empy==3.3.4

#ardu 
This directory is the arduino project for the arduino connected to "Lari". The two should bnnected via usb
