# MoonPie 2.0 (mp2)
Nathan Jensen ([nath.h.jensen@gmail.com](mailto:nath.h.jensen@gmail.com))
Feel free to email me if you have questions about how our robot works/how to make a better robot/autonomy system then we did.

This repository contains the ROS 2 Humble workspace for the MoonPie robot project. It includes various packages for robot control, navigation, and simulation.

## Notes

- You should not use anything straight from this repository, it's all garbage and needs to be redone. It might be useful to take ideas or snippets from though.
- Ros2 is good and you should probably use it. Ros nodes autoconnect over the wifi, but the firewall has to be configured correctly or disabled.
- The Intel Realsense D435 is not bad, but librealsense is difficult to set up. You should ask me or Dr. Petruska and his crew on how to get it set up. The Realsense T265 is also really good, but librealsense versions get more complicated if you want to use it
- Ditch the raspberry pi. You'll want a good computer on the robot, don't mess around with using a bad one other than an arduino to control the motors. A jetson isn't a bad choice, but I haven't tried the librealsense jeston installation. Unless you want to run a neural net, which probably is unnessecary, I think that the NUC in the second e-box is a fine computer.
- For both computers, the username is moonpie and the password is moonpie for the main, auto login account
- Also if you're reusing the computers, both of them have services that auto launch named lari.service and osamu.service. You can disable them with sudo systemctl disable _service name_.
- If you use a main computer/arduino architecture again (which I think isn't a bad idea), then you need to be really careful about how you send data to the arduino. We used a very nicely labeled json to send data over usb connection, which meant that it was flexible and easy to change things, but when we got to competition, it caused huge problems. We had done throughput limiting to ensure that things were happening correctly on several levels, but it was always pretty hackish and if you throughput limit on something that isn't the direct connection to the arduino, then the network delays make your messages get too close and then the overlap and don't get through. A lot of failures we had were due to this fact, and you have to get this right.
- IMU's are worthless. Not worth the effort, and there wasn't a lot of effort we put in to making it work.
- Simulations are good but if there's only one main coding person then you don't need to. I think that if you're trying to do an auto movment system, getting our robot working and just testing on that isn't a bad idea. Also, some of the team members would probably disagree, but I think that you guys should build an entirely new robot. Ours had several issues, mainly about the power level and gearing of some of the motors, and it will probably be easiest just to start from scratch. Plus, you'll have a whole robot to test autonomy on.
- In the mission control room, you need to have two people: a code person and a driver. They should not be the same person, so that the driver can drive while the code person fixes the code. 
- Autonomy should be developed in strict order. I thought that I should go immediately for the most autonomous systems and then fall back, but that makes for a really bad experience at competiton. You need to get the easy bits working out to find all the bugs at the easy levels of that system, and then when you make harder systems, you have experience with the infrastructre and a well tuned base system. So you should make solid teleop, and then make solid single dig, etc. In our competition, I don't think that full auto was worth the points for any team that attempted it. It's only worth 100 more points than the other things combined (based on my reading of the rules), and it's much more difficult to achieve and will dig less regolith.
- The regolith at UCF and KSC are very different.
- You should do full competition-like runs before going to competition. It makes things much less stressful when you have to go into the competition for the first time.
- Send me an email if you have question about how things work in competition, or ESPECIALLY if you're struggling with librealsense stuff. It's super annoying and, at least as of may 22, I know a fair bit about how to get it to work.

## Project Structure

This is a description of all the things in the repo written with chat. Note that none of it is very good and you should redo all of it.

- `moonpie_lari/` - ROS nodes for the Raspberry Pi "Lari" that handles sensor and motor communications
- `moonpie_osamu/` - ROS nodes for the RealSense camera system
- `ardu/` - Arduino project for the microcontroller connected to "Lari"
- `nav2_straightline_planner/` - Custom navigation planner for ROS 2 Navigation Stack
- `moonpie_rtabmap_examples/` - Examples and configurations for RTAB-Map SLAM
- `sim3/` - Simulation environment and configurations
- `startup_scripts/` - System startup and initialization scripts
- `nav_test/` - Navigation testing and debugging tools
- `imu/` - IMU sensor integration and processing
- `beacon/` - Beacon detection and tracking system

## Prerequisites

1. Ubuntu 22.04
2. ROS 2 Humble (follow [official installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html))
3. Required system packages (install using `apt-packages.txt`):
   ```bash
   sudo apt install $(cat apt-packages.txt)
   ```
   ***note: these have not been well maintained. Likely there are missing dependencies that will also need to be installed
4. Intel RealSense SDK and ROS wrapper (follow [librealsense installation guide](https://github.com/IntelRealSense/realsense-ros))

## Setup and Building

1. Clone the repository:
   ```bash
   git clone --recursive [repository-url]
   cd mp2
   ```
   ***note: not all machines need all of the submodules.

2. Source ROS 2 environment (add to `~/.bashrc` for persistence):
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. Build the workspace:
   ```bash
   colcon build
   ```

4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Running the System

The system was run on three computers:
- The mission control computer
    - this computer is my raspberry pi. There's I think at least one somewhere in all the stuff we left you, you can slap all the dependencies on it 
- The raspberry pi on the robot (named Lari), which handles communication with the arduino
- The intel nuc in the second e-box (named Osamu), which handles autonomy tasks

***note: due to bad design choices, all of the computers need to be able to compile the moonpie_osamu package.

## The packages

### moonpie_lari
The main package for the Raspberry Pi "Lari" that handles low-level robot control and sensor communication.
- `arduino_control`: Main node for controlling the Arduino microcontroller
- `arduino_comms`: Handles communication with the Arduino
- `beacon_control`: Manages beacon detection and tracking

### moonpie_osamu
The package for the Intel NUC "Osamu" that handles high-level autonomy tasks.
- Contains URDF models for robot visualization
- Manages RealSense camera integration
- Handles navigation and SLAM tasks

Launch Files:
- `osamu.launch.py`: Main launch file that starts the complete system
  - Robot visualization (robot state publisher and RViz)
  - RealSense D435 camera with pointcloud
  - SLAM nodes for mapping
  - ArUco marker detection
  - Navigation system
  - Mission control system

- `slam.launch.py`: RTAB-Map SLAM configuration
  - RGBD Synchronization
  - RTAB-Map core with depth input
  - Optional RTAB-Map visualization
  - Optional EKF for robot localization
  - Configurable IMU integration

- `camera_only.launch.py`: RealSense camera configuration
  - D435 camera driver setup
  - Pointcloud generation
  - Camera parameter configuration

- `aruco.launch.py`: ArUco marker detection system
  - Marker detection and tracking
  - Pose estimation
  - Visualization tools

- `mission_control.launch.py`: Mission execution system
  - Mission planning and execution
  - Task management
  - State machine control

- `robot_visualization.launch.py`: Robot visualization tools
  - RViz configuration
  - Robot state publisher
  - TF tree visualization

- `robot_description.launch.py`: Robot model configuration
  - URDF loading
  - Joint state publisher
  - TF configuration

- Development and Testing Launch Files:
  - `osamu_dev.launch.py`: Development configuration with debugging tools
  - `test_realsense_d435i_*.launch.py`: Various RealSense camera test configurations
    - Color camera testing
    - Infrared camera testing
    - Stereo camera testing

### nav2_straightline_planner
A custom navigation planner for the ROS 2 Navigation Stack.
- Provides a simple straight-line path planning strategy
- Integrates with Nav2 for robot navigation
- Used for basic point-to-point navigation tasks

### moonpie_rtabmap_examples
Examples and configurations for RTAB-Map SLAM.
- Contains SLAM configurations and parameters
- Includes example launch files for mapping and localization
- Provides visualization tools for SLAM data

### sim3
Simulation environment and configurations.
- Contains Gazebo simulation models
- Includes simulation launch files
- Provides testing environment for autonomy algorithms

### startup_scripts
System initialization and startup scripts.
- Contains scripts for automatic system startup
- Includes configuration files for different robot modes
- Provides utilities for system management

### nav_test
Navigation testing and debugging tools.
- Contains test scenarios for navigation
- Includes debugging utilities
- Provides visualization tools for navigation testing

### imu
IMU sensor integration and processing.
- Handles IMU data processing
- Provides sensor fusion capabilities
- Includes calibration tools

### beacon
Beacon detection and tracking system.
- Manages beacon detection algorithms
- Handles beacon tracking and localization
- Provides visualization tools for beacon data

