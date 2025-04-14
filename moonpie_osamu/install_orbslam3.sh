#! /bin/bash

# Create orbslam3 directory only if it doesn't exist
if [ ! -d "orbslam3" ]; then
    mkdir orbslam3
fi

cd orbslam3

# install dependencies

# ros-humble-cv-bridge
sudo apt-get install ros-humble-cv-bridge

# eigen 3.4.0
sudo apt-get install libeigen3-dev

# pangolin
sudo apt-get install libglew-dev #pangolin dependency

#download pangolin
if [ ! -d "Pangolin" ]; then
    git clone https://github.com/stevenlovegrove/Pangolin.git
    cd Pangolin
    ./scripts/install_prerequisites.sh
    git checkout v0.6

    # Add missing include to color.h file
    echo '#include <limits>' > temp_include
    cat Pangolin/include/pangolin/gl/color.h >> temp_include
    mv temp_include Pangolin/include/pangolin/gl/color.h
else
    cd Pangolin
fi

#build pangolin
if [ ! -d "build" ]; then
    mkdir build
fi
cd build
cmake ..
make
make install

#clone orbslam3
cd ../..
if [ ! -d "ORB_SLAM3" ]; then
    git clone https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git ORB_SLAM3
fi

cd ORB_SLAM3

# Replace 'monotonic' with 'steady' in the RGB-D-Inertial example file
# shouldn't be needed in the fixed version
# sed -i 's/monotonic/steady/g' Examples/RGB-D-Inertial/rgbd_intertial_realsense_D435i.cc

chmod +x build.sh
./build.sh

#add /lib to path
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib:./lib

#save the ORB_SLAM3 path to a variable
ORB_SLAM3_PATH=$(pwd)

#install orbslam3 ros2 wrapper
cd ../..
git clone git@github.com:zang09/ORB-SLAM3-STEREO-FIXED.git ORB_SLAM3_ROS2
cd ORB_SLAM3_ROS2

#save the python site-packages path to a variable
PYTHON_SITE_PACKAGES=$(python3 -c "import site; print(site.getsitepackages()[0])")

#change line 5 of ORB_SLAM3_ROS2/CMakeLists.txt so that the thing in the "" is the python site-packages path
sed -i '5s|.*|set(PYTHON_SITE_PACKAGES \"${PYTHON_SITE_PACKAGES}\")|' CMakeLists.txt

#change line 8 of ORB_SLAM3_ROS2/CMakeModules/FindORB_SLAM3.cmake so that the thing in the "" is the ORB_SLAM3 path
sed -i '8s|.*|set(ORB_SLAM3_PATH \"${ORB_SLAM3_PATH}\")|' CMakeModules/FindORB_SLAM3.cmake

#build orbslam3 ros2 wrapper
colcon build --symlink-install --packages-select orbslam3_ros2

