#! /bin/bash

mkdir orbslam3
cd orbslam3

# install dependencies

# ros-humble-cv-bridge
sudo apt-get install ros-humble-cv-bridge

# eigen 3.4.0
sudo apt-get install libeigen3-dev

# pangolin
sudo apt-get install libglew-dev #pangolin dependency

#download pangolin
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
./scripts/install_prerequisites.sh
git checkout v0.6

# Add missing include to color.h file
echo '#include <limits>' > temp_include
cat Pangolin/include/pangolin/gl/color.h >> temp_include
mv temp_include Pangolin/include/pangolin/gl/color.h

#build pangolin
mkdir build
cd build
cmake ..
make
make install

#clone orbslam3
git clone -b c++14_comp https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3

cd ORB_SLAM3

# Replace 'monotonic' with 'steady' in the RGB-D-Inertial example file
sed -i 's/monotonic/steady/g' Examples/RGB-D-Inertial/rgbd_intertial_realsense_D435i.cc

chmod +x build.sh
./build.sh

#add /lib to path
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib:./lib