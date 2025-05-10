cmake_minimum_required(VERSION 3.8)
project(moonpie_osamu)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(aruco_opencv_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

# Declare message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BehaviorStatus.msg"
  "msg/MissionCommand.msg"
  DEPENDENCIES std_msgs
)

# Ensure message generation occurs before building executables
ament_export_dependencies(rosidl_default_runtime)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Set up Qt MOC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Create the mission control executable
add_executable(mission_control
  moonpie_osamu/mission_control.cpp
)

target_include_directories(mission_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

# Link against generated message dependencies
ament_target_dependencies(mission_control
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  aruco_opencv_msgs
)

# Add dependency on generated message targets
add_dependencies(mission_control ${PROJECT_NAME}__rosidl_typesupport_cpp)

# Link against the message type support
target_link_libraries(mission_control
  ${rclcpp_LIBRARIES}
  ${rclcpp_action_LIBRARIES}
  ${PROJECT_NAME}__rosidl_typesupport_cpp
  tf2_geometry_msgs::tf2_geometry_msgs
)

# Install the mission control executable
install(TARGETS mission_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install RViz config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Add mission control panel executable (AFTER message generation)
add_executable(mission_control_panel
  src/mission_control_panel.cpp
  include/moonpie_osamu/mission_control_panel.hpp
)

target_include_directories(mission_control_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

# Link libraries for mission control panel
target_link_libraries(mission_control_panel
  rclcpp::rclcpp
  Qt5::Widgets
  ${OpenCV_LIBS}
  moonpie_osamu__rosidl_typesupport_cpp
)

ament_target_dependencies(mission_control_panel
  std_msgs
  cv_bridge
  sensor_msgs
)

# Ensure message headers are available before building the executable
add_dependencies(mission_control_panel ${PROJECT_NAME}__rosidl_typesupport_cpp)

# Install mission control panel
install(TARGETS
  mission_control_panel
  DESTINATION lib/${PROJECT_NAME}
)

ament_package() 