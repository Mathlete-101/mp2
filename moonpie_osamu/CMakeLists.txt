cmake_minimum_required(VERSION 3.8)
project(moonpie_osamu)

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

# Include directories
include_directories(
  include
)

# Create the mission control executable
add_executable(mission_control
  moonpie_osamu/mission_control.cpp
)

target_include_directories(mission_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(mission_control
  ${rclcpp_LIBRARIES}
  ${rclcpp_action_LIBRARIES}
)

ament_target_dependencies(mission_control
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  std_msgs
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

ament_package() 