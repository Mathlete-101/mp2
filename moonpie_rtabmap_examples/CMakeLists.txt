cmake_minimum_required(VERSION 3.5)
project(moonpie_rtabmap_examples)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
   DESTINATION share/${PROJECT_NAME}
)

ament_package()
