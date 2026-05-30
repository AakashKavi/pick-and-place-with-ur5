cmake_minimum_required(VERSION 3.8)
project(ur5_pick_place_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  worlds
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
