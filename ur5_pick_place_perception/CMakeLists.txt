cmake_minimum_required(VERSION 3.8)
project(ur5_pick_place_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation kdtree)

# Object detector node
add_executable(object_detector
  src/object_detector.cpp
  src/clustering.cpp
)

target_include_directories(object_detector
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(object_detector
  ${PCL_LIBRARIES}
)

ament_target_dependencies(object_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  pcl_ros
  pcl_conversions
  visualization_msgs
)

install(TARGETS object_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_clustering
    test/test_clustering.cpp
    src/clustering.cpp
  )

  target_include_directories(test_clustering
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
      ${PCL_INCLUDE_DIRS}
  )

  target_link_libraries(test_clustering
    ${PCL_LIBRARIES}
  )
endif()

ament_package()
