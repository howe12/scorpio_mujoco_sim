cmake_minimum_required(VERSION 3.8)
project(terrain_analyzer)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(se2_grid_msgs REQUIRED)
find_package(se2_grid_core REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)

add_executable(terrain_analyzer_node src/terrain_analyzer.cpp)

target_include_directories(terrain_analyzer_node PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${se2_grid_core_INCLUDE_DIRS}
)

target_link_libraries(terrain_analyzer_node
  ${se2_grid_core_LIBRARIES}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(terrain_analyzer_node
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
  geometry_msgs
  se2_grid_msgs
  pcl_conversions
)

install(TARGETS terrain_analyzer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
