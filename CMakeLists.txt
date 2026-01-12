cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0074 NEW)  # Fix PCL_ROOT warning

project(city_analyzer_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation features search)

add_executable(city_node src/main.cpp src/CityAnalyzer.cpp)

target_include_directories(city_node PRIVATE
  ${PCL_INCLUDE_DIRS}
  include
  src
)

# PCL + OpenMP (plain) → ROS (keyword)
target_link_libraries(city_node 
  ${PCL_LIBRARIES}
)

ament_target_dependencies(city_node
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
)

install(TARGETS city_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  launch/city_visualizer.launch.py
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
