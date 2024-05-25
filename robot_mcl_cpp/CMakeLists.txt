cmake_minimum_required(VERSION 3.5)
project(robot_mcl_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find package dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
# Declare a C++ executable
add_executable(mcl_node src/mcl_node_full.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(mcl_node
  rclcpp
  tf2_geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
  tf2
  OpenCV
)

# Install the executable
install(TARGETS
  mcl_node
  DESTINATION lib/${PROJECT_NAME}
)
# Specify include directories
# Install header files
install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/)
include_directories(include)
target_link_libraries(mcl_node
  ${OpenCV_LIBRARIES}
)
# Export dependencies
ament_package()
