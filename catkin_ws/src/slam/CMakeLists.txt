cmake_minimum_required(VERSION 3.0.2)
project(slam)

add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2_ros
)

## Declare a catkin package
catkin_package()

## Specify locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}                  # created by find_package()
)

## Declare a C++ executable
add_executable(slam_node src/main.cpp src/local_slam_node.cpp)

## Specify libraries to link executable target against
target_link_libraries(slam_node
  ${catkin_LIBRARIES}                     # created by find_package()
)

## Add cmake target dependencies of the executable
add_dependencies(slam_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}     # list of message/service/action generation targets defined within your own package
  ${catkin_EXPORTED_TARGETS})             # list of message/service/action generation targets from all the other catkin packages you depend on in find_package()
