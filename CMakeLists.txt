cmake_minimum_required(VERSION 2.8.3)
project(move_base)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

catkin_package()

add_executable(move_base src/move_base.cpp)
target_link_libraries(move_base ${catkin_LIBRARIES} phidget21)
