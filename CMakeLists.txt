cmake_minimum_required(VERSION 2.8.3)

project(joy_translate)

find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  turtlesim
  geometry_msgs
  sensor_msgs
)

catkin_package()

add_executable(joy_translate src/translate.cpp)

target_link_libraries(joy_translate  ${catkin_LIBRARIES})
