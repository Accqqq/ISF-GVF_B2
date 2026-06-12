cmake_minimum_required(VERSION 3.0.2)
project(b2_gvf_sim)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  tf
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(b2_kinematic_sim src/b2_kinematic_sim.cpp)
target_link_libraries(b2_kinematic_sim ${catkin_LIBRARIES})
