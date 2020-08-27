cmake_minimum_required(VERSION 2.8.3)
project(human_cues_tag_simulator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  apriltag_ros
  )

# Uncomment this if the package has a setup.py. This macro ensures
# modules and global scripts declared therein get installed
# See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES human_cues_tag_simulator
  # CATKIN_DEPENDS apriltags_ros
  #  DEPENDS system_lib
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_executable(find_tags_simulated
  ./src/find_tags_simulated.cpp
  )

target_link_libraries(find_tags_simulated
  ${catkin_LIBRARIES}
  )


# Mark executable scripts (Python etc.) for installation
# in contrast to setup.py, you can choose the destination
install(PROGRAMS
  ./nodes/tag_placer
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
