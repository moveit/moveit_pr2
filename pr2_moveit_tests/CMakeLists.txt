cmake_minimum_required(VERSION 2.8.3)
project(pr2_moveit_tests)

add_compile_options(-std=c++11)

## Find catkin and any catkin packages on which
## this package depends at build time
find_package(catkin REQUIRED
  COMPONENTS
  moveit_core
  moveit_ros_planning
  pluginlib
  rostest
  rosunit
)

find_package(PkgConfig REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

## Specify additional locations of header files
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare things to be passed to other projects
catkin_package()

add_subdirectory(kinematics)
