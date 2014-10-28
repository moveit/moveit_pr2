cmake_minimum_required(VERSION 2.8.3)
project(pr2_moveit_plugins)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin COMPONENTS
  moveit_core
  pluginlib
  actionlib
  roscpp
  rosconsole
  urdf
  kdl_parser
  tf
  tf_conversions
  eigen_conversions
  control_msgs
  pr2_mechanism_msgs
  pr2_controllers_msgs
  cmake_modules
  REQUIRED)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED thread)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  DEPENDS
    moveit_core
    )

include_directories(pr2_arm_kinematics/include)
include_directories(pr2_moveit_controller_manager/include)
include_directories(pr2_moveit_sensor_manager/include)
#include_directories(pr2_moveit_constraint_sampler/include)


include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(pr2_arm_kinematics)
add_subdirectory(pr2_moveit_controller_manager)
add_subdirectory(pr2_moveit_sensor_manager)
#add_subdirectory(pr2_moveit_constraint_sampler)

install(
  FILES
    pr2_moveit_kinematics_plugin_description.xml
    pr2_moveit_controller_manager_plugin_description.xml
    pr2_moveit_sensor_manager_plugin_description.xml
#    pr2_moveit_constraint_sampler_plugin_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

