cmake_minimum_required(VERSION 2.8.3)
project(pr2_moveit_plugins)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})

find_package(Boost REQUIRED thread)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

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
  REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  DEPENDS
    moveit_core
    )

include_directories(hack_for_groovy)

include_directories(pr2_arm_kinematics/include)
include_directories(pr2_moveit_controller_manager/include)
include_directories(pr2_moveit_sensor_manager/include)
#include_directories(pr2_moveit_constraint_sampler/include)

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

# HACK
install(DIRECTORY hack_for_groovy/ DESTINATION include)