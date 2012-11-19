cmake_minimum_required(VERSION 2.8.3)
project(pr2_moveit_plugins)

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
  REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  LIBRARIES
    pr2_moveit_arm_kinematics
    pr2_moveit_controller_manager
  INCLUDE_DIRS
    pr2_arm_kinematics/include
  DEPENDS
    moveit_core
    )

include_directories(pr2_arm_kinematics/include)
include_directories(pr2_moveit_controller_manager/include)
#include_directories(pr2_moveit_sensor_manager/include)
#include_directories(pr2_moveit_constraint_sampler/include)

add_subdirectory(pr2_arm_kinematics)
#add_subdirectory(pr2_moveit_controller_manager)
#add_subdirectory(pr2_moveit_sensor_manager)
#add_subdirectory(pr2_moveit_constraint_sampler)
#add_subdirectory(pr2_gripper_controller)

install(
  FILES
    pr2_kinematics_plugin_description.xml
    pr2_moveit_controller_manager_plugin_description.xml
#    pr2_moveit_sensor_manager_plugin_description.xml
#    pr2_moveit_constraint_sampler_plugin_description.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
