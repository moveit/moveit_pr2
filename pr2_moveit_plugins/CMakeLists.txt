cmake_minimum_required(VERSION 2.8.3)
project(pr2_moveit_plugins)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  actionlib
  cmake_modules
  control_msgs
  eigen_conversions
  kdl_parser
  moveit_core
  pluginlib
  pr2_controllers_msgs
  pr2_mechanism_msgs
  rosconsole
  roscpp
  tf
  tf_conversions
  urdf
  angles
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED thread)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    control_msgs
    moveit_core
    pr2_controllers_msgs
    pr2_mechanism_msgs
)

include_directories(pr2_arm_kinematics/include)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})

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
