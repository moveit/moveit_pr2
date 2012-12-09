cmake_minimum_required(VERSION 2.8.3)
project(pr2_moveit_tests)

## Find catkin and system dependencies

## Find catkin and any catkin packages on which
## this package depends at build time
find_package(catkin REQUIRED COMPONENTS moveit_core moveit_ros_planning pluginlib)

find_package(PkgConfig REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

pkg_check_modules(FCL REQUIRED fcl)
include_directories(${FCL_INCLUDE_DIRS})
link_directories(${FCL_LIBRARY_DIRS})

pkg_check_modules(OCTOMAP REQUIRED octomap)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})


## Declare things to be passed to other projects
catkin_package()

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_subdirectory(kinematics)
