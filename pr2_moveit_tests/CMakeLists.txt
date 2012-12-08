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

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment if the package has a setup.py
# catkin_python_setup()

## Declare ROS messages and services

## Generate messages in the 'msg' folder
# add_message_files(
#   DIRECTORY msg
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   DIRECTORY srv
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
# #  std_msgs  # Or other packages containing msgs
# )

## Declare things to be passed to other projects
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS # catkin-pkg1 catkin-pkg2
#  DEPENDS Boost # other-3rd-party-libraries
)

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp library
#add_library(${PROJECT_NAME}
#
#)

## Declare a cpp executable
# add_executable(${PROJECT_NAME}_node src/pr2_moveit_tests_node.cpp)

## Add dependencies to the executable
# add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME})

## Specify libraries to link the executable against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES} ${Boost_LIBRARIES}
# )

## Mark executable scripts (Python etc.) for installation
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pr2_moveit_tests.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add any nosetest based python tests
# catkin_add_nosetests(test)

add_subdirectory(kinematics)