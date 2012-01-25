cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(pr2_arm_kinematics_lib
  src/pr2_arm_ik.cpp
  src/pr2_arm_kinematics_utils.cpp
  src/pr2_arm_ik_solver.cpp
  src/pr2_arm_kinematics.cpp
  src/pr2_arm_kinematics_plugin.cpp
)

rosbuild_add_executable(pr2_arm_kinematics_node src/main.cpp)
target_link_libraries(pr2_arm_kinematics_node pr2_arm_kinematics_lib)

#rosbuild_add_executable(ik_with_cost_test test/test_ik_with_cost.cpp)
#target_link_libraries(ik_with_cost_test pr2_arm_ik)

#rosbuild_add_executable(pr2_ik_regression_test test/test_ik_regress.cpp)
#rosbuild_declare_test(pr2_ik_regression_test)
#target_link_libraries(pr2_ik_regression_test pr2_arm_ik gtest)
#rosbuild_add_rostest(test_pr2_arm_ik.xml)

## Check whether there's an X display.  Intended to use in gating tests that
## require a display.
## TODO: after ROS 0.10 is released, remove this re-declaration
#macro(rosbuild_check_for_display var)
#  execute_process(COMMAND "xdpyinfo"
#                  OUTPUT_VARIABLE _dummy
#                  ERROR_VARIABLE _dummy
#                  RESULT_VARIABLE _xdpyinfo_failed)
#  if(_xdpyinfo_failed)
#    set(${var} 0)
#  else(_xdpyinfo_failed)
#    set(${var} 1)
#  endif(_xdpyinfo_failed)
#endmacro(rosbuild_check_for_display)

#if(1)
#  rosbuild_add_executable(pr2_ik_node_regression_test test/test_ik_node_regress.cpp)
#  rosbuild_declare_test(pr2_ik_node_regression_test)
#  target_link_libraries(pr2_ik_node_regression_test pr2_arm_ik gtest)
#  rosbuild_add_rostest(test_pr2_arm_ik_node.xml)
#endif(1)
