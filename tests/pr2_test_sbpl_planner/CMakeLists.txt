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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# rosbuild_add_executable(test_pr2_sbpl_planner test/pr2_sbpl_planner_test.cpp)
# rosbuild_add_gtest_build_flags(test_pr2_sbpl_planner TIMEOUT 10.0)
# rosbuild_add_rostest(test/test_sbpl_planner.test)

rosbuild_add_executable(test_pr2_sbpl_ompl_planner test/pr2_sbpl_ompl_planner_test.cpp)
rosbuild_add_gtest_build_flags(test_pr2_sbpl_ompl_planner TIMEOUT 600.0)
rosbuild_add_rostest(test/test_sbpl_ompl_planner.test)
