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

rosbuild_add_executable(call_benchmark src/call_benchmark.cpp)
rosbuild_add_executable(call_cca src/call_cca.cpp)

# Unit tests
rosbuild_add_gtest(test_planner test/test_planner.cpp)
rosbuild_add_gtest(test_simple_plan test/test_simple_plan.cpp)
rosbuild_add_gtest(test_path_constrained_plan test/test_path_constrained_plan.cpp)
rosbuild_add_gtest(test_complex_path_constrained_plan test/test_complex_path_constrained_plan.cpp)
rosbuild_add_gtest(test_dual_arm test/test_dual_arm_plan.cpp)
