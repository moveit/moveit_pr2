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
rosbuild_add_executable(test_planner src/test_planner.cpp)
rosbuild_add_gtest_build_flags(test_planner)
rosbuild_add_rostest(test/test_planner.test)

rosbuild_add_executable(test_simple_plan src/test_simple_plan.cpp)
rosbuild_add_gtest_build_flags(test_simple_plan)
rosbuild_add_rostest(test/test_simple_plan.test)

rosbuild_add_executable(test_path_constrained_plan src/test_path_constrained_plan.cpp)
rosbuild_add_gtest_build_flags(test_path_constrained_plan)
rosbuild_add_rostest(test/test_path_constrained_plan.test)

#rosbuild_add_executable(test_complex_path_constrained_plan src/test_complex_path_constrained_plan.cpp)
#rosbuild_add_gtest_build_flags(test_complex_path_constrained_plan)
#rosbuild_add_rostest(test/test_complex_path_constrained_plan.test)

rosbuild_add_executable(test_dual_arm_plan src/test_dual_arm_plan.cpp)
rosbuild_add_gtest_build_flags(test_dual_arm_plan)
rosbuild_add_rostest(test/test_dual_arm.test)
