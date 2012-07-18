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

# rosbuild_add_executable(test_pr2_cd_speed test/pr2_moveit_speed_test.cpp)
# rosbuild_add_gtest_build_flags(test_pr2_cd_speed)
# rosbuild_add_rostest(test/test_collision_detection_speed.test)

rosbuild_add_executable(test_pr2_low_spheres_speed test/pr2_low_spheres_speed_test.cpp)
rosbuild_add_gtest_build_flags(test_pr2_low_spheres_speed)
rosbuild_add_rostest(test/test_collision_detection_low_spheres_speed.test)

rosbuild_add_executable(test_pr2_planning_scene_collision_detection_speed test/pr2_moveit_planning_scene_speed_test.cpp)
rosbuild_add_gtest_build_flags(test_pr2_planning_scene_collision_detection_speed)
rosbuild_add_rostest(test/test_planning_scene_collision_detection_speed.test)


