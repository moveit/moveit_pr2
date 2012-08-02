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

rosbuild_add_executable(pr2_ik_regression_test test/test_ik_regress.cpp)
rosbuild_declare_test(pr2_ik_regression_test)
target_link_libraries(pr2_ik_regression_test gtest)
rosbuild_add_rostest(test_pr2_arm_ik.launch)

rosbuild_add_executable(pr2_kinematics_test_as_plugin test/test_kinematics_as_plugin.cpp)
rosbuild_declare_test(pr2_kinematics_test_as_plugin)
target_link_libraries(pr2_kinematics_test_as_plugin gtest)

#rosbuild_add_rostest(test_pr2_arm_kinematics_as_plugin.launch)
rosbuild_add_rostest(test_pr2_arm_kinematics_as_kdl_plugin.launch)
