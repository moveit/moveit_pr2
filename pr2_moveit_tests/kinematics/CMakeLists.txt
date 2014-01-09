#catkin_add_gtest(pr2_kinematics_test src/test_kinematics_as_plugin.cpp)
#target_link_libraries(pr2_kinematics_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})

find_package(orocos_kdl)

add_rostest_gtest(pr2_kinematics_test launch/pr2_plugin_test.launch src/test_kinematics_as_plugin.cpp)
if(TARGET pr2_kinematics_test)
  target_link_libraries(pr2_kinematics_test ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES})
endif()

add_rostest_gtest(pr2_kinematics_kdl_test launch/kdl_plugin_test.launch src/test_kinematics_as_plugin.cpp)
if(TARGET pr2_kinematics_kdl_test)
  target_link_libraries(pr2_kinematics_kdl_test ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES})
endif()

#catkin_add_gtest(pr2_constraint_aware_kinematics_test src/test_constraint_aware_kinematics.cpp)
#target_link_libraries(pr2_constraint_aware_kinematics_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_rostest_gtest(pr2_jacobian_test launch/pr2_jacobian_tests.launch src/test_jacobian.cpp)
if(TARGET pr2_jacobian_test)
  target_link_libraries(pr2_jacobian_test ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES})
endif()