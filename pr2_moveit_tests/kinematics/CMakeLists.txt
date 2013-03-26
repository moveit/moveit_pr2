catkin_add_gtest(pr2_kinematics_test src/test_kinematics_as_plugin.cpp)
target_link_libraries(pr2_kinematics_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_add_gtest(pr2_constraint_aware_kinematics_test src/test_constraint_aware_kinematics.cpp)
target_link_libraries(pr2_constraint_aware_kinematics_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_add_gtest(pr2_jacobian_test src/test_jacobian.cpp)
target_link_libraries(pr2_jacobian_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
