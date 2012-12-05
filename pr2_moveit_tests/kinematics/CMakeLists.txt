catkin_add_gtest(pr2_kinematics_test src/test_kinematics_as_plugin.cpp)
target_link_libraries(pr2_kinematics_test ${catkin_LIBRARIES})
install(TARGETS pr2_kinematics_test RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
