add_executable(kinematic_model_tutorial src/kinematic_model_tutorial.cpp)
target_link_libraries(kinematic_model_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS kinematic_model_tutorial DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_executable(ros_api_tutorial src/ros_api_tutorial.cpp)
target_link_libraries(ros_api_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS ros_api_tutorial DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
