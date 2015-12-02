set(MOVEIT_LIB_NAME pr2_moveit_controller_manager)

add_library(${MOVEIT_LIB_NAME} src/pr2_moveit_controller_manager.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} pr2_mechanism_msgs_generate_messages_cpp pr2_controllers_msgs_generate_messages_cpp)

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

