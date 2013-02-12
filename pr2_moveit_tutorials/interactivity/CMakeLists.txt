add_executable(interactivity_tutorial
  src/interactivity_tutorial.cpp
  src/interactive_robot.cpp
  src/imarker.cpp
  src/pose_string.cpp
)
target_link_libraries(interactivity_tutorial
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  interactive_markers
)
target_link_libraries(interactivity_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS interactivity_tutorial DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
