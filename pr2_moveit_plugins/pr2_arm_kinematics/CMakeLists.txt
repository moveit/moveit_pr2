set(MOVEIT_LIB_NAME pr2_moveit_arm_kinematics)

add_library(${MOVEIT_LIB_NAME} 
  src/pr2_arm_ik.cpp
  src/pr2_arm_kinematics_utils.cpp
  src/pr2_arm_ik_solver.cpp
  src/pr2_arm_kinematics_plugin.cpp
  )

target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
