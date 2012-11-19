set(MOVEIT_LIB_NAME pr2_moveit_constraint_sampler)

add_library(${MOVEIT_LIB_NAME} src/pr2_constraint_sampler.cpp)
target_link_libaries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
