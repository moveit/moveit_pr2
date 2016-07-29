cmake_minimum_required(VERSION 2.8.3)
project(pr2_jacobian_tests)

find_package(catkin REQUIRED COMPONENTS kdl_parser)

catkin_package(
    CATKIN_DEPENDS kdl_parser
)

install(FILES
    pr2.srdf 
    test_pr2_jacobian_solver.launch 
    upload_pr2.launch 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

