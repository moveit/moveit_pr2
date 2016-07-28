cmake_minimum_required(VERSION 2.8.3)
project(pr2_jacobian_tests)

find_package(catkin REQUIRED COMPONENTS kdl_parser)

catkin_package(
    DEPENDS kdl_parser
    CATKIN_DEPENDS kdl_parser
)
