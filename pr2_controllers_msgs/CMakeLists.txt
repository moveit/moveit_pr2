cmake_minimum_required(VERSION 2.8.3)
project(pr2_controllers_msgs)

find_package(catkin REQUIRED actionlib_msgs geometry_msgs message_generation std_msgs trajectory_msgs)

add_action_files(DIRECTORY action FILES
  PointHead.action
  Pr2GripperCommand.action
  )

add_message_files(DIRECTORY msg FILES
  Pr2GripperCommand.msg)

generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs std_msgs trajectory_msgs)

catkin_package(CATKIN_DEPENDS actionlib_msgs geometry_msgs message_runtime std_msgs trajectory_msgs)
