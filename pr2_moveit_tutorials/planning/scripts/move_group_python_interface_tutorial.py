#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley

import sys
import rospy
import moveit_commander
#from moveit_commander import RobotCommander, roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import RobotState

def move_group_python_interface_tutorial():
  # BEGIN_TUTORIAL
  #
  # Setup
  # ^^^^^
  #
  # First initialize moveit_commander and rospy.
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

  # This sleep is ONLY to allow Rviz to come up
  print "============ Waiting for RVIZ..."
  rospy.sleep(20)
  print "============ Starting tutorial "

  # Instantiate a RobotCommander object.  This object is an interface to
  # the robot as a whole.
  robot = moveit_commander.RobotCommander()

  # Instantiate a PlanningSceneInterface object.  This object is an interface
  # to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  # Instantiate a MoveGroupCommander object.  This object is an interface
  # to one group of joints.  In this case the group is the joints in the right
  # arm.  This interface can be used to plan and execute motions on the right
  # arm.
  group = moveit_commander.MoveGroupCommander("left_arm")


  # Getting Basic Information
  # ^^^^^^^^^^^^^^^^^^^^^^^^^
  #
  # We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % group.get_planning_frame()

  # We can also print the name of the end-effector link for this group
  print "============ Reference frame: %s" % group.get_end_effector_link()

  # We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()

  # Sometimes for debugging it is useful to print the entire state of the
  # robot.
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"


  # Planning to a Pose goal
  # ^^^^^^^^^^^^^^^^^^^^^^^
  # We can plan a motion for this group to a desired pose for the 
  # end-effector

    








  # When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

  # END_TUTORIAL

  print "============ STOPPING"


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass

