/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <kinematic_constraints/utils.h>
#include <planning_scene_monitor/planning_scene_monitor.h>

static const std::string ROBOT_DESCRIPTION="robot_description";

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pr2_move_group_test", ros::init_options::AnonymousName);
  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION);
  planning_scene::PlanningScene &scene = *psm.getPlanningScene();
  
  actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> act("move_group", false);
  act.waitForServer();

  
  moveit_msgs::MoveGroupGoal goal;
  /*
  goal.request.group_name = "";
  goal.request.allowed_planning_time = ros::Duration(0.5);
  goal.request.goal_constraints.resize(1);  
  goal.request.goal_constraints[0].joint_constraints.resize(1);
  goal.request.goal_constraints[0].joint_constraints[0].position = -2.0;
  goal.request.goal_constraints[0].joint_constraints[0].joint_name = "r_shoulder_pan_joint";
  goal.request.goal_constraints[0].joint_constraints[0].position = 0.0;
  goal.request.goal_constraints[0].joint_constraints[0].tolerance_above = 0.001;
  goal.request.goal_constraints[0].joint_constraints[0].tolerance_below = 0.001;
  goal.request.goal_constraints[0].joint_constraints[0].weight = 1.0;
  */



  goal.request.group_name = "right_arm";
  goal.request.num_planning_attempts = 1;
  goal.request.allowed_planning_time = ros::Duration(5.0);
  
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "torso_lift_link";
  pose.pose.position.x = 0.75;
  pose.pose.position.y = -0.2;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;
  moveit_msgs::Constraints g0 = kinematic_constraints::constructGoalConstraints("r_wrist_roll_link", pose);
  
  /*
  pose.pose.position.x = 0.35;
  pose.pose.position.y = -0.6;
  pose.pose.position.z = 1.25;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;    
  moveit_msgs::Constraints g1 = kinematic_constraints::constructGoalConstraints("r_wrist_roll_link", pose);
  */  
  goal.request.goal_constraints.resize(1);
  goal.request.goal_constraints[0] = g0;

  //  goal.request.goal_constraints[0] = kinematic_constraints::mergeConstraints(g1, g0);

  planning_models::RobotState *start = scene.getCurrentState();

  for (int i = 0 ; i < 50 ; ++i){
  start.getJointStateGroup("right_arm")->setToRandomValues();
  goal.request.goal_constraints.resize(1);
  goal.request.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(start.getJointStateGroup("right_arm"));
  
  act.sendGoal(goal);
  if(!act.waitForResult(ros::Duration(5.0))) {
    ROS_INFO_STREAM("Apparently returned early");
  }
  if (act.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("It worked!");
  else
    ROS_WARN_STREAM("Fail: " << act.getState().toString() << ": " << act.getState().getText());
  std::cout << *act.getResult() << std::endl;
  sleep(5);
  }
  return 0;
}
