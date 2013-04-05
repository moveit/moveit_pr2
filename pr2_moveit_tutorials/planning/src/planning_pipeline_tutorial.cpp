/*********************************************************************
*
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* Author: Sachin Chitta
*********************************************************************/

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
 
int main(int argc, char **argv)
{
  ros::init (argc, argv, "move_group_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");  

  /* SETUP A PLANNING SCENE*/
  /* Load the robot model */
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

  /* Get a shared pointer to the model */
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

  /* Construct a planning scene - NOTE: this is for illustration purposes only.
     The recommended way to construct a planning scene is to use the planning_scene_monitor 
     to construct it for you.*/
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

  /* SETUP THE PLANNING PIPELINE*/  
  planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(robot_model, "planning_plugin", "request_adapters"));  

  /* Sleep a little to allow time to startup rviz, etc. */
  ros::WallDuration sleep_time(8.0);
  sleep_time.sleep();

  /* CREATE A MOTION PLAN REQUEST FOR THE RIGHT ARM OF THE PR2 */
  /* We will ask the end-effector of the PR2 to go to a desired location */
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;

  /* A desired pose */
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "torso_lift_link";  
  pose.pose.position.x = 0.75;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.0;  
  pose.pose.orientation.w = 1.0;
  
  /* A desired tolerance */
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);
  
  /*Create the request */
  req.group_name = "right_arm";
  moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("r_wrist_roll_link", pose, tolerance_pose, tolerance_angle);  
  req.goal_constraints.push_back(pose_goal);     

  /* Call the pipeline */
  //  planning_pipeline->generatePlan((planning_scene::PlanningSceneConstPtr) planning_scene, req, res);
  planning_pipeline->generatePlan(planning_scene, req, res);

  /* Check that the planning was successful */
  if(res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  
  /* Visualize the generated plan */
  /* Publisher for display */  
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);  
  moveit_msgs::DisplayTrajectory display_trajectory;  

  /* Visualize the trajectory */
  ROS_INFO("Visualizing the trajectory");  
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);  

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);  
  display_publisher.publish(display_trajectory);

  sleep_time.sleep();

  /* NOW TRY A JOINT SPACE GOAL */
  /* First, set the state in the planning scene to the final state of the last plan */
  robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
  planning_scene->setCurrentState(response.trajectory_start);
  robot_state::JointStateGroup* joint_state_group = robot_state.getJointStateGroup("right_arm");  
  joint_state_group->setVariableValues(response.trajectory.joint_trajectory.points.back().positions);  

  /* Now, setup a joint space goal*/
  robot_state::RobotState goal_state(robot_model);
  robot_state::JointStateGroup* goal_group = goal_state.getJointStateGroup("right_arm");  
  std::vector<double> joint_values(7, 0.0);  
  joint_values[0] = -2.0;
  joint_values[3] = -0.2;
  joint_values[5] = -0.15;  
  goal_group->setVariableValues(joint_values);  
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_group);
  
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);
  
  /* Call the pipeline */
  planning_pipeline->generatePlan(planning_scene, req, res);

  /* Check that the planning was successful */
  if(res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  
  /* Visualize the trajectory */
  ROS_INFO("Visualizing the trajectory");  
  res.getMessage(response);  

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);  
  //Now you should see two planned trajectories in series
  display_publisher.publish(display_trajectory);

  sleep_time.sleep();


  /* PRE-PROCESSING THE MOTION PLAN REQUEST */
  /* Let's purposefully set the initial state to be outside the joint limits and let the 
     planning request adapter deal with it */
  /* First, set the state in the planning scene to the final state of the last plan */
  robot_state = planning_scene->getCurrentStateNonConst();
  planning_scene->setCurrentState(response.trajectory_start);
  joint_state_group = robot_state.getJointStateGroup("right_arm");  
  joint_state_group->setVariableValues(response.trajectory.joint_trajectory.points.back().positions);  

  //Now, set one of the joints slightly outside its upper limit
  robot_state::JointState* joint_state = joint_state_group->getJointState("r_shoulder_pan_joint");
  const std::vector<std::pair<double, double> >& joint_bounds = joint_state->getVariableBounds();
  std::vector<double> tmp_values(1, 0.0);
  tmp_values[0] = joint_bounds[0].first - 0.01;
  joint_state->setVariableValues(tmp_values);  
  
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal);
  planning_pipeline->generatePlan(planning_scene, req, res);
  if(res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }  
  /* Visualize the trajectory */
  ROS_INFO("Visualizing the trajectory");  
  res.getMessage(response);  
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);  
  //Now you should see three planned trajectories in series
  display_publisher.publish(display_trajectory);
 
  sleep_time.sleep();
  ROS_INFO("Done");  
  return 0;  
}
