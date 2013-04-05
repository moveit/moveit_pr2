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

  /* SETUP THE PLANNER*/
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::Planner> > planner_plugin_loader;
  planning_interface::PlannerPtr planner_instance;
  std::string planner_plugin_name;

  /* Get the name of the planner we want to use */
  if (!node_handle.getParam("planning_plugin", planner_plugin_name))
    ROS_FATAL_STREAM("Could not find planner plugin name");  

  /* Make sure to catch all exceptions */
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::Planner>("moveit_core", "planning_interface::Planner"));
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model))
      ROS_FATAL_STREAM("Could not initialize planner instance");    
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch(pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0 ; i < classes.size() ; ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                     << "Available plugins: " << ss.str());
  }

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

  /* CALL THE PLANNER */
  planner_instance->solve(planning_scene, req, res);

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
  
  /* Call the Planner */
  planner_instance->solve(planning_scene, req, res);

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

  /* Now, let's try to go back to the first goal*/
  /* First, set the state in the planning scene to the final state of the last plan */
  joint_state_group->setVariableValues(response.trajectory.joint_trajectory.points.back().positions);  

  /* Now, we go back to the first goal*/
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal);
  planner_instance->solve(planning_scene, req, res);
  res.getMessage(response);  
  display_trajectory.trajectory.push_back(response.trajectory);  
  display_publisher.publish(display_trajectory);
  
  /* Let's create a new pose goal */
  pose.pose.position.x = 0.65;
  pose.pose.position.y = -0.2;
  pose.pose.position.z = -0.1;  
  moveit_msgs::Constraints pose_goal_2 = kinematic_constraints::constructGoalConstraints("r_wrist_roll_link", pose, tolerance_pose, tolerance_angle);  

  /* First, set the state in the planning scene to the final state of the last plan */
  joint_state_group->setVariableValues(response.trajectory.joint_trajectory.points.back().positions);  

  /* Now, let's try to move to this new pose goal*/
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal_2);

  /* But, let's impose a path constraint on the motion. 
     Here, we are asking for the end-effector to stay level*/
  geometry_msgs::QuaternionStamped quaternion;
  quaternion.header.frame_id = "torso_lift_link";
  quaternion.quaternion.w = 1.0;
  
  req.path_constraints = kinematic_constraints::constructGoalConstraints("r_wrist_roll_link", quaternion);

  // imposing path constraints requires the planner to reason in the space of possible positions of the end-effector
  // (the workspace of the robot)
  // because of this, we need to specify a bound for the allowed planning volume as well;
  // Note: a default bound is automatically filled by the WorkspaceBounds request adapter (part of the OMPL pipeline,
  // but that is not being used in this example).
  // We use a bound that definitely includes the reachable space for the arm. This is fine because sampling is not done in this volume 
  // when planning for the arm; the bounds are only used to determine if the sampled configurations are valid.
  req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y = req.workspace_parameters.min_corner.z = -2.0;
  req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y = req.workspace_parameters.max_corner.z =  2.0;
  

  planner_instance->solve(planning_scene, req, res);
  res.getMessage(response);  
  display_trajectory.trajectory.push_back(response.trajectory);  
  //Now you should see four planned trajectories in series
  display_publisher.publish(display_trajectory);

  sleep_time.sleep();
  ROS_INFO("Done");  
  planner_instance.reset();
  
  return 0;  
}
