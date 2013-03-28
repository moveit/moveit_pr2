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

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>

bool userCallback(const robot_state::RobotState &kinematic_state, bool verbose)
{
  // get the joint value for the right shoulder pan of the PR2 robot
  const std::vector<double>& joint_state_values = kinematic_state.getJointState("r_shoulder_pan_joint")->getVariableValues();
  return (joint_state_values.front() > 0.0);
}
 
int main(int argc, char **argv)
{
  ros::init (argc, argv, "right_arm_kinematics");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  /* Load the robot model */
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

  /* Get a shared pointer to the model */
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

  /* Construct a planning scene - NOTE: this is for illustration purposes only.
     The recommended way to construct a planning scene is to use the planning_scene_monitor 
     to construct it for you.*/
  planning_scene::PlanningScene planning_scene(kinematic_model);

  /* The planning scene contains a RobotState *representation of the robot configuration
   We can get a reference to it.*/
  robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();

  /* COLLISION CHECKING */
  /* Let's check if the current state is in self-collision. All self-collision checks use an unpadded version of the 
     robot collision model, i.e. no extra padding is applied to the robot.*/
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 1 : Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");  

  /* Let's change the current state that the planning scene has and check if that is in self-collision*/
  current_state.setToRandomValues();
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 2 : Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");  
  
  /* Now, we will do collision checking only for the right_arm of the PR2, i.e. we will check whether 
   there are any collisions between the right arm and other parts of the body of the robot.*/
  collision_request.group_name = "right_arm";  
  current_state.setToRandomValues();
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 3: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");  

  /* We will first manually set the right arm to a position where we know internal (self) collisions 
     do happen.*/
  std::vector<double> joint_values;  
  robot_state::JointStateGroup* joint_state_group = current_state.getJointStateGroup("right_arm");
  joint_state_group->getVariableValues(joint_values);
  joint_values[0] = 1.57; //hard-coded since we know collisions will happen here
  joint_state_group->setVariableValues(joint_values);
  /* Note that this state is now actually outside the joint limits of the PR2, which we can also check for
     directly.*/
  ROS_INFO_STREAM("Current state is " << (current_state.satisfiesBounds() ? "valid" : "not valid"));  
  
  /* Now, we will try and get contact information for any collisions that 
     might have happened at a given configuration of the right arm. */
  collision_request.contacts = true;
  collision_request.max_contacts = 1000;

  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 4: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");  
  for(collision_detection::CollisionResult::ContactMap::const_iterator it = collision_result.contacts.begin(); 
      it != collision_result.contacts.end(); 
      ++it)
  {
    ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());    
  }

  /* We will pass in a changed collision matrix to allow the particular set of contacts that just happened*/
  collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();  
  robot_state::RobotState copied_state = planning_scene.getCurrentState();  
  
  for(collision_detection::CollisionResult::ContactMap::const_iterator it = collision_result.contacts.begin(); 
      it != collision_result.contacts.end(); 
      ++it)
  {
    acm.setEntry(it->first.first, it->first.second, true);    
  }
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result, copied_state, acm);
  ROS_INFO_STREAM("Test 5: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");  

  /* While we have been checking for self-collisions, we can use the checkCollision functions instead which will 
     check for both self-collisions and for collisions with the environment (which is currently empty). 
     This is the set of collision checking functions that you will use most often in a planner. Note that collision checks 
     with the environment will use the padded version of the robot. Padding helps in keeping the robot further away from 
     obstacles in the environment.*/
  collision_result.clear();
  planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);
  ROS_INFO_STREAM("Test 6: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");  


  /* CONSTRAINT CHECKING*/
  /* Let's first create a constraint for the end-effector of the right arm of the PR2 */
  const robot_model::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("right_arm");
  std::string end_effector_name = joint_model_group->getLinkModelNames().back();
  
  geometry_msgs::PoseStamped desired_pose;
  desired_pose.pose.orientation.w = 1.0;
  desired_pose.pose.position.x = 0.75;
  desired_pose.pose.position.y = -0.185;
  desired_pose.pose.position.z = 1.3;
  desired_pose.header.frame_id = "base_footprint";
  
  moveit_msgs::Constraints goal_constraint = kinematic_constraints::constructGoalConstraints(end_effector_name, desired_pose);
  
  /* Now, we can directly check it for a state using the PlanningScene class*/
  copied_state.setToRandomValues();  
  bool state_constrained = planning_scene.isStateConstrained(copied_state, goal_constraint);
  ROS_INFO_STREAM("Test 7: Random state is " << (state_constrained ? "constrained" : "not constrained"));

  /* There's a more efficient way of checking constraints (when you want to check the same constraint over 
     and over again, e.g. inside a planner). We first construct a KinematicConstraintSet which pre-processes 
     the ROS Constraints messages and sets it up for quick processing. */
  kinematic_constraints::KinematicConstraintSet kinematic_constraint_set(kinematic_model, planning_scene.getTransforms());
  kinematic_constraint_set.add(goal_constraint);
  
  bool state_constrained_2 = planning_scene.isStateConstrained(copied_state, kinematic_constraint_set);
  ROS_INFO_STREAM("Test 8: Random state is " << (state_constrained_2 ? "constrained" : "not constrained"));
  
  /* There's an even more efficient way to do this using the KinematicConstraintSet class directly.*/
  kinematic_constraints::ConstraintEvaluationResult constraint_eval_result = kinematic_constraint_set.decide(copied_state);
  ROS_INFO_STREAM("Test 9: Random state is " << (constraint_eval_result.satisfied ? "constrained" : "not constrained"));

  /* Now, lets try a user-defined callback. This is done by specifying the callback using the 
     setStateFeasibilityPredicate function to which you pass the desired callback. 
     Now, whenever anyone calls isStateFeasible, the callback will be checked.*/
  planning_scene.setStateFeasibilityPredicate(userCallback);  
  bool state_feasible = planning_scene.isStateFeasible(copied_state);  
  ROS_INFO_STREAM("Test 10: Random state is " << (state_feasible ? "feasible" : "not feasible"));

  /* Whenever anyone calls isStateValid, three checks are conducted: (a) collision checking (b) constraint checking and 
     (c) feasibility checking using the user-defined callback */
  bool state_valid = planning_scene.isStateValid(copied_state, kinematic_constraint_set, "right_arm");  
  ROS_INFO_STREAM("Test 10: Random state is " << (state_valid ? "valid" : "not valid"));

  ros::shutdown(); 
  return 0;
}
