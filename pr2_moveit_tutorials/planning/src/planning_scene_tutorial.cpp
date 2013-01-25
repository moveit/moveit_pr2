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
#include <moveit/planning_models_loader/kinematic_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
 
int main(int argc, char **argv)
{
  ros::init (argc, argv, "right_arm_kinematics");
  ros::AsyncSpinner spinner(1);
  spinner.start();
 
  /* Load the robot model */
  planning_models_loader::KinematicModelLoader kinematic_model_loader("robot_description");

  /* Get a shared pointer to the model */
  kinematic_model::KinematicModelPtr kinematic_model = kinematic_model_loader.getModel();

  /* Construct a planning scene - NOTE: this is for illustration purposes only
     The recommended way to construct a planning scene is to use the planning_scene_monitor 
     to construct it for you.*/
  planning_scene::PlanningScene planning_scene;
  planning_scene.configure(kinematic_model);

  /* The planning scene contains a KinematicState representation of the robot configuration
   We can get a reference to it.*/
  kinematic_state::KinematicState& current_state = planning_scene.getCurrentState();

  /* COLLISION CHECKING */
  /* Let's check if the current state is in self-collision. All self-collision checks use an unpadded version of the 
     robot collision model, i.e. no extra padding is applied to the robot.*/
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_response;
  planning_scene.checkSelfCollision(collision_request, collision_response);
  ROS_INFO_STREAM("Test 1 : Current state is " << (collision_response.collision ? "in" : "not in") << " self collision");  

  /* Let's change the current state that the planning scene has and check if that is in self-collision*/
  current_state.setToRandomValues();
  collision_response.clear();
  planning_scene.checkSelfCollision(collision_request, collision_response);
  ROS_INFO_STREAM("Test 2 : Current state is " << (collision_response.collision ? "in" : "not in") << " self collision");  
  
  /* Now, we will do collision checking only for the right_arm of the PR2, i.e. we will check whether 
   there are any collisions between the right arm and other parts of the body of the robot.*/
  collision_request.group_name = "right_arm";  
  current_state.setToRandomValues();
  collision_response.clear();
  planning_scene.checkSelfCollision(collision_request, collision_response);
  ROS_INFO_STREAM("Test 3: Current state is " << (collision_response.collision ? "in" : "not in") << " self collision");  

  /* We will first manually set the right arm to a position where we know internal (self) collisions 
     do happen.*/
  std::vector<double> joint_values;  
  kinematic_state::JointStateGroup* joint_state_group = current_state.getJointStateGroup("right_arm");
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

  collision_response.clear();
  planning_scene.checkSelfCollision(collision_request, collision_response);
  ROS_INFO_STREAM("Test 4: Current state is " << (collision_response.collision ? "in" : "not in") << " self collision");  
  for(collision_detection::CollisionResult::ContactMap::const_iterator it = collision_response.contacts.begin(); 
      it != collision_response.contacts.end(); 
      ++it)
  {
    ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());    
  }

  /* We will pass in a changed collision matrix to allow the particular set of contacts that just happened*/
  collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();  
  kinematic_state::KinematicState copied_state = planning_scene.getCurrentState();  
  
  for(collision_detection::CollisionResult::ContactMap::const_iterator it = collision_response.contacts.begin(); 
      it != collision_response.contacts.end(); 
      ++it)
  {
    acm.setEntry(it->first.first, it->first.second, true);    
  }
  collision_response.clear();
  planning_scene.checkSelfCollision(collision_request, collision_response, copied_state, acm);
  ROS_INFO_STREAM("Test 5: Current state is " << (collision_response.collision ? "in" : "not in") << " self collision");  

  /* While we have been checking for self-collisions, we can use the checkCollision functions instead which will 
     check for both self-collisions and for collisions with the environment (which is currently empty). 
     This is the set of collision checking functions that you will use most often in a planner. Note that collision checks 
     with the environment will use the padded version of the robot. Padding helps in keeping the robot further away from 
     obstacles in the environment. */
  collision_response.clear();
  planning_scene.checkCollision(collision_request, collision_response, copied_state, acm);
  ROS_INFO_STREAM("Test 6: Current state is " << (collision_response.collision ? "in" : "not in") << " self collision");  

  ros::shutdown(); 
  return 0;
}
