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
#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>

// MoveIt!
#include <moveit/kinematics_constraint_aware/kinematics_constraint_aware.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/joint_state_group.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <eigen_conversions/eigen_msg.h>
#include <urdf/model.h>
#include <srdfdom/model.h>

TEST(ConstraintAwareKinematics, getIK)
{
  std::string group_name = "right_arm";  
  std::string ik_link_name = "r_wrist_roll_link";  

  ROS_INFO("Initializing IK solver");      
  planning_scene::PlanningScenePtr planning_scene;  
  robot_model_loader::RDFLoader robot_model_loader("robot_description"); /** Used to load the robot model */  
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  
  const boost::shared_ptr<srdf::Model> &srdf = robot_model_loader.getSRDF();
  const boost::shared_ptr<urdf::ModelInterface>& urdf_model = robot_model_loader.getURDF();

  planning_scene.reset(new planning_scene::PlanningScene(kinematic_model));

  const robot_model::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(group_name);

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  robot_state::JointStateGroup* joint_state_group = kinematic_state->getJointStateGroup(group_name);
  kinematic_state->setToDefaultValues();  

  kinematics_constraint_aware::KinematicsConstraintAware solver(kinematic_model, "right_arm");  

  ros::NodeHandle nh("~");
  int number_ik_tests;  
  nh.param("number_ik_tests", number_ik_tests, 1);  

  int acceptable_success_percentage;
  nh.param("accepatable_success_percentage", acceptable_success_percentage, 95);  
  
  unsigned int num_success = 0;

  kinematics_constraint_aware::KinematicsRequest kinematics_request;
  kinematics_constraint_aware::KinematicsResponse kinematics_response;
  kinematics_response.solution_.reset(new robot_state::RobotState(planning_scene->getCurrentState()));
  
  kinematics_request.group_name_ = group_name;
  kinematics_request.timeout_ = ros::Duration(5.0);
  kinematics_request.check_for_collisions_ = false;
  kinematics_request.robot_state_ = kinematic_state;

  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = kinematic_model->getModelFrame();  

  for(std::size_t i = 0; i < (unsigned int) number_ik_tests; ++i)
  {
    joint_state_group->setToRandomValues();
    const Eigen::Affine3d &end_effector_state = joint_state_group->getRobotState()->getLinkState(ik_link_name)->getGlobalLinkTransform();    
    Eigen::Quaterniond quat(end_effector_state.rotation());
    Eigen::Vector3d point(end_effector_state.translation());
    goal.pose.position.x = point.x();
    goal.pose.position.y = point.y();
    goal.pose.position.z = point.z();
    goal.pose.orientation.x = quat.x();
    goal.pose.orientation.y = quat.y();
    goal.pose.orientation.z = quat.z();
    goal.pose.orientation.w = quat.w();

    joint_state_group->setToRandomValues();
    kinematics_request.pose_stamped_vector_.clear();
    kinematics_request.pose_stamped_vector_.push_back(goal);    
    ros::WallTime start = ros::WallTime::now();    
    if(solver.getIK(planning_scene, kinematics_request, kinematics_response))
      num_success++;    
    else
      printf("Failed in %f\n", (ros::WallTime::now()-start).toSec());   
  }  
  bool test_success = (((double)num_success)/number_ik_tests > acceptable_success_percentage/100.0);
  printf("success ratio: %d of %d", num_success, number_ik_tests);  
  EXPECT_TRUE(test_success);  
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init (argc, argv, "right_arm_kinematics");
  return RUN_ALL_TESTS();
}
