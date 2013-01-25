/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

#include <moveit/pr2_arm_kinematics/pr2_arm_kinematics_plugin.h>
#include <geometry_msgs/PoseStamped.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>
#include <ros/ros.h>
#include <algorithm>
#include <numeric>

#include <pluginlib/class_list_macros.h>

using namespace KDL;
using namespace tf;
using namespace std;
using namespace ros;

//register PR2ArmKinematics as a KinematicsBase implementation
PLUGINLIB_EXPORT_CLASS(pr2_arm_kinematics::PR2ArmKinematicsPlugin, kinematics::KinematicsBase);

namespace pr2_arm_kinematics {

PR2ArmKinematicsPlugin::PR2ArmKinematicsPlugin():active_(false){}

bool PR2ArmKinematicsPlugin::isActive()
{
  if(active_)
    return true;
  return false;
}

bool PR2ArmKinematicsPlugin::initialize(const std::string& robot_description,
                                        const std::string& group_name,
                                        const std::string& base_name,
                                        const std::string& tip_name,
                                        double search_discretization)
{
  setValues(robot_description, group_name, base_name, tip_name,search_discretization);
  urdf::Model robot_model;
  std::string xml_string;
  ros::NodeHandle private_handle("~/"+group_name);
  dimension_ = 7;
  while(!loadRobotModel(private_handle,robot_model,xml_string) && private_handle.ok())
  {
    ROS_ERROR("Could not load robot model. Are you sure the robot model is on the parameter server?");
    ros::Duration(0.5).sleep();
  }

  ROS_DEBUG("Loading KDL Tree");
  if(!getKDLChain(xml_string,base_frame_,tip_frame_,kdl_chain_))
  {
    active_ = false;
    ROS_ERROR("Could not load kdl tree");
  }
  ROS_DEBUG("Advertising services");
  jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  private_handle.param<int>("free_angle",free_angle_,2);

  pr2_arm_ik_solver_.reset(new pr2_arm_kinematics::PR2ArmIKSolver(robot_model, base_frame_,tip_frame_, search_discretization_,free_angle_));
  if(!pr2_arm_ik_solver_->active_)
  {
    ROS_ERROR("Could not load ik");
    active_ = false;
  }
  else
  {

    pr2_arm_ik_solver_->getSolverInfo(ik_solver_info_);
    pr2_arm_kinematics::getKDLChainInfo(kdl_chain_,fk_solver_info_);
    fk_solver_info_.joint_names = ik_solver_info_.joint_names;

    for(unsigned int i=0; i < ik_solver_info_.joint_names.size(); i++)
    {
      ROS_DEBUG("PR2Kinematics:: joint name: %s",ik_solver_info_.joint_names[i].c_str());
    }
    for(unsigned int i=0; i < ik_solver_info_.link_names.size(); i++)
    {
      ROS_DEBUG("PR2Kinematics can solve IK for %s",ik_solver_info_.link_names[i].c_str());
    }
    for(unsigned int i=0; i < fk_solver_info_.link_names.size(); i++)
    {
      ROS_DEBUG("PR2Kinematics can solve FK for %s",fk_solver_info_.link_names[i].c_str());
    }
    ROS_DEBUG("PR2KinematicsPlugin::active for %s",group_name.c_str());
    active_ = true;
  }    
  pr2_arm_ik_solver_->setFreeAngle(2);
  return active_;
}

bool PR2ArmKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           std::vector<double> &solution,
                                           moveit_msgs::MoveItErrorCodes &error_code) const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code.val = error_code.NO_IK_SOLUTION; 
    return false;
  }
    
  KDL::Frame pose_desired;
  tf::poseMsgToKDL(ik_pose, pose_desired);

  //Do the IK
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;
  jnt_pos_in.resize(dimension_);
  for(int i=0; i < dimension_; i++)
  {
    jnt_pos_in(i) = ik_seed_state[i];
  }

  int ik_valid = pr2_arm_ik_solver_->CartToJnt(jnt_pos_in,
                                               pose_desired,
                                               jnt_pos_out);
  if(ik_valid == pr2_arm_kinematics::NO_IK_SOLUTION)
  {
    error_code.val = error_code.NO_IK_SOLUTION; 
    return false;
  }

  if(ik_valid >= 0)
  {
    solution.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
      solution[i] = jnt_pos_out(i);
    }
    error_code.val = error_code.SUCCESS;
    return true;
  }
  else
  {
    ROS_DEBUG("An IK solution could not be found");   
    error_code.val = error_code.NO_IK_SOLUTION; 
    return false;
  }
}

bool PR2ArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              std::vector<double> &solution,
                                              moveit_msgs::MoveItErrorCodes &error_code) const
{
  static IKCallbackFn solution_callback = 0;  
  static std::vector<double> consistency_limits;  
  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code);   
}

bool PR2ArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              const std::vector<double> &consistency_limits,
                                              std::vector<double> &solution,
                                              moveit_msgs::MoveItErrorCodes &error_code) const
{
  static IKCallbackFn solution_callback = 0;  
  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code);   
}

bool PR2ArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              std::vector<double> &solution,
                                              const IKCallbackFn &solution_callback,
                                              moveit_msgs::MoveItErrorCodes &error_code) const
{
  static std::vector<double> consistency_limits;  
  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code);  
}

bool PR2ArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              double timeout,
                                              const std::vector<double> &consistency_limits,
                                              std::vector<double> &solution,
                                              const IKCallbackFn &solution_callback,
                                              moveit_msgs::MoveItErrorCodes &error_code) const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code.val = error_code.FAILURE;
    return false;
  }
  if(!consistency_limits.empty() && consistency_limits.size() != dimension_)
  {
    ROS_ERROR("Consistency limits should be of size: %d",dimension_);    
    error_code.val = error_code.FAILURE; 
    return false;
  }

  KDL::Frame pose_desired;
  tf::poseMsgToKDL(ik_pose, pose_desired);

  //Do the IK
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;
  jnt_pos_in.resize(dimension_);
  for(int i=0; i < dimension_; i++)
  {
    jnt_pos_in(i) = ik_seed_state[i];
  }

  int ik_valid;
  if(consistency_limits.empty())
  {
    ik_valid = pr2_arm_ik_solver_->CartToJntSearch(jnt_pos_in,
                                                   pose_desired,
                                                   jnt_pos_out,
                                                   timeout,
                                                   error_code,
                                                   solution_callback ? 
                                                   boost::bind(solution_callback, _1, _2, _3):
                                                   IKCallbackFn());      
  }
  else
  {
    ik_valid = pr2_arm_ik_solver_->CartToJntSearch(jnt_pos_in,
                                                   pose_desired,
                                                   jnt_pos_out,
                                                   timeout,
                                                   consistency_limits[free_angle_],
                                                   error_code,
                                                   solution_callback ? 
                                                   boost::bind(solution_callback, _1, _2, _3):
                                                   IKCallbackFn());    
  }

  if(ik_valid == pr2_arm_kinematics::NO_IK_SOLUTION)
    return false;

  if(ik_valid >= 0)
  {
    solution.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
      solution[i] = jnt_pos_out(i);
    }
    return true;
  }
  else
  {
    ROS_DEBUG("An IK solution could not be found");   
    return false;
  }
}

bool PR2ArmKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                           const std::vector<double> &joint_angles,
                                           std::vector<geometry_msgs::Pose> &poses) const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
    return false;
  }

  KDL::Frame p_out;
  KDL::JntArray jnt_pos_in;
  geometry_msgs::PoseStamped pose;
  tf::Stamped<tf::Pose> tf_pose;

  jnt_pos_in.resize(dimension_);
  for(int i=0; i < dimension_; i++)
  {
    jnt_pos_in(i) = joint_angles[i];
    //    ROS_DEBUG("Joint angle: %d %f",i,joint_angles[i]);
  }

  poses.resize(link_names.size());

  bool valid = true;
  for(unsigned int i=0; i < poses.size(); i++)
  {
    //    ROS_DEBUG("End effector index: %d",pr2_arm_kinematics::getKDLSegmentIndex(kdl_chain_,link_names[i]));
    if(jnt_to_pose_solver_->JntToCart(jnt_pos_in,p_out,pr2_arm_kinematics::getKDLSegmentIndex(kdl_chain_,link_names[i])) >= 0)
    {
      tf::poseKDLToMsg(p_out,poses[i]);
    }
    else
    {
      ROS_ERROR("Could not compute FK for %s",link_names[i].c_str());
      valid = false;
    }
  }
  return valid;
}

const std::vector<std::string>& PR2ArmKinematicsPlugin::getJointNames() const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
  }
  return ik_solver_info_.joint_names;
}

const std::vector<std::string>& PR2ArmKinematicsPlugin::getLinkNames() const
{
  if(!active_)
  {
    ROS_ERROR("kinematics not active");
  }
  return fk_solver_info_.link_names;
}

} // namespace
