/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Sachin Chitta
 */

#include <pr2_arm_kinematics/pr2_arm_kinematics_plugin.h>
#include <geometry_msgs/PoseStamped.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>
#include "ros/ros.h"
#include <algorithm>
#include <numeric>

#include <pluginlib/class_list_macros.h>

using namespace KDL;
using namespace tf;
using namespace std;
using namespace ros;

//register PR2ArmKinematics as a KinematicsBase implementation
PLUGINLIB_DECLARE_CLASS(pr2_arm_kinematics,PR2ArmKinematicsPlugin, pr2_arm_kinematics::PR2ArmKinematicsPlugin, kinematics::KinematicsBase)

namespace pr2_arm_kinematics {

 PR2ArmKinematicsPlugin::PR2ArmKinematicsPlugin():active_(false){}

  bool PR2ArmKinematicsPlugin::isActive()
  {
    if(active_)
      return true;
    return false;
  }

  bool PR2ArmKinematicsPlugin::initialize(const std::string &name)
  {
    urdf::Model robot_model;
    std::string tip_name, xml_string;
    ros::NodeHandle private_handle("~/"+name);
    dimension_ = 7;
    while(!loadRobotModel(private_handle,robot_model,root_name_,tip_name,xml_string) && private_handle.ok())
    {
      ROS_ERROR("Could not load robot model. Are you sure the robot model is on the parameter server?");
      ros::Duration(0.5).sleep();
    }

    ROS_DEBUG("Loading KDL Tree");
    if(!getKDLChain(xml_string,root_name_,tip_name,kdl_chain_))
    {
      active_ = false;
      ROS_ERROR("Could not load kdl tree");
    }
    ROS_DEBUG("Advertising services");
    jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    private_handle.param<int>("free_angle",free_angle_,2);

    private_handle.param<double>("search_discretization",search_discretization_,0.01);
    pr2_arm_ik_solver_.reset(new pr2_arm_kinematics::PR2ArmIKSolver(robot_model,root_name_,tip_name, search_discretization_,free_angle_));
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
      ROS_DEBUG("PR2KinematicsPlugin::active for %s",name.c_str());
      active_ = true;
    }    
    return active_;
  }

  bool PR2ArmKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
					     std::vector<double> &solution,
					     int &error_code)
  {
    if(!active_)
    {
      ROS_ERROR("kinematics not active");
      error_code = kinematics::NO_IK_SOLUTION; 
      return false;
    }
    
    KDL::Frame pose_desired;
    tf::PoseMsgToKDL(ik_pose, pose_desired);

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
      error_code = kinematics::NO_IK_SOLUTION; 
      return false;
    }

    if(ik_valid >= 0)
    {
      solution.resize(dimension_);
      for(int i=0; i < dimension_; i++)
      {
        solution[i] = jnt_pos_out(i);
      }
      error_code = kinematics::SUCCESS;
      return true;
    }
    else
    {
      ROS_DEBUG("An IK solution could not be found");   
      error_code = kinematics::NO_IK_SOLUTION; 
      return false;
    }
  }

  bool PR2ArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                const std::vector<double> &ik_seed_state,
						double timeout,
                                                std::vector<double> &solution,
						int &error_code)
  {
    if(!active_)
    {
      ROS_ERROR("kinematics not active");
      error_code = kinematics::INACTIVE; 
      return false;
    }
    KDL::Frame pose_desired;
    tf::PoseMsgToKDL(ik_pose, pose_desired);

    //Do the IK
    KDL::JntArray jnt_pos_in;
    KDL::JntArray jnt_pos_out;
    jnt_pos_in.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
        jnt_pos_in(i) = ik_seed_state[i];
    }

    int ik_valid = pr2_arm_ik_solver_->CartToJntSearch(jnt_pos_in,
                                                       pose_desired,
                                                       jnt_pos_out,
                                                       timeout);

    if (ik_valid != pr2_arm_kinematics::NO_IK_SOLUTION && ik_valid >= 0)
    {
      solution.resize(dimension_);
      for(int i=0; i < dimension_; i++)
      {
        solution[i] = jnt_pos_out(i);
      }
      error_code = kinematics::SUCCESS;
      return true;
    }
    else
    {
      ROS_DEBUG("An IK solution could not be found");   
      error_code = kinematics::NO_IK_SOLUTION; 
      return false;
    }
  }

void PR2ArmKinematicsPlugin::desiredPoseCallback(const KDL::JntArray& jnt_array, 
                                                 const KDL::Frame& ik_pose,
                                                 moveit_msgs::MoveItErrorCodes& error_code)
{
  std::vector<double> ik_seed_state;
  ik_seed_state.resize(dimension_);
  int int_error_code;
  for(int i=0; i < dimension_; i++)
    ik_seed_state[i] = jnt_array(i);

  geometry_msgs::Pose ik_pose_msg;
  tf::PoseKDLToMsg(ik_pose,ik_pose_msg);

  desiredPoseCallback_(ik_pose_msg,ik_seed_state,int_error_code);
  if(int_error_code)
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  else
    error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;     
}


void PR2ArmKinematicsPlugin::jointSolutionCallback(const KDL::JntArray& jnt_array, 
                                                   const KDL::Frame& ik_pose,
                                                   moveit_msgs::MoveItErrorCodes& error_code)
{
  std::vector<double> ik_seed_state;
  ik_seed_state.resize(dimension_);
  int int_error_code;
  for(int i=0; i < dimension_; i++)
    ik_seed_state[i] = jnt_array(i);

  geometry_msgs::Pose ik_pose_msg;
  tf::PoseKDLToMsg(ik_pose,ik_pose_msg);

  solutionCallback_(ik_pose_msg,ik_seed_state,int_error_code);
  if(int_error_code > 0)
    error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  else
    error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;     
}

  bool PR2ArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                const std::vector<double> &ik_seed_state,
						double timeout,
                                                std::vector<double> &solution,
                                                const IKCallbackFn &desired_pose_callback,
                                                const IKCallbackFn &solution_callback,
						int &error_code_int)  
  {
    if(!active_)
    {
      ROS_ERROR("kinematics not active");
      error_code_int = kinematics::INACTIVE;
      return false;
    }
    KDL::Frame pose_desired;
    tf::PoseMsgToKDL(ik_pose, pose_desired);

    desiredPoseCallback_ = desired_pose_callback;
    solutionCallback_    = solution_callback;

    //Do the IK
    KDL::JntArray jnt_pos_in;
    KDL::JntArray jnt_pos_out;
    jnt_pos_in.resize(dimension_);
    for(int i=0; i < dimension_; i++)
    {
        jnt_pos_in(i) = ik_seed_state[i];
    }

    moveit_msgs::MoveItErrorCodes error_code;
    int ik_valid = pr2_arm_ik_solver_->CartToJntSearch(jnt_pos_in,
                                                       pose_desired,
                                                       jnt_pos_out,
                                                       timeout,
                                                       error_code,
                                                       boost::bind(&PR2ArmKinematicsPlugin::desiredPoseCallback, this, _1, _2, _3),
                                                       boost::bind(&PR2ArmKinematicsPlugin::jointSolutionCallback, this, _1, _2, _3));
    if(ik_valid == pr2_arm_kinematics::NO_IK_SOLUTION)
       return false;

    if(ik_valid >= 0)
    {
      solution.resize(dimension_);
      for(int i=0; i < dimension_; i++)
      {
        solution[i] = jnt_pos_out(i);
      }
      error_code_int = kinematics::SUCCESS;
      return true;
    }
    else
    {
      ROS_DEBUG("An IK solution could not be found");   
      error_code_int = error_code.val;
      return false;
    }
  }

  bool PR2ArmKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                             const std::vector<double> &joint_angles,
                                             std::vector<geometry_msgs::Pose> &poses)
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
    }

    poses.resize(link_names.size());

    bool valid = true;
    for(unsigned int i=0; i < poses.size(); i++)
    {
      ROS_DEBUG("End effector index: %d",pr2_arm_kinematics::getKDLSegmentIndex(kdl_chain_,link_names[i]));
      if(jnt_to_pose_solver_->JntToCart(jnt_pos_in,p_out,pr2_arm_kinematics::getKDLSegmentIndex(kdl_chain_,link_names[i])) >=0)
      {
        tf::PoseKDLToMsg(p_out,poses[i]);
      }
      else
      {
        ROS_ERROR("Could not compute FK for %s",link_names[i].c_str());
        valid = false;
      }
    }
    return valid;
  }

  std::string PR2ArmKinematicsPlugin::getBaseFrame()
  {
    if(!active_)
    {
      ROS_ERROR("kinematics not active");
      return std::string("");
    }
    return root_name_;
  }

  std::string PR2ArmKinematicsPlugin::getToolFrame()
  {
    if(!active_ || ik_solver_info_.link_names.empty())
    {
      ROS_ERROR("kinematics not active");
      return std::string("");
    }
    return ik_solver_info_.link_names[0];
  }

  std::vector<std::string> PR2ArmKinematicsPlugin::getJointNames()
  {
    if(!active_)
    {
      std::vector<std::string> empty;
      ROS_ERROR("kinematics not active");
      return empty;
    }
    return ik_solver_info_.joint_names;
  }

  std::vector<std::string> PR2ArmKinematicsPlugin::getLinkNames()
  {
    if(!active_)
    {
      std::vector<std::string> empty;
      ROS_ERROR("kinematics not active");
      return empty;
    }
    return fk_solver_info_.link_names;
  }

} // namespace
