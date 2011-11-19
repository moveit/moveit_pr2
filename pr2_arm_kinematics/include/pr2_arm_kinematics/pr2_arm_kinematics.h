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
*
* Author: Sachin Chitta
*********************************************************************/

#ifndef PR2_ARM_IK_NODE_H
#define PR2_ARM_IK_NODE_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <angles/angles.h>
#include <pr2_arm_kinematics/pr2_arm_ik_solver.h>
#include <tf_conversions/tf_kdl.h>

#include <kinematics_msgs/GetPositionFK.h>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetKinematicSolverInfo.h>
#include <moveit_msgs/MoveItErrorCodes.h>

#include <kdl/chainfksolverpos_recursive.hpp>

#include <boost/shared_ptr.hpp>

namespace pr2_arm_kinematics
{
  class PR2ArmKinematics
  {
    public:

    /** @class
     *  @brief ROS/KDL based interface for the inverse kinematics of the PR2 arm
     *  @author Sachin Chitta <sachinc@willowgarage.com>
     *
     *  This class provides a ROS/KDL based interface to the inverse kinematics of the PR2 arm. It inherits from the KDL::ChainIkSolverPos class
     *  but also exposes additional functionality to return the multiple solutions from an inverse kinematics computation. It uses an instance of
     *  a ros::NodeHandle to find the robot description. It can thus be used only if the robot description is available on a ROS param server.
     *
     *  To use this wrapper, you must have a roscore running with a robot description available from the ROS param server. 
     */
    PR2ArmKinematics(bool create_transform_listener = true);

    virtual ~PR2ArmKinematics();

    /** 
     *  @brief Specifies if the node is active or not
     *  @return True if the node is active, false otherwise.
     */
    bool isActive();

    /**
     * @brief This is the basic IK service method that will compute and return an IK solution.
     * @param A request message. See service definition for GetPositionIK for more information on this message.
     * @param The response message. See service definition for GetPositionIK for more information on this message.
     */
    virtual bool getPositionIK(kinematics_msgs::GetPositionIK::Request &request, 
                               kinematics_msgs::GetPositionIK::Response &response);

    /**
     * @brief This is the basic kinematics info service that will return information about the kinematics node.
     * @param A request message. See service definition for GetKinematicSolverInfo for more information on this message.
     * @param The response message. See service definition for GetKinematicSolverInfo for more information on this message.
     */
    bool getIKSolverInfo(kinematics_msgs::GetKinematicSolverInfo::Request &request, 
                         kinematics_msgs::GetKinematicSolverInfo::Response &response);

    /**
     * @brief This is the basic kinematics info service that will return information about the kinematics node.
     * @param A request message. See service definition for GetKinematicSolverInfo for more information on this message.
     * @param The response message. See service definition for GetKinematicSolverInfo for more information on this message.
     */
    bool getFKSolverInfo(kinematics_msgs::GetKinematicSolverInfo::Request &request, 
                         kinematics_msgs::GetKinematicSolverInfo::Response &response);

    /**
     * @brief This is the basic forward kinematics service that will return information about the kinematics node.
     * @param A request message. See service definition for GetPositionFK for more information on this message.
     * @param The response message. See service definition for GetPositionFK for more information on this message.
     */
    bool getPositionFK(kinematics_msgs::GetPositionFK::Request &request, 
                       kinematics_msgs::GetPositionFK::Response &response);

    protected:

    // Helper function that assumes that everything is in the correct frame
    bool getPositionIKHelper(kinematics_msgs::GetPositionIK::Request &request, 
			     kinematics_msgs::GetPositionIK::Response &response);
    
    virtual bool transformPose(const std::string& des_frame,
			       const geometry_msgs::PoseStamped& pose_in,
			       geometry_msgs::PoseStamped& pose_out);
    
    bool active_;
    int free_angle_;
    urdf::Model robot_model_;
    double search_discretization_;
    ros::NodeHandle node_handle_, root_handle_;
    boost::shared_ptr<pr2_arm_kinematics::PR2ArmIKSolver> pr2_arm_ik_solver_;
    ros::ServiceServer ik_service_,fk_service_,ik_solver_info_service_,fk_solver_info_service_;
    tf::TransformListener* tf_;
    std::string root_name_;
    int dimension_;
    boost::shared_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_pose_solver_;
    KDL::Chain kdl_chain_;
    kinematics_msgs::KinematicSolverInfo ik_solver_info_, fk_solver_info_;
  };
}

#endif
