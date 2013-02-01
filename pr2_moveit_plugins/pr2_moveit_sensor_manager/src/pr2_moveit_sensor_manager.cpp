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

#include <ros/ros.h>
#include <moveit/sensor_manager/sensor_manager.h>
#include <pluginlib/class_list_macros.h>
#include <boost/math/constants/constants.hpp>

#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>

namespace pr2_moveit_sensor_manager
{

class Pr2MoveItSensorManager : public moveit_sensor_manager::MoveItSensorManager
{
public:
  
  Pr2MoveItSensorManager() : node_handle_("~")
  {     
    node_handle_.param("head_pointing_frame", head_pointing_frame_, std::string("/camera"));
    head_action_client_.reset(new actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction>("/head_traj_controller/point_head_action", true));
  }
  
  virtual ~Pr2MoveItSensorManager()
  {
  }
  
  virtual void getSensorsList(std::vector<std::string> &names) const
  {
    names.resize(1);
    names[0] = "head";
  }
  
  virtual moveit_sensor_manager::SensorInfo getSensorInfo(const std::string &name) const
  {
    // I made this up.
    moveit_sensor_manager::SensorInfo si;
    if (name == "head")
    {
      si.origin_frame = head_pointing_frame_;
      si.min_dist = 0.1;
      si.max_dist = 3.0;
      si.x_angle = boost::math::constants::pi<double>() / 3.0;
      si.y_angle = boost::math::constants::pi<double>() / 3.0;
    }  
    else
      ROS_ERROR("Unknown sensor: '%s'", name.c_str());
    return si;
  }

  virtual bool hasSensors() const
  {
    return true;
  }
  
  virtual bool pointSensorTo(const std::string &name, const geometry_msgs::PointStamped &target, moveit_msgs::RobotTrajectory &sensor_trajectory)
  {
    if (name != "head")
    {
      ROS_ERROR("Unknown sensor: '%s'", name.c_str());
      return false;
    }    
    if (!head_action_client_->isServerConnected())
    {
      ROS_ERROR("Head action server is not connected");
      return false;
    }
    
    sensor_trajectory = moveit_msgs::RobotTrajectory();
    
    pr2_controllers_msgs::PointHeadGoal goal;
    goal.pointing_frame = head_pointing_frame_;
    goal.max_velocity = 1.0;
    goal.pointing_axis.x = 0.0;
    goal.pointing_axis.y = 0.0;
    goal.pointing_axis.z = 1.0;
    goal.target = target;
    actionlib::SimpleClientGoalState result = head_action_client_->sendGoalAndWait(goal, ros::Duration(5.0));
    if (result == actionlib::SimpleClientGoalState::SUCCEEDED)
      return true;
    else
    {
      ROS_WARN_STREAM("Head moving action is done with state " << result.toString() << ": " << result.getText());
      return false;
    }
  }
  
protected: 
  
  ros::NodeHandle node_handle_;
  std::string head_pointing_frame_;  
  boost::shared_ptr<actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> > head_action_client_;
  
};

PLUGINLIB_EXPORT_CLASS(pr2_moveit_sensor_manager::Pr2MoveItSensorManager,
                       moveit_sensor_manager::MoveItSensorManager);
}
