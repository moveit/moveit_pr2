/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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

/** \author E. Gil Jones */

#include <pr2_gripper_controller/pr2_gripper_trajectory_controller_handler.h>
#include <pluginlib/class_list_macros.h>

namespace pr2_gripper_controller 
{

static const double GRIPPER_OPEN = 0.086;
static const double GRIPPER_CLOSED = 0.0;
static const double DEFAULT_GRIPPER_OBJECT_PRESENCE_THRESHOLD = 0.0021;

bool Pr2GripperTrajectoryControllerHandler::initialize(const std::string& group_name, 
                                                       const std::string& controller_name,
                                                       const std::string& ns_name)
{
  TrajectoryControllerHandler::initialize(group_name, controller_name, ns_name);
  pr2_gripper_action_client_.reset(new actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction>(controller_name+"/"+ns_name, true));
  
  while(ros::ok() && !pr2_gripper_action_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO_STREAM("Waiting for the pr2_gripper action for group " << group_name << " on the topic " << controller_name+"/"+ns_name << " to come up");
  }
  return true;
}

  /// \brief Start executing.  Gripper will either open or close.
bool Pr2GripperTrajectoryControllerHandler::executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory,
                                                    boost::shared_ptr<trajectory_execution::TrajectoryRecorder>& recorder,
                                                    const trajectory_execution::TrajectoryFinishedCallbackFunction& traj_callback)
{
  recorder_ = recorder;
  trajectory_finished_callback_ = traj_callback;
  
  initializeRecordedTrajectory(trajectory);
  
  pr2_controllers_msgs::Pr2GripperCommandGoal gripper_command;
  gripper_command.command.max_effort = 10000;
  
  double jval = trajectory.points[0].positions[0];
  if(jval > .5) {
    ROS_INFO_STREAM("Commanding gripper open");
    gripper_command.command.position = GRIPPER_OPEN;
  } else {
    ROS_INFO_STREAM("Commanding gripper closed");
    gripper_command.command.position = GRIPPER_CLOSED;
  }
  
  pr2_gripper_action_client_->sendGoal(gripper_command,
                                       boost::bind(&Pr2GripperTrajectoryControllerHandler::controllerDoneCallback, this, _1, _2),
                                       boost::bind(&Pr2GripperTrajectoryControllerHandler::controllerActiveCallback, this),
                                       boost::bind(&Pr2GripperTrajectoryControllerHandler::controllerFeedbackCallback, this, _1));
  
  recorder_->registerCallback(group_controller_ns_combo_name_, 
                              boost::bind(&Pr2GripperTrajectoryControllerHandler::addNewStateToRecordedTrajectory, this, _1, _2, _3));
  return true;
}

void Pr2GripperTrajectoryControllerHandler::controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                                                                   const pr2_controllers_msgs::Pr2GripperCommandResultConstPtr& result)
{
  ROS_INFO_STREAM("Gripper controller is done with state " << state.toString());
  
  if( controller_state_ == trajectory_execution::TrajectoryControllerStates::EXECUTING ||
      controller_state_ == trajectory_execution::TrajectoryControllerStates::OVERSHOOTING )
  {
    if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      controller_state_ = trajectory_execution::TrajectoryControllerStates::SUCCESS;
    }
    else
    {
      ROS_WARN_STREAM("Failed state is " << state.toString() );
      controller_state_ = trajectory_execution::TrajectoryControllerStates::EXECUTION_FAILURE;
    }
    
    // We don't record overshoot on the gripper
    done();
  }
}

void Pr2GripperTrajectoryControllerHandler::controllerActiveCallback() 
{
  ROS_DEBUG_STREAM("Controller went active");
}

void Pr2GripperTrajectoryControllerHandler::controllerFeedbackCallback(const pr2_controllers_msgs::Pr2GripperCommandFeedbackConstPtr& feedback)
{
  ROS_DEBUG_STREAM("Got feedback");
}

}

PLUGINLIB_DECLARE_CLASS(pr2_gripper_controller, Pr2GripperTrajectoryControllerHandler,
                        pr2_gripper_controller::Pr2GripperTrajectoryControllerHandler,
                        trajectory_execution::TrajectoryControllerHandler);
