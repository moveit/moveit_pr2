/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <trajectory_execution_ros/trajectory_execution_monitor_ros.h>
#include <planning_scene_monitor/planning_scene_monitor.h>
#include <csignal>

boost::shared_ptr<trajectory_execution_ros::TrajectoryExecutionMonitorRos> emon;

void sigHandler(int x) {
  emon.reset();
  exit(0);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_execution_test", ros::init_options::NoSigintHandler);

  signal(SIGINT, sigHandler);
  signal(SIGTERM, sigHandler);

  boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

  emon.reset(new trajectory_execution_ros::TrajectoryExecutionMonitorRos(planning_scene_monitor_->getPlanningScene()->getRobotModel()));

  ROS_INFO_STREAM("Current for arms is " << emon->getCurrentController("arms"));

  //should switch to default controller for arms, stopping other two
  //emon->switchAssociatedStopStartControllers("arms", "both_arms_controller");

  //sh
  //emon->restoreToOriginalConfiguration();

  ros::waitForShutdown();
}
