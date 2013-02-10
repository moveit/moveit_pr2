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
#include <gtest/gtest.h>
#include <pr2_mechanism_msgs/ListControllers.h>

boost::shared_ptr<trajectory_execution_ros::TrajectoryExecutionMonitorRos> emon;
boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
ros::ServiceClient lister_service;

void sigHandler(int x) {
  emon.reset();
  exit(0);
}

void getRunningControllerMap(std::map<std::string, bool>& controller_map) {
  controller_map.clear();

  pr2_mechanism_msgs::ListControllers::Request req;
  pr2_mechanism_msgs::ListControllers::Response res;

  if(!lister_service.call(req, res)) {
    ROS_WARN_STREAM("Something wrong with lister service");
    return;
  }
  for(unsigned int i = 0; i < res.controllers.size(); i++) {
    controller_map[res.controllers[i]] = (res.state[i] == "running");
  }
}

TEST(TrajectoryExecutionTests, loadUnload) {

  emon.reset(new trajectory_execution_ros::TrajectoryExecutionMonitorRos(planning_scene_monitor_->getPlanningScene()->getRobotModel()));
  std::map<std::string, bool> initial_map = emon->getOriginalControllerConfiguration();
  emon.reset();
  std::map<std::string, bool> after_map;
  getRunningControllerMap(after_map);
  ASSERT_EQ(after_map.size(), initial_map.size());
  std::map<std::string, bool>::iterator it2 = after_map.begin();
  for(std::map<std::string, bool>::iterator it = initial_map.begin();
      it != initial_map.end();
      it++, it2++) {
    EXPECT_EQ(it->first, it2->first);
    EXPECT_EQ(it->second, it2->second);
  }
}

TEST(TrajectoryExecutionTests, switchRestore) {
  emon.reset(new trajectory_execution_ros::TrajectoryExecutionMonitorRos(planning_scene_monitor_->getPlanningScene()->getRobotModel()));
  std::map<std::string, bool> initial_map = emon->getOriginalControllerConfiguration();

  EXPECT_TRUE(emon->getCurrentController("arms") == "r_arm_controller" ||
              emon->getCurrentController("arms") == "l_arm_controller");
  emon->switchAssociatedStopStartControllers("arms", "both_arms_controller");

  EXPECT_EQ(emon->getCurrentController("arms"), "both_arms_controller");
  EXPECT_EQ(emon->getCurrentController("right_arm"), "both_arms_controller");
  EXPECT_EQ(emon->getCurrentController("left_arm"), "both_arms_controller");

  emon->switchAssociatedStopStartControllers("right_arm", "r_arm_controller");
  //should be right arm, as that's what we replaced arms with
  EXPECT_EQ(emon->getCurrentController("arms"), "r_arm_controller");
  EXPECT_EQ(emon->getCurrentController("left_arm"), "l_arm_controller");

  emon->restoreOriginalControllers();

  std::map<std::string, bool> after_map;
  getRunningControllerMap(after_map);
  ASSERT_EQ(after_map.size(), initial_map.size());
  std::map<std::string, bool>::iterator it2 = after_map.begin();
  for(std::map<std::string, bool>::iterator it = initial_map.begin();
      it != initial_map.end();
      it++, it2++) {
    EXPECT_EQ(it->first, it2->first);
    EXPECT_EQ(it->second, it2->second);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_execution_tests", ros::init_options::NoSigintHandler);
  testing::InitGoogleTest(&argc, argv);

  signal(SIGINT, sigHandler);
  signal(SIGTERM, sigHandler);

  ros::NodeHandle nh;

  ros::service::waitForService("/pr2_controller_manager/list_controllers");

  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
  lister_service = nh.serviceClient<pr2_mechanism_msgs::ListControllers>("/pr2_controller_manager/list_controllers", true);

  int ret = RUN_ALL_TESTS();

  emon.reset();
  
  return ret;
  
  // ROS_INFO_STREAM("Current for arms is " << emon->getCurrentController("arms"));

  // //should switch to default controller for arms, stopping other two
  // emon->switchAssociatedStopStartControllers("arms", "both_arms_controller");

  // //sh
  // //emon->restoreToOriginalConfiguration();

  // ros::waitForShutdown();
}
