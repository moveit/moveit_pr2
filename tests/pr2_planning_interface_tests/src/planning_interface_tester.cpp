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

#include "planning_interface/planning_interface.h"
#include <planning_scene_monitor/planning_scene_monitor.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <gtest/gtest.h>

static planning_scene_monitor::PlanningSceneMonitor *g_psm = NULL;

TEST(PlanningInterfaceTester, loadAllPlanners)
{
  pluginlib::ClassLoader<planning_interface::Planner>* planner_loader;
  try
  {
    planner_loader = new pluginlib::ClassLoader<planning_interface::Planner>("planning_interface", "planning_interface::Planner");
  }
  catch(pluginlib::PluginlibException& ex)
  {
    FAIL() << "Exception while creating class loader " << ex.what();
  }

  std::vector<std::string> classes;
  std::vector<boost::shared_ptr<planning_interface::Planner> > planners;
  planning_scene::PlanningSceneConstPtr scene = g_psm->getPlanningScene();
  planning_models::RobotModelConstPtr model = scene->getRobotModel();

  classes = planner_loader->getDeclaredClasses();
  // Must have some planners
  ASSERT_GT(classes.size(), 0);
  printf("Loading classes:\n");
  for(std::vector<std::string>::const_iterator it = classes.begin();
      it != classes.end();
      ++it)  
    printf("  %s\n", it->c_str());
  fflush(stdout);
  return;
  
  for(std::vector<std::string>::const_iterator it = classes.begin();
      it != classes.end();
      ++it)
  {
    try
    {
      boost::shared_ptr<planning_interface::Planner> p(planner_loader->createUnmanagedInstance(*it));
      p->init(model);
      planners.push_back(p);
    }
    catch(pluginlib::PluginlibException& ex)
    {
      // All planners must load
      ADD_FAILURE() << "Exception while loading planner: " << *it << ": " << ex.what();
    }
  }

  for(std::vector<boost::shared_ptr<planning_interface::Planner> >::const_iterator it = planners.begin();
      it != planners.end();
      ++it)
  {
    // A dumb test: require that the planners return true from
    // canServiceRequest
    moveit_msgs::GetMotionPlan::Request req;
    planning_interface::PlannerCapability capabilities;
    bool can_service = (*it)->canServiceRequest(req, capabilities);
    EXPECT_TRUE(can_service);
   
    // Another dumb test: require that the planners return false from solve
    moveit_msgs::GetMotionPlan::Response res;
    bool solved = (*it)->solve(scene, req, res);
    EXPECT_FALSE(solved);
  }
}

static const std::string ROBOT_DESCRIPTION="robot_description";


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "planner_loader");

  g_psm = new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION);
  if(g_psm->getPlanningScene() && g_psm->getPlanningScene()->isConfigured())
  {
    g_psm->startWorldGeometryMonitor();
    g_psm->startSceneMonitor();
    g_psm->startStateMonitor();
    int r = RUN_ALL_TESTS();
    delete g_psm;
    return r;
  }
  else
  {
    ROS_ERROR("Planning scene not configured");
    delete g_psm;
    
    return 1;
  }
}
