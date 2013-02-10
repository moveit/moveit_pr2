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

/* Author: Ioan Sucan */

#include <gtest/gtest.h>
#include <planning_scene_monitor/planning_scene_monitor.h>
#include <ompl_interface_ros/ompl_interface_ros.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <kinematic_constraints/utils.h>

static const std::string PLANNER_SERVICE_NAME="/ompl_planning/plan_kinematic_path";
static const std::string ROBOT_DESCRIPTION="robot_description";

TEST(OmplPlanning, PathConstrainedSimplePlan)
{
  ros::NodeHandle nh;
  ros::service::waitForService(PLANNER_SERVICE_NAME);
  ros::Publisher pub = nh.advertise<moveit_msgs::DisplayTrajectory>("display_motion_plan", 1);
  
  ros::ServiceClient planning_service_client = nh.serviceClient<moveit_msgs::GetMotionPlan>(PLANNER_SERVICE_NAME);
  EXPECT_TRUE(planning_service_client.exists());
  EXPECT_TRUE(planning_service_client.isValid());
  
  moveit_msgs::GetMotionPlan::Request mplan_req;
  moveit_msgs::GetMotionPlan::Response mplan_res;
  
  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION);
  planning_scene::PlanningScene &scene = *psm.getPlanningScene();
  EXPECT_TRUE(scene.isConfigured());
  
  mplan_req.motion_plan_request.planner_id = "RRTConnectkConfigDefault";
  mplan_req.motion_plan_request.group_name = "right_arm";
  mplan_req.motion_plan_request.num_planning_attempts = 1;
  mplan_req.motion_plan_request.allowed_planning_time = ros::Duration(15.0);
  const std::vector<std::string>& joint_names = scene.getRobotModel()->getJointModelGroup("right_arm")->getJointModelNames();

  mplan_req.motion_plan_request.start_state.joint_state.name = joint_names;
  mplan_req.motion_plan_request.start_state.joint_state.position.push_back(-1.21044517893021499);
  mplan_req.motion_plan_request.start_state.joint_state.position.push_back(0.038959594993384528);
  mplan_req.motion_plan_request.start_state.joint_state.position.push_back(-0.81412902362644646);
  mplan_req.motion_plan_request.start_state.joint_state.position.push_back(-1.0989597173881371);
  mplan_req.motion_plan_request.start_state.joint_state.position.push_back(2.3582101183671629);
  mplan_req.motion_plan_request.start_state.joint_state.position.push_back(-1.993988668449755);
  mplan_req.motion_plan_request.start_state.joint_state.position.push_back(-2.2779628049776051);

  
  moveit_msgs::PositionConstraint pcm;
  pcm.link_name = "r_wrist_roll_link";
  pcm.header.frame_id = psm.getPlanningScene()->getPlanningFrame();

  pcm.constraint_region.primitive_poses.resize(1);
  pcm.constraint_region.primitive_poses[0].position.x = 0.5;
  pcm.constraint_region.primitive_poses[0].position.y = 0.0;
  pcm.constraint_region.primitive_poses[0].position.z = 0.7;
  pcm.constraint_region.primitive_poses[0].orientation.w = 1.0;

  pcm.constraint_region.primitives.resize(1);
  pcm.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  pcm.constraint_region.primitives[0].dimensions.x = 0.1;
  pcm.constraint_region.primitives[0].dimensions.y = 0.1;
  pcm.constraint_region.primitives[0].dimensions.z = 0.1;
  pcm.weight = 1.0;
  mplan_req.motion_plan_request.goal_constraints.resize(1);
  mplan_req.motion_plan_request.goal_constraints[0].position_constraints.push_back(pcm);


  // add path constraints
  moveit_msgs::Constraints &constr = mplan_req.motion_plan_request.path_constraints;  
  constr.orientation_constraints.resize(1);
  moveit_msgs::OrientationConstraint &ocm = constr.orientation_constraints[0];
  ocm.link_name = "r_wrist_roll_link";
  ocm.header.frame_id = psm.getPlanningScene()->getPlanningFrame();
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.15;
  ocm.absolute_y_axis_tolerance = 0.15;
  ocm.absolute_z_axis_tolerance = M_PI;
  ocm.weight = 1.0; 
  std::cout << mplan_req.motion_plan_request << std::endl;
  
  ASSERT_TRUE(planning_service_client.call(mplan_req, mplan_res));
  ASSERT_EQ(mplan_res.error_code.val, mplan_res.error_code.SUCCESS);
  EXPECT_GT(mplan_res.trajectory.joint_trajectory.points.size(), 0);
  
  
  moveit_msgs::DisplayTrajectory d;
  d.model_id = scene.getRobotModel()->getName();
  d.trajectory_start = mplan_res.trajectory_start;
  d.trajectory = mplan_res.trajectory;
  pub.publish(d);
  ros::Duration(0.5).sleep();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  
  ros::init(argc, argv, "test_ompl_planning", ros::init_options::AnonymousName);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  return RUN_ALL_TESTS();
}
