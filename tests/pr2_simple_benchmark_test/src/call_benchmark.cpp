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

#include <planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/ComputePlanningBenchmark.h>
#include <kinematic_constraints/utils.h>

static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string BENCHMARK_SERVICE_NAME="benchmark_planning_problem"; // name of the advertised service (within the ~ namespace)

int main(int argc, char **argv)
{
  ros::init(argc, argv, "call_moveit_benchmark", ros::init_options::AnonymousName);
      
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  // fill a benchmark request
  moveit_msgs::ComputePlanningBenchmark::Request req;
  moveit_msgs::ComputePlanningBenchmark::Request res;
  
  // fill planning scene
  boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener());
  planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION, tf);

  Eigen::Affine3d t;
  t = Eigen::Translation3d(0.45, -0.45, 0.7);
  psm.getPlanningScene()->getWorldNonConst()->addToObject("pole", shapes::ShapePtr(new shapes::Box(0.1, 0.1, 1.4)), t);

  if (psm.getPlanningScene()->isConfigured())
    psm.getPlanningScene()->getPlanningSceneMsg(req.scene);
  req.motion_plan_request.start_state = req.scene.robot_state;
  
  // average over 3 runs
  req.default_average_count = 10;
  req.filename = "benchmark_results.log";
  
  // fill in a goal
  req.motion_plan_request.group_name = "right_arm";
  req.motion_plan_request.num_planning_attempts = 1;
  req.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  const std::vector<std::string>& joint_names = psm.getPlanningScene()->getRobotModel()->getJointModelGroup("right_arm")->getJointModelNames();
  req.motion_plan_request.goal_constraints.resize(1);
  req.motion_plan_request.goal_constraints[0].joint_constraints.resize(joint_names.size());
  for(unsigned int i = 0; i < joint_names.size(); i++)
  {
      req.motion_plan_request.goal_constraints[0].joint_constraints[i].joint_name = joint_names[i];
      req.motion_plan_request.goal_constraints[0].joint_constraints[i].position = 0.0;
      req.motion_plan_request.goal_constraints[0].joint_constraints[i].tolerance_above = 0.001;
      req.motion_plan_request.goal_constraints[0].joint_constraints[i].tolerance_below = 0.001;
      req.motion_plan_request.goal_constraints[0].joint_constraints[i].weight = 1.0;
  }
  req.motion_plan_request.goal_constraints[0].joint_constraints[0].position = -2.0;
  req.motion_plan_request.goal_constraints[0].joint_constraints[3].position = -.2;
  req.motion_plan_request.goal_constraints[0].joint_constraints[5].position = -.2;
  /*

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = psm.getPlanningScene()->getRobotModel()->getModelFrame();
  pose.pose.position.x = 0.55;
  pose.pose.position.y = 0.2;
  pose.pose.position.z = 1.25;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;
  moveit_msgs::Constraints goal = kinematic_constraints::constructGoalConstraints("r_wrist_roll_link", pose);
  req.motion_plan_request.goal_constraints.push_back(goal);
  */
  
  req.planner_interfaces.resize(1);
  req.planner_interfaces[0].name = "ompl_interface_ros/OMPLPlanner";
  
  ros::NodeHandle nh;
  ros::service::waitForService(BENCHMARK_SERVICE_NAME);
  ros::ServiceClient benchmark_service_client = nh.serviceClient<moveit_msgs::ComputePlanningBenchmark>(BENCHMARK_SERVICE_NAME);
  if (!benchmark_service_client.call(req, res))
      ROS_ERROR("Error calling benchmark service");
  
  ros::waitForShutdown();
  
  return 0;
}
