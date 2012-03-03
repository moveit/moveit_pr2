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

#include "constraints.h"
#include "sampler.h"
#include <moveit_msgs/ComputePlanningBenchmark.h>

static const std::string BENCHMARK_SERVICE_NAME="/ompl_planning/benchmark_planning_problem";

void setupEnv(void)
{
    ros::NodeHandle nh;
    tf::TransformListener tf;
    planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION, &tf);
    ros::Publisher pub_scene = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    planning_scene::PlanningScenePtr scene = psm.getPlanningScene();
    scene->setName("experiment1");
    Eigen::Affine3d t;
    t = Eigen::Translation3d(0.45, -0.45, 0.7);
    scene->getCollisionWorld()->addToObject("pole", new shapes::Box(0.1, 0.1, 1.4), t);
    ros::Duration(0.5).sleep();
    
    moveit_msgs::PlanningScene psmsg;
    psm.getPlanningScene()->getPlanningSceneMsg(psmsg);
    pub_scene.publish(psmsg);
    ROS_INFO("Scene published.");
    
    ros::Duration(0.5).sleep();
}

void benchmarkPathConstrained(const std::string &config)
{
    ros::NodeHandle nh;
    ros::service::waitForService(BENCHMARK_SERVICE_NAME);
    ros::ServiceClient benchmark_service_client = nh.serviceClient<moveit_msgs::ComputePlanningBenchmark>(BENCHMARK_SERVICE_NAME);
    
    moveit_msgs::ComputePlanningBenchmark::Request mplan_req;
    moveit_msgs::ComputePlanningBenchmark::Response mplan_res;
    
    planning_scene_monitor::PlanningSceneMonitor psm(ROBOT_DESCRIPTION, NULL);
    planning_scene::PlanningScene &scene = *psm.getPlanningScene();
    
    mplan_req.average_count = 50;
    mplan_req.motion_plan_request.planner_id = config;
    mplan_req.motion_plan_request.group_name = "right_arm"; 
    
    mplan_req.motion_plan_request.allowed_planning_time = ros::Duration(15.0);
    const std::vector<std::string>& joint_names = scene.getKinematicModel()->getJointModelGroup("right_arm")->getJointModelNames();
    mplan_req.motion_plan_request.goal_constraints.resize(1);
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints.resize(joint_names.size());
    
    for(unsigned int i = 0; i < joint_names.size(); i++)
    {
	mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].joint_name = joint_names[i];
	mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].position = 0.0;
	mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].tolerance_above = 1e-12;
	mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].tolerance_below = 1e-12;
	mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].weight = 1.0;
    }    
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[0].position = -0.30826385287398406;
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[1].position = 0.61185361475247468;
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[2].position = -0.67790861269459102;
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[3].position = -1.0372591097007691;
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[4].position = -0.89601966543848288; 
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[5].position = -1.9776217463278662; 
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[6].position = 1.8552611548679128;
    
    
    mplan_req.motion_plan_request.start_state.joint_state.name = joint_names;
    mplan_req.motion_plan_request.start_state.joint_state.position.push_back(-1.21044517893021499);
    mplan_req.motion_plan_request.start_state.joint_state.position.push_back(0.038959594993384528);
    mplan_req.motion_plan_request.start_state.joint_state.position.push_back(-0.81412902362644646);
    mplan_req.motion_plan_request.start_state.joint_state.position.push_back(-1.0989597173881371);
    mplan_req.motion_plan_request.start_state.joint_state.position.push_back(2.3582101183671629);
    mplan_req.motion_plan_request.start_state.joint_state.position.push_back(-1.993988668449755);
    mplan_req.motion_plan_request.start_state.joint_state.position.push_back(-2.2779628049776051);
    
    
    // add path constraints
    mplan_req.motion_plan_request.path_constraints = getSingleArmConstraints(); 
    
    benchmark_service_client.call(mplan_req, mplan_res);
}

void runExp(void)
{
    //    benchmarkPathConstrained("SBLkConfigDefault");
    //    benchmarkPathConstrained("ESTkConfigDefault");
    //    benchmarkPathConstrained("BKPIECEkConfigDefault");
    benchmarkPathConstrained("LBKPIECEkConfigDefault");
    benchmarkPathConstrained("KPIECEkConfigDefault");
    benchmarkPathConstrained("RRTkConfigDefault");
    benchmarkPathConstrained("RRTConnectkConfigDefault");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "iros_2012_exp", ros::init_options::AnonymousName);
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    setupEnv();
    runExp();
    
    return 0;
}
