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
    scene->setName("experiment2");

    // add a pole
    Eigen::Affine3d t;
    t = Eigen::Translation3d(0.65, 0, 0.7);
    scene->getCollisionWorld()->addToObject("pole", new shapes::Box(0.1, 0.1, 1.4), t);

    // add an attached object
    moveit_msgs::AttachedCollisionObject aco;
    aco.link_name = "r_wrist_roll_link";
    aco.touch_links.push_back("r_wrist_roll_link");
    
    moveit_msgs::CollisionObject &co = aco.object;
    co.id = "attached";
    co.header.stamp = ros::Time::now();
    co.header.frame_id = aco.link_name;
    co.operation = moveit_msgs::CollisionObject::ADD;
    co.shapes.resize(1);
    co.shapes[0].type = moveit_msgs::Shape::BOX;
    co.shapes[0].dimensions.push_back(0.3);
    co.shapes[0].dimensions.push_back(0.01);
    co.shapes[0].dimensions.push_back(0.3);
    co.poses.resize(1);
    co.poses[0].position.x = 0.28;
    co.poses[0].position.y = 0;
    co.poses[0].position.z = 0;
    co.poses[0].orientation.w = 1.0;
    psm.getPlanningScene()->processAttachedCollisionObjectMsg(aco);

    std::map<std::string, double> sv; sv["head_tilt_joint"] = 0.5;
    psm.getPlanningScene()->getCurrentState().setStateValues(sv);
    std::vector<double> tuck(7);
    tuck[0] = 0.06024;
    tuck[1] = 1.248526;
    tuck[2] =  1.789070;
    tuck[3] = -1.683386;
    tuck[4] = -1.7343417;
    tuck[5] = -0.0962141;
    tuck[6] = -0.0864407;
    psm.getPlanningScene()->getCurrentState().getJointStateGroup("left_arm")->setStateValues(tuck);
    
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
    
    mplan_req.average_count = 50;
    mplan_req.motion_plan_request.planner_id = config;
    mplan_req.motion_plan_request.group_name = "right_arm"; 
    
    mplan_req.motion_plan_request.allowed_planning_time = ros::Duration(15.0);
    mplan_req.motion_plan_request.random_valid_start_goal = true;
        
    // add path constraints
    mplan_req.motion_plan_request.path_constraints = getVisibilityConstraints("attached");
    
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
