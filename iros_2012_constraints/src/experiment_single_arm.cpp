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
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <ompl_interface_ros/ompl_interface_ros.h>

static const std::string BENCHMARK_SERVICE_NAME="/ompl_planning/benchmark_planning_problem";
static const std::string PLANNING_SERVICE_NAME="/ompl_planning/plan_kinematic_path";

planning_scene_monitor::PlanningSceneMonitor *psm = NULL;

void setupEnv(void)
{  
    psm = new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION);
    ros::NodeHandle nh;
    tf::TransformListener tf;
    ros::Publisher pub_scene = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
    planning_scene::PlanningScenePtr scene = psm->getPlanningScene();
    scene->setName("experiment1");
    Eigen::Affine3d t;
    t = Eigen::Translation3d(0.48, -0.47, 0.7);
    scene->getCollisionWorld()->addToObject("pole1", new shapes::Box(0.1, 0.1, 1.4), t);
    t = Eigen::Translation3d(0.48, 0.45, 0.7);
    scene->getCollisionWorld()->addToObject("pole2", new shapes::Box(0.1, 0.3, 1.4), t);
    t = Eigen::Translation3d(0.38, -0.57, 0.7);
    scene->getCollisionWorld()->addToObject("pole3", new shapes::Box(0.3, 0.1, 1.4), t);
    t = Eigen::Translation3d(0.6, 0., 0.35);
    scene->getCollisionWorld()->addToObject("table", new shapes::Box(0.3, 0.3, t.translation().z()*2.0), t);



    std::vector<double> tuck(7);
    tuck[0] = 0.06024;
    tuck[1] = 1.248526;
    tuck[2] =  1.789070;
    tuck[3] = -1.683386;
    tuck[4] = -1.7343417;
    tuck[5] = -0.0962141;
    tuck[6] = -0.0864407;
    psm->getPlanningScene()->getCurrentState().getJointStateGroup("left_arm")->setStateValues(tuck);
    
    
    moveit_msgs::PlanningScene psmsg;
    psm->getPlanningScene()->getPlanningSceneMsg(psmsg);
    ros::WallDuration(1.0).sleep();
    pub_scene.publish(psmsg);
    ros::WallDuration(1.0).sleep();
    ROS_INFO("Scene published.");
}

void benchmarkPathConstrained(const std::string &config)
{
    ros::NodeHandle nh;
    ros::service::waitForService(BENCHMARK_SERVICE_NAME);
    ros::ServiceClient benchmark_service_client = nh.serviceClient<moveit_msgs::ComputePlanningBenchmark>(BENCHMARK_SERVICE_NAME);
    
    moveit_msgs::ComputePlanningBenchmark::Request mplan_req;
    moveit_msgs::ComputePlanningBenchmark::Response mplan_res;
    
    planning_scene::PlanningScene &scene = *psm->getPlanningScene();
    
    mplan_req.average_count = 100;
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
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[5].position = -1.7;//-1.9776217463278662; 
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
  benchmarkPathConstrained("SBLkConfigDefault");
  //  benchmarkPathConstrained("ESTkConfigDefault");
  //  benchmarkPathConstrained("BKPIECEkConfigDefault");
  benchmarkPathConstrained("LBKPIECEkConfigDefault");
  benchmarkPathConstrained("KPIECEkConfigDefault");
  benchmarkPathConstrained("RRTkConfigDefault");
  benchmarkPathConstrained("RRTConnectkConfigDefault");
}

void testPlan(void)
{
    ros::NodeHandle nh;
    ros::service::waitForService(PLANNING_SERVICE_NAME);
    ros::Publisher pub = nh.advertise<moveit_msgs::DisplayTrajectory>("display_motion_plan", 1);
    
    ros::ServiceClient service_client = nh.serviceClient<moveit_msgs::GetMotionPlan>(PLANNING_SERVICE_NAME);
    
    moveit_msgs::GetMotionPlan::Request mplan_req;
    moveit_msgs::GetMotionPlan::Response mplan_res;
    
    planning_scene::PlanningScene &scene = *psm->getPlanningScene();
    
    mplan_req.motion_plan_request.group_name = "right_arm";
    mplan_req.motion_plan_request.num_planning_attempts = 1;
    mplan_req.motion_plan_request.allowed_planning_time = ros::Duration(15.0);
    const std::vector<std::string>& joint_names = scene.getKinematicModel()->getJointModelGroup("right_arm")->getJointModelNames();
    mplan_req.motion_plan_request.goal_constraints.resize(1);
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints.resize(joint_names.size());
    for(unsigned int i = 0; i < joint_names.size(); i++)
    {
        mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].joint_name = joint_names[i];
        mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].position = 0.0;
        mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].tolerance_above = 0.001;
        mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].tolerance_below = 0.001;
        mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].weight = 1.0;
    }
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[0].position = -0.20826385287398406;
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[1].position = 0.61185361475247468;
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[2].position = -0.67790861269459102;
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[3].position = -1.0372591097007691;
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[4].position = -0.89601966543848288; 
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[5].position = -1.7;
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[6].position = 1.8552611548679128;
    
    

    mplan_req.motion_plan_request.start_state.joint_state.name = joint_names;
    mplan_req.motion_plan_request.start_state.joint_state.position.push_back(-1.21044517893021499);
    mplan_req.motion_plan_request.start_state.joint_state.position.push_back(0.038959594993384528);
    mplan_req.motion_plan_request.start_state.joint_state.position.push_back(-0.81412902362644646);
    mplan_req.motion_plan_request.start_state.joint_state.position.push_back(-1.0989597173881371);
    mplan_req.motion_plan_request.start_state.joint_state.position.push_back(2.3582101183671629);
    mplan_req.motion_plan_request.start_state.joint_state.position.push_back(-1.993988668449755);
    mplan_req.motion_plan_request.start_state.joint_state.position.push_back(-2.2779628049776051);
    
    mplan_req.motion_plan_request.path_constraints = getSingleArmConstraints(); 

    if (service_client.call(mplan_req, mplan_res))
    {
	moveit_msgs::DisplayTrajectory d;
	d.model_id = scene.getKinematicModel()->getName();
	d.trajectory_start = mplan_res.trajectory_start;
	d.trajectory = mplan_res.trajectory;
	pub.publish(d);
	ros::WallDuration(0.5).sleep();
    }
    
}

void computeDB(void)
{
    ompl_interface_ros::OMPLInterfaceROS ompl_interface(psm->getPlanningScene()->getKinematicModel());
    moveit_msgs::Constraints c = getSingleArmConstraints(); 
    ompl_interface.addConstraintApproximation(c, "right_arm", "PoseModel", psm->getPlanningScene()->getCurrentState(), 10000, 100);
    ompl_interface.saveConstraintApproximations("/home/isucan/c/");
    ROS_INFO("Done");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "iros_2012_exp", ros::init_options::AnonymousName);
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    setupEnv();

    //    testPlan(); runExp();
    
    computeDB();
    
    return 0;
}
