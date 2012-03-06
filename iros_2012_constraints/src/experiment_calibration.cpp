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
#include <ompl_interface_ros/ompl_interface_ros.h>
#include <moveit_msgs/ComputePlanningBenchmark.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/DisplayTrajectory.h>

static const std::string BENCHMARK_SERVICE_NAME="/ompl_planning/benchmark_planning_problem";
static const std::string PLANNING_SERVICE_NAME="/ompl_planning/plan_kinematic_path";

planning_scene_monitor::PlanningSceneMonitor *psm = NULL;

void setupEnv(void)
{
    psm = new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION);
    ros::NodeHandle nh;
    ros::Publisher pub_scene = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    planning_scene::PlanningScenePtr scene = psm->getPlanningScene();
    scene->setName("experiment2");

    // add a pole
    Eigen::Affine3d t;
    t = Eigen::Translation3d(0.65, 0, 0.7);
    scene->getCollisionWorld()->addToObject("pole", new shapes::Box(0.1, 0.1, 1.4), t);

    // add an attached object
    moveit_msgs::AttachedCollisionObject aco;
    aco.link_name = "r_wrist_roll_link";
    aco.touch_links.push_back("r_wrist_roll_link");
    aco.touch_links.push_back("r_gripper_l_finger_tip_link");
    aco.touch_links.push_back("r_gripper_r_finger_tip_link");
    
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
    psm->getPlanningScene()->processAttachedCollisionObjectMsg(aco);

    std::map<std::string, double> sv; sv["head_tilt_joint"] = 0.5;
    psm->getPlanningScene()->getCurrentState().setStateValues(sv);
    std::vector<double> tuck(7);
    tuck[0] = 0.06024;
    tuck[1] = 1.248526;
    tuck[2] =  1.789070;
    tuck[3] = -1.683386;
    tuck[4] = -1.7343417;
    tuck[5] = -0.0962141;
    tuck[6] = -0.0864407;
    psm->getPlanningScene()->getCurrentState().getJointStateGroup("left_arm")->setStateValues(tuck);
    
    ros::WallDuration(1.0).sleep();
    
    moveit_msgs::PlanningScene psmsg;
    psm->getPlanningScene()->getPlanningSceneMsg(psmsg);
    pub_scene.publish(psmsg);
    ROS_INFO("Scene published.");
    
    ros::WallDuration(1.0).sleep();
}

void benchmarkPathConstrained(const std::string &config)
{
    ros::NodeHandle nh;
    ros::service::waitForService(BENCHMARK_SERVICE_NAME);
    ros::ServiceClient benchmark_service_client = nh.serviceClient<moveit_msgs::ComputePlanningBenchmark>(BENCHMARK_SERVICE_NAME);
    
    moveit_msgs::ComputePlanningBenchmark::Request mplan_req;
    moveit_msgs::ComputePlanningBenchmark::Response mplan_res;
    
    mplan_req.average_count = 50;
    mplan_req.motion_plan_request.planner_id = config;
    mplan_req.motion_plan_request.group_name = "right_arm"; 
    
    mplan_req.motion_plan_request.allowed_planning_time = ros::Duration(30.0);


    const std::vector<std::string>& joint_names = psm->getPlanningScene()->getKinematicModel()->getJointModelGroup("right_arm")->getJointModelNames();
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
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[0].position = -0.284597;
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[1].position = 0.5359;
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[2].position = 0.359428;
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[3].position = -0.164032;
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[4].position = -0.640892; 
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[5].position = -1.71483;    
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[6].position = 1.73173;
    
    
    mplan_req.motion_plan_request.start_state.joint_state.name = joint_names;
    mplan_req.motion_plan_request.start_state.joint_state.position.push_back(0.371285);
    mplan_req.motion_plan_request.start_state.joint_state.position.push_back(0.452859);
    mplan_req.motion_plan_request.start_state.joint_state.position.push_back(-1.33889);
    mplan_req.motion_plan_request.start_state.joint_state.position.push_back(-0.690347);
    mplan_req.motion_plan_request.start_state.joint_state.position.push_back(1.61033);
    mplan_req.motion_plan_request.start_state.joint_state.position.push_back(-0.958497);
    mplan_req.motion_plan_request.start_state.joint_state.position.push_back(0.652208);
    
    
    // add path constraints
    mplan_req.motion_plan_request.path_constraints = getVisibilityConstraints("attached");
    
    benchmark_service_client.call(mplan_req, mplan_res);
}

void runExp(void)
{
    //    benchmarkPathConstrained("SBLkConfigDefault");
    //    benchmarkPathConstrained("ESTkConfigDefault");
    //    benchmarkPathConstrained("BKPIECEkConfigDefault");

    //    benchmarkPathConstrained("LBKPIECEkConfigDefault");
    benchmarkPathConstrained("KPIECEkConfigDefault");
    //    benchmarkPathConstrained("RRTkConfigDefault");
        benchmarkPathConstrained("RRTConnectkConfigDefault");
}

void testPlan(void)
{    
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<moveit_msgs::DisplayTrajectory>("display_motion_plan", 1);
    ros::service::waitForService(PLANNING_SERVICE_NAME);    

    ros::ServiceClient service_client = nh.serviceClient<moveit_msgs::GetMotionPlan>(PLANNING_SERVICE_NAME);
    
    moveit_msgs::GetMotionPlan::Request mplan_req;
    moveit_msgs::GetMotionPlan::Response mplan_res;
    
    mplan_req.motion_plan_request.planner_id = "KPIECEkConfigDefault";
    mplan_req.motion_plan_request.group_name = "right_arm";     
    mplan_req.motion_plan_request.allowed_planning_time = ros::Duration(65.0);
    mplan_req.motion_plan_request.random_valid_start_goal = true;
        
    // add path constraintsx
    mplan_req.motion_plan_request.path_constraints = getVisibilityConstraints("attached");
    
    if (service_client.call(mplan_req, mplan_res))
    {
      moveit_msgs::DisplayTrajectory d;
      d.model_id = psm->getPlanningScene()->getKinematicModel()->getName();
      d.trajectory_start = mplan_res.trajectory_start;
      d.trajectory = mplan_res.trajectory;
      std::cout << d.trajectory.joint_trajectory.points[0] << std::endl;
      std::cout << d.trajectory.joint_trajectory.points.back() << std::endl;
      pub.publish(d);
      ros::Duration(0.5).sleep();
    }
}

void computeDB(void)
{
    ompl_interface_ros::OMPLInterfaceROS ompl_interface(psm->getPlanningScene()->getKinematicModel());
    moveit_msgs::Constraints c =  getVisibilityConstraints("attached");
    ompl_interface.addConstraintApproximation(c, "right_arm", "JointModel", psm->getPlanningScene()->getCurrentState(), 100000, 20);
    ompl_interface.saveConstraintApproximations("/home/isucan/c/");
    ROS_INFO("Done");
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "iros_2012_exp", ros::init_options::AnonymousName);
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    setupEnv();    
    //    computeDB();
    

    //    testPlan();
    runExp();
    
    return 0;
}
