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
#include <ompl/base/spaces/SE3StateSpace.h>

static const std::string BENCHMARK_SERVICE_NAME="/ompl_planning/benchmark_planning_problem";
static const std::string PLANNING_SERVICE_NAME="/ompl_planning/plan_kinematic_path";

planning_scene_monitor::PlanningSceneMonitor *psm = NULL;
std::vector<double> positions(14);
std::vector<double> offer_tray(14);
std::vector<double> left_side(14);

void setupEnv(void)
{
  sleep(1);
    psm = new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION);
    ros::NodeHandle nh;
    sleep(1);
    ros::Publisher pub_scene = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 5);
    planning_scene::PlanningScenePtr scene = psm->getPlanningScene();
    scene->setName("experiment3");

    // add a pole
    Eigen::Affine3d t;
    t = Eigen::Translation3d(0.9, -0.2, 0.65);
    scene->getCollisionWorld()->addToObject("table", new shapes::Box(0.7, 1.5, 0.04), t);

    t = Eigen::Translation3d(0.9, -0.2, 0.31);
    scene->getCollisionWorld()->addToObject("support", new shapes::Box(0.08, 0.08, 0.61), t);

    t = Eigen::Translation3d(0.65, 0.3, 0.72);
    scene->getCollisionWorld()->addToObject("object1", new shapes::Box(0.04, 0.04, 0.14), t);

    t = Eigen::Translation3d(0.75, 0.3, 0.72);
    scene->getCollisionWorld()->addToObject("object2", new shapes::Box(0.04, 0.04, 0.14), t);

    std_msgs::ColorRGBA c;
    c.r = 0.9f;
    c.g = 0.0f;
    c.b = 0.0f;
    scene->setColor("object1", c);
    scene->setColor("object2", c);

    c.r = 0.1f;
    c.g = 0.8f;
    c.b = 0.3f;
    scene->setColor("table", c);
    scene->setColor("support", c);

    c.r = 0.1f;
    c.g = 0.2f;
    c.b = 0.9f;
    scene->setColor("attached", c);

    // add an attached object
    moveit_msgs::AttachedCollisionObject aco;
    aco.link_name = "r_wrist_roll_link";
    aco.touch_links.push_back("r_wrist_roll_link");
    aco.touch_links.push_back("r_gripper_l_finger_tip_link");
    aco.touch_links.push_back("r_gripper_r_finger_tip_link");
    aco.touch_links.push_back("l_gripper_l_finger_tip_link");
    aco.touch_links.push_back("l_gripper_r_finger_tip_link");
    
    moveit_msgs::CollisionObject &co = aco.object;
    co.id = "attached";
    co.header.stamp = ros::Time::now();
    co.header.frame_id = aco.link_name;
    co.operation = moveit_msgs::CollisionObject::ADD;
    co.shapes.resize(1);
    co.shapes[0].type = moveit_msgs::Shape::BOX;
    co.shapes[0].dimensions.push_back(0.35);
    co.shapes[0].dimensions.push_back(0.01);
    co.shapes[0].dimensions.push_back(0.15);
    co.poses.resize(1);
    co.poses[0].position.x = 0.34;
    co.poses[0].position.y = 0;
    co.poses[0].position.z = 0;
    co.poses[0].orientation.w = 1.0;
    psm->getPlanningScene()->processAttachedCollisionObjectMsg(aco);

    positions[0] = 0.731802;
    positions[1] = 0.845047;
    positions[2] = 1.46567;
    positions[3] = -1.68298;
    positions[4] = 1.00218;
    positions[5] = -0.793014;
    positions[6] = -0.229461;
    positions[7] = -0.995584;
    positions[8] = 1.06078;
    positions[9] = -0.746608;
    positions[10] = -1.49875;
    positions[11] = -1.1598;
    positions[12] = -1.7317;
    positions[13] = 0.315568;
    //    psm->getPlanningScene()->getCurrentState().getJointStateGroup("arms")->setStateValues(positions);


    offer_tray[0] =  -0.056477;
    offer_tray[1] =  0.206485;
    offer_tray[2] =  2.70591;
    offer_tray[3] =  -0.238801;
    offer_tray[4] =  -1.07001;
    offer_tray[5] =  -1.6267;
    offer_tray[6] =  0.426203;
    offer_tray[7] =  -0.872557;
    offer_tray[8] =  0.27542;
    offer_tray[9] =  -1.75356;
    offer_tray[10] =  -0.789074;
    offer_tray[11] =  -0.0152482;
    offer_tray[12] =  -1.39915;
    offer_tray[13] =  -0.321267;
    psm->getPlanningScene()->getCurrentState().getJointStateGroup("arms")->setStateValues(offer_tray);

    left_side[0] =  2.13008;
    left_side[1] =  0.893423;
    left_side[2] =  1.49479;
    left_side[3] =  -1.44857;
    left_side[4] =  0.898135;    
    left_side[5] =  -1.55074;    
    left_side[6] =  0.0402895;
    left_side[7] =  0.495095;
    left_side[8] =  0.0952849;
    left_side[9] =  -2.93666;
    left_side[10] =  -1.06755;
    left_side[11] =  1.26707;
    left_side[12] =  -1.48356;
    left_side[13] =  -1.12132;
    
    ros::WallDuration(2.0).sleep();
    
    moveit_msgs::PlanningScene psmsg;
    psm->getPlanningScene()->getPlanningSceneMsg(psmsg);
    pub_scene.publish(psmsg);
    ROS_INFO("Scene published.");
    
    ros::WallDuration(2.0).sleep();
}

void benchmarkPathConstrained(const std::string &config, double offset)
{
    ros::NodeHandle nh;
    ros::service::waitForService(BENCHMARK_SERVICE_NAME);
    ros::ServiceClient benchmark_service_client = nh.serviceClient<moveit_msgs::ComputePlanningBenchmark>(BENCHMARK_SERVICE_NAME);
    
    moveit_msgs::ComputePlanningBenchmark::Request mplan_req;
    moveit_msgs::ComputePlanningBenchmark::Response mplan_res;
    
    mplan_req.average_count = 50;
    mplan_req.motion_plan_request.planner_id = config;
    mplan_req.motion_plan_request.group_name = "arms"; 
    
    mplan_req.motion_plan_request.allowed_planning_time = ros::Duration(15.0);


    const std::vector<std::string>& joint_names = psm->getPlanningScene()->getKinematicModel()->getJointModelGroup("arms")->getJointModelNames();
    mplan_req.motion_plan_request.goal_constraints.resize(1);
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints.resize(joint_names.size());
    
    for(unsigned int i = 0; i < joint_names.size(); i++)
    {
	mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].joint_name = joint_names[i];
	mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].position = offer_tray[i];
	mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].tolerance_above = 1e-12;
	mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].tolerance_below = 1e-12;
	mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].weight = 1.0;
    }    
    
    mplan_req.motion_plan_request.start_state.joint_state.name = joint_names;
    mplan_req.motion_plan_request.start_state.joint_state.position = left_side;
    
    
    // add path constraints
    mplan_req.motion_plan_request.path_constraints = getDualArmConstraints(offset);
    
    benchmark_service_client.call(mplan_req, mplan_res);
}

void runExp(double offset)
{
  benchmarkPathConstrained("SBLkConfigDefault", offset);
  //    benchmarkPathConstrained("ESTkConfigDefault");
  //    benchmarkPathConstrained("BKPIECEkConfigDefault");
  
  benchmarkPathConstrained("LBKPIECEkConfigDefault", offset);
  benchmarkPathConstrained("KPIECEkConfigDefault", offset);
  benchmarkPathConstrained("RRTkConfigDefault", offset);
  benchmarkPathConstrained("RRTConnectkConfigDefault", offset);
}

void testPlan(double offset)
{    
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<moveit_msgs::DisplayTrajectory>("display_motion_plan", 1);
  ros::service::waitForService(PLANNING_SERVICE_NAME);    

    ros::ServiceClient service_client = nh.serviceClient<moveit_msgs::GetMotionPlan>(PLANNING_SERVICE_NAME);
    
    moveit_msgs::GetMotionPlan::Request mplan_req;
    moveit_msgs::GetMotionPlan::Response mplan_res;
    
    mplan_req.motion_plan_request.planner_id = "RRTConnectkConfigDefault";
    mplan_req.motion_plan_request.group_name = "arms";     
    mplan_req.motion_plan_request.allowed_planning_time = ros::Duration(15.0);
    mplan_req.motion_plan_request.random_valid_start_goal = true;
        
    const std::vector<std::string>& joint_names = psm->getPlanningScene()->getKinematicModel()->getJointModelGroup("arms")->getJointModelNames();
    mplan_req.motion_plan_request.goal_constraints.resize(1);
    mplan_req.motion_plan_request.goal_constraints[0].joint_constraints.resize(joint_names.size());
    
    for(unsigned int i = 0; i < joint_names.size(); i++)
    {
	mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].joint_name = joint_names[i];
	mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].position = offer_tray[i];
	mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].tolerance_above = 1e-12;
	mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].tolerance_below = 1e-12;
	mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].weight = 1.0;
    }    
    
    mplan_req.motion_plan_request.start_state.joint_state.name = joint_names;
    mplan_req.motion_plan_request.start_state.joint_state.position = left_side;
    
    
    // add path constraintsx
    mplan_req.motion_plan_request.path_constraints = getDualArmConstraints(offset);
    
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

bool orderFn(const ompl::base::State *s1, const ompl::base::State *s2)
{
  const ompl::base::SE3StateSpace::StateType *se3_11 = s1->as<ompl::base::CompoundState>()->components[0]->as<ompl::base::CompoundState>()->components[1]->as<ompl::base::SE3StateSpace::StateType>();
  const ompl::base::SE3StateSpace::StateType *se3_12 = s1->as<ompl::base::CompoundState>()->components[1]->as<ompl::base::CompoundState>()->components[1]->as<ompl::base::SE3StateSpace::StateType>();
  
  const ompl::base::SE3StateSpace::StateType *se3_21 = s2->as<ompl::base::CompoundState>()->components[0]->as<ompl::base::CompoundState>()->components[1]->as<ompl::base::SE3StateSpace::StateType>();
  const ompl::base::SE3StateSpace::StateType *se3_22 = s2->as<ompl::base::CompoundState>()->components[1]->as<ompl::base::CompoundState>()->components[1]->as<ompl::base::SE3StateSpace::StateType>();
  
  double dx = se3_11->getX() - se3_12->getX();
  double dy = se3_11->getY() - se3_12->getY();
  double dz = se3_11->getZ() - se3_12->getZ();
  double d1 = sqrt(dx*dx  + dy*dy + dz*dz);
  dx = se3_21->getX() - se3_22->getX();
  dy = se3_21->getY() - se3_22->getY();
  dz = se3_21->getZ() - se3_22->getZ();
  double d2 = sqrt(dx*dx  + dy*dy + dz*dz);
  return d1 > d2;
}

void computeDB(double offset, int ns, int ne)
{
    ompl_interface_ros::OMPLInterfaceROS ompl_interface(psm->getPlanningScene()->getKinematicModel());
    moveit_msgs::Constraints c = getDualArmConstraints(offset);
    ompl_interface.addConstraintApproximation(c, "arms", "PoseModel", psm->getPlanningScene()->getCurrentState(), 
                                              boost::bind(&orderFn, _1, _2), ns, ne);
    ompl_interface.saveConstraintApproximations("/home/isucan/c/");
    ROS_INFO("Done");
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "iros_2012_exp", ros::init_options::AnonymousName);
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    setupEnv();    

    double offset = 0.7;
    if (argc == 3)
    {
      int ns = atoi(argv[1]);
      int ne = atoi(argv[2]);
      computeDB(offset, ns, ne);
    }
    else
    {
      testPlan(offset); //runExp(offset);
    }
    
    return 0;
}
