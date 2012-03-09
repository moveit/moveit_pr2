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
std::vector<double> goal_0_6(14);
std::vector<double> start_0_6(14);
std::vector<double> goal_0_5(14);
std::vector<double> start_0_5(14);
std::vector<double> goal_0_8(14);
std::vector<double> start_0_8(14);

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

    t = Eigen::Translation3d(0.3, -0.65, 0.65);
    scene->getCollisionWorld()->addToObject("tablex", new shapes::Box(0.7, 0.6, 0.04), t);

    t = Eigen::Translation3d(0.9, -0.2, 0.31);
    scene->getCollisionWorld()->addToObject("support", new shapes::Box(0.08, 0.08, 0.61), t);

    t = Eigen::Translation3d(0.65, 0.3, 0.72);
    scene->getCollisionWorld()->addToObject("object1", new shapes::Box(0.04, 0.04, 0.14), t);

    t = Eigen::Translation3d(0.75, 0.3, 0.72);
    scene->getCollisionWorld()->addToObject("object2", new shapes::Box(0.04, 0.04, 0.14), t);

    t = Eigen::Translation3d(0.4, -0.4, 0.8);
    scene->getCollisionWorld()->addToObject("object3", new shapes::Box(0.05, 0.05, 0.25), t);

    std_msgs::ColorRGBA c;
    c.r = 0.9f;
    c.g = 0.0f;
    c.b = 0.0f;
    scene->setColor("object1", c);
    scene->setColor("object2", c);
    scene->setColor("object3", c);

    c.r = 0.1f;
    c.g = 0.8f;
    c.b = 0.3f;
    scene->setColor("table", c);  
    scene->setColor("tablex", c);
    scene->setColor("support", c);

    c.r = 0.1f;
    c.g = 0.2f;
    c.b = 0.9f;
    scene->setColor("attached", c);

    // add an attached object
    moveit_msgs::AttachedCollisionObject aco;
    aco.link_name = "r_wrist_roll_link";
    aco.touch_links.push_back("r_wrist_roll_link");
    aco.touch_links.push_back("r_wrist_flex_link");
    aco.touch_links.push_back("l_wrist_roll_link");
    aco.touch_links.push_back("l_wrist_flex_link");
    aco.touch_links.push_back("r_gripper_l_finger_tip_link");
    aco.touch_links.push_back("r_gripper_r_finger_tip_link");
    aco.touch_links.push_back("l_gripper_l_finger_tip_link");
    aco.touch_links.push_back("l_gripper_r_finger_tip_link");
    aco.touch_links.push_back("r_gripper_l_finger_link");
    aco.touch_links.push_back("r_gripper_r_finger_link");
    aco.touch_links.push_back("l_gripper_l_finger_link");
    aco.touch_links.push_back("l_gripper_r_finger_link");
    aco.touch_links.push_back("l_gripper_palm_link");
    aco.touch_links.push_back("r_gripper_palm_link");
    aco.touch_links.push_back("r_motor_accelerometer_link");
    aco.touch_links.push_back("l_motor_accelerometer_link");
    
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

    start_0_6[0] = 1.34156;
    start_0_6[1] = 1.01507;
    start_0_6[2] = 1.01421;
    start_0_6[3] = -0.956862;
    start_0_6[4] = 1.14058;
    start_0_6[5] = -1.7723;
    start_0_6[6] = 0.274768;
    start_0_6[7] = 0.543448;
    start_0_6[8] = 1.08639;
    start_0_6[9] = 0.161299;
    start_0_6[10] = -0.862036;
    start_0_6[11] = -1.62438;
    start_0_6[12] = -1.48133;
    start_0_6[13] = -0.230606;

    goal_0_6[0] = 0.307431;
    goal_0_6[1] = -0.350818;
    goal_0_6[2] = 1.48688;
    goal_0_6[3] = -1.96938;
    goal_0_6[4] = 2.90038;
    goal_0_6[5] = -0.488208;
    goal_0_6[6] = 3.00527;
    goal_0_6[7] = -0.810138;
    goal_0_6[8] = -0.35244;
    goal_0_6[9] = -2.00063;
    goal_0_6[10] = -0.873304;
    goal_0_6[11] = 0.516566;
    goal_0_6[12] = -1.99173;
    goal_0_6[13] = -0.073988;
    
    start_0_5[0] = 2.1319;
    start_0_5[1] = 1.00411;
    start_0_5[2] = 2.15505;
    start_0_5[3] = -1.68104;
    start_0_5[4] = 0.91095;
    start_0_5[5] = -0.799175;
    start_0_5[6] = 0.269671;
    start_0_5[7] = 0.094083;
    start_0_5[8] = 0.925437;
    start_0_5[9] = -1.20162;
    start_0_5[10] = -0.959545;
    start_0_5[11] = -0.875904;
    start_0_5[12] = -1.365;
    start_0_5[13] = -0.280029;
    
    goal_0_5[0] = 0.200434;
    goal_0_5[1] = -0.352736;
    goal_0_5[2] = 1.91295;
    goal_0_5[3] = -1.26381;
    goal_0_5[4] = -0.752352;
    goal_0_5[5] = -0.603635;
    goal_0_5[6] = 0.353072;
    goal_0_5[7] = -0.734753;
    goal_0_5[8] = -0.35236;
    goal_0_5[9] = -2.06899;
    goal_0_5[10] = -1.00025;
    goal_0_5[11] = 0.583209;
    goal_0_5[12] = -1.49615;
    goal_0_5[13] = -0.184431;
    


    start_0_8[0] =  0.745964;
    start_0_8[1] =  1.27262;
    start_0_8[2] =  -0.032833;
    start_0_8[3] =  -0.984991;
    start_0_8[4] =  1.72441;
    start_0_8[5] =  -1.99906;
    start_0_8[6] =  0.319863;
    start_0_8[7] =  -0.110486;
    start_0_8[8] =  0.583719;
    start_0_8[9] =  -2.70953;
    start_0_8[10] =  -0.362025;
    start_0_8[11] =  0.828837;
    start_0_8[12] =  -1.6888;
    start_0_8[13] =  -0.902069;
    
    goal_0_8[0] =  -0.0223649;
    goal_0_8[1] =  0.0453215;
    goal_0_8[2] =  0.929686;
    goal_0_8[3] =  -0.383853;
    goal_0_8[4] =  0.626532;
    goal_0_8[5] =  -1.49834;
    goal_0_8[6] =  -0.174885;
    goal_0_8[7] =  -0.731765;
    goal_0_8[8] =  -0.353563;
    goal_0_8[9] =  -3.08874;
    goal_0_8[10] =  -0.651675;
    goal_0_8[11] =  1.37769;
    goal_0_8[12] =  -1.98146;
    goal_0_8[13] =  -0.338797;



    psm->getPlanningScene()->getCurrentState().getJointStateGroup("arms")->setStateValues(offer_tray);

    ros::WallDuration(1.0).sleep();
    
    moveit_msgs::PlanningScene psmsg;
    psm->getPlanningScene()->getPlanningSceneMsg(psmsg);
    pub_scene.publish(psmsg);
    ROS_INFO("Scene published.");
    
    ros::WallDuration(1.0).sleep();
}

void benchmarkPathConstrained(const std::string &config, double offset)
{
    ros::NodeHandle nh;
    ros::service::waitForService(BENCHMARK_SERVICE_NAME);
    ros::ServiceClient benchmark_service_client = nh.serviceClient<moveit_msgs::ComputePlanningBenchmark>(BENCHMARK_SERVICE_NAME);
    
    moveit_msgs::ComputePlanningBenchmark::Request mplan_req;
    moveit_msgs::ComputePlanningBenchmark::Response mplan_res;
    
    mplan_req.average_count = 100;
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
    //    mplan_req.motion_plan_request.random_valid_start_goal = true;
        
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

    //    mplan_req.motion_plan_request.goal_constraints[0] = getDualArmConstraints(offset);
    
    // add path constraints
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
    //    ompl_interface.addConstraintApproximation(c, "arms", "PoseModel", psm->getPlanningScene()->getCurrentState(), 
    //                                              boost::bind(&orderFn, _1, _2), ns, ne);
    ompl_interface.addConstraintApproximation(c, "arms", "PoseModel", psm->getPlanningScene()->getCurrentState(), 
                                              ns, ne);
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
      testPlan(offset);  runExp(offset);
    }
    
    return 0;
}
