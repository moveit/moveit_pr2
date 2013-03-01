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

/** \author E. Gil Jones */

#include <gtest/gtest.h>
#include <urdf_parser/urdf_parser.h>
#include <sbpl_interface/sbpl_meta_interface.h>
#include <ompl_interface_ros/ompl_interface_ros.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <kinematic_constraints/utils.h>
#include <planning_models/conversions.h>
#include <collision_distance_field/collision_world_hybrid.h>
#include <collision_distance_field_ros/collision_robot_hybrid_ros.h>
#include <rdf_loader/rdf_loader.h>
#include <ompl_interface_ros/ompl_interface_ros.h>
  
class Pr2SBPLOMPLPlannerTester : public testing::Test{

protected:

  virtual void SetUp() 
  {
    
    rml_.reset(new rdf_loader::RDFLoader("robot_description"));

    acm_.reset(new collision_detection::AllowedCollisionMatrix());

    ros::NodeHandle nh;
    
    XmlRpc::XmlRpcValue coll_ops;
    nh.getParam("robot_description_planning/default_collision_operations", coll_ops);
    
    if (coll_ops.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_WARN("default_collision_operations is not an array");
      return;
    }
    
    if (coll_ops.size() == 0)
    {
      ROS_WARN("No collision operations in default collision operations");
      return;
    }
    
    for (int i = 0 ; i < coll_ops.size() ; ++i)
    {
      if (!coll_ops[i].hasMember("object1") || !coll_ops[i].hasMember("object2") || !coll_ops[i].hasMember("operation"))
      {
        ROS_WARN("All collision operations must have two objects and an operation");
        continue;
      }
      acm_->setEntry(std::string(coll_ops[i]["object1"]), std::string(coll_ops[i]["object2"]), std::string(coll_ops[i]["operation"]) == "disable");
    }
  }

  virtual void TearDown()
  {

  }

protected:

  boost::shared_ptr<rdf_loader::RDFLoader> rml_;
  collision_detection::AllowedCollisionMatrixPtr acm_;

};

TEST_F(Pr2SBPLOMPLPlannerTester, SimplePlan) 
{
  planning_scene::PlanningScenePtr ps(new planning_scene::PlanningScene());
  ps->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorHybridROS::create());
  ps->configure(rml_->getURDF(), rml_->getSRDF());
  ASSERT_TRUE(ps->isConfigured());
  ps->getAllowedCollisionMatrixNonConst() = *acm_;
  
  sbpl_interface::SBPLMetaInterface sbpl_planner(ps->getRobotModel());
  ompl_interface_ros::OMPLInterfaceROS ompl_planner(ps->getRobotModel());

  moveit_msgs::GetMotionPlan::Request mplan_req;
  moveit_msgs::GetMotionPlan::Response mplan_res_sbpl;
  mplan_req.motion_plan_request.group_name = "right_arm";
  mplan_req.motion_plan_request.num_planning_attempts = 5;
  mplan_req.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  const std::vector<std::string>& joint_names = ps->getRobotModel()->getJointModelGroup("right_arm")->getJointModelNames();
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
  mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[0].position = -2.0;
  mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[3].position = -.2;
  mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[5].position = -.2;

  ros::WallTime before_sbpl = ros::WallTime::now();
  sbpl_planner.solve(ps,
                     mplan_req,
                     mplan_res_sbpl);
  std::cerr << "sbpl took " << (ros::WallTime::now()-before_sbpl) << std::endl;

  ASSERT_EQ(mplan_res_sbpl.error_code.val, mplan_res_sbpl.error_code.SUCCESS);
  EXPECT_GT(mplan_res_sbpl.trajectory.joint_trajectory.points.size(), 0);

  moveit_msgs::GetMotionPlan::Response mplan_res_ompl;
  ros::WallTime before_ompl = ros::WallTime::now();
  ompl_planner.solve(ps,
                     mplan_req,
                     mplan_res_ompl);
  std::cerr << "ompl took " << (ros::WallTime::now()-before_ompl) << std::endl;

  ASSERT_EQ(mplan_res_ompl.error_code.val, mplan_res_ompl.error_code.SUCCESS);
  EXPECT_GT(mplan_res_ompl.trajectory.joint_trajectory.points.size(), 0);
}  

static const unsigned int NUM_TRIALS = 10;

TEST_F(Pr2SBPLOMPLPlannerTester, ManyPlan) 
{
  planning_scene::PlanningScenePtr ps(new planning_scene::PlanningScene());
  ps->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorHybridROS::create());
  ps->configure(rml_->getURDF(), rml_->getSRDF());
  ASSERT_TRUE(ps->isConfigured());
  ps->getAllowedCollisionMatrixNonConst() = *acm_;

  planning_models::RobotState *::JointStateGroup* start_jsg = ps->getCurrentState().getJointStateGroup("right_arm");
  planning_models::RobotState *goal_state(ps->getCurrentState());
  planning_models::RobotState *::JointStateGroup* goal_jsg = goal_state.getJointStateGroup("right_arm");
  
  sbpl_interface::SBPLMetaInterface sbpl_planner(ps->getRobotModel());
  ompl_interface_ros::OMPLInterfaceROS ompl_planner(ps->getRobotModel());

  moveit_msgs::GetMotionPlan::Request mplan_req;
  moveit_msgs::GetMotionPlan::Response mplan_res_sbpl;
  moveit_msgs::GetMotionPlan::Response mplan_res_ompl;
  mplan_req.motion_plan_request.group_name = "right_arm";
  mplan_req.motion_plan_request.num_planning_attempts = 5;
  mplan_req.motion_plan_request.allowed_planning_time = ros::Duration(5.0);
  const std::vector<std::string>& joint_names = ps->getRobotModel()->getJointModelGroup("right_arm")->getJointModelNames();
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

  unsigned int comp_trials = 0;
  unsigned int succ_trials = 0;
  double max_planning_time = 0.0;
  double total_planning_time = 0.0;
  double max_sbpl_solve_time = 0.0;
  double total_sbpl_solve_time = 0.0;
  double max_ompl_solve_time = 0.0;
  double total_ompl_solve_time = 0.0;
  while(comp_trials < NUM_TRIALS) {
    while(1) {
      start_jsg->setToRandomValues();
      goal_jsg->setToRandomValues();
      std::vector<double> goal_vals;
      goal_jsg->getGroupStateValues(goal_vals);
      for(unsigned int i = 0; i < joint_names.size(); i++) {
        mplan_req.motion_plan_request.goal_constraints[0].joint_constraints[i].position = goal_vals[i];
      }
      std::vector<double> start_vals;
      start_jsg->getGroupStateValues(start_vals);
      for(unsigned int i = 0; i < start_vals.size(); i++) {
        std::cerr << "Start joint " << i << " val " << start_vals[i] << std::endl;
      }
      for(unsigned int i = 0; i < goal_vals.size(); i++) {
        std::cerr << "Goal joint " << i << " val " << goal_vals[i] << std::endl;
      }
      moveit_msgs::GetMotionPlan::Response mplan_res;
      ros::WallTime before_sbpl = ros::WallTime::now();
      if(sbpl_planner.solve(ps,
                            mplan_req,
                            mplan_res)) {
        double st = (ros::WallTime::now()-before_sbpl).toSec();
        if(st > max_sbpl_solve_time) {
          max_sbpl_solve_time = st;
        }
        total_sbpl_solve_time += st;
        comp_trials++;
        succ_trials++;
        if(sbpl_planner.getLastPlanningStatistics().total_planning_time_.toSec() > max_planning_time) {
          max_planning_time = sbpl_planner.getLastPlanningStatistics().total_planning_time_.toSec();
        }
        ASSERT_LT(sbpl_planner.getLastPlanningStatistics().total_planning_time_.toSec(), 5.0);
        total_planning_time += sbpl_planner.getLastPlanningStatistics().total_planning_time_.toSec();
        ros::WallTime before_ompl = ros::WallTime::now();
        if(ompl_planner.solve(ps,
                              mplan_req,
                              mplan_res_ompl)) {
          double st = (ros::WallTime::now()-before_sbpl).toSec();
          if(st > max_sbpl_solve_time) {
            max_ompl_solve_time = st;
          }
          total_ompl_solve_time += st;
        } else {
          std::cerr << "ompl has a problem" << std::endl;
        }
        break;
      } else {
        if(mplan_res.error_code.val != moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION &&
           mplan_res.error_code.val != moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION) {
          std::cerr << "Bad error code " << mplan_res.error_code.val << std::endl;
          comp_trials++;
          break;
        } else {
          //std::cerr << "Something in collision" << std::endl;
        }
      }
    }
  }

  std::cerr << "Average sbpl time " << total_sbpl_solve_time/(comp_trials*1.0) << " max " << max_sbpl_solve_time << std::endl;
  std::cerr << "Average ompl time " << total_ompl_solve_time/(comp_trials*1.0) << " max " << max_ompl_solve_time << std::endl;

  std::cerr << "Average planning time " << total_planning_time/(comp_trials*1.0) << " max " << max_planning_time << std::endl;
  EXPECT_EQ(succ_trials, comp_trials);

  //ASSERT_EQ(mplan_res.error_code.val, mplan_res.error_code.SUCCESS);
  //EXPECT_GT(mplan_res.trajectory.joint_trajectory.points.size(), 0);
}  

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_sbpl_planning");

  return RUN_ALL_TESTS();
}

