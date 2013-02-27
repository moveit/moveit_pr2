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
#include <chomp_motion_planner/chomp_planner.h>
#include <chomp_motion_planner/chomp_parameters.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <kinematic_constraints/utils.h>
#include <planning_models/conversions.h>
#include <collision_distance_field/collision_detector_allocator_distance_field.h>
#include <rdf_loader/rdf_loader.h>

class Pr2ChompPlannerTester : public testing::Test{

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

TEST_F(Pr2ChompPlannerTester, SimplePlan) 
{
  planning_scene::PlanningScenePtr ps(new planning_scene::PlanningScene());
  ps->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorDistanceField::create());
  ps->configure(rml_->getURDF(), rml_->getSRDF());
  ASSERT_TRUE(ps->isConfigured());
  ps->getAllowedCollisionMatrixNonConst() = *acm_;
  
  chomp::ChompPlanner chomp_planner(ps->getRobotModel());

  moveit_msgs::GetMotionPlan::Request mplan_req;
  moveit_msgs::GetMotionPlan::Response mplan_res;
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
  
  chomp::ChompParameters params;
  chomp_planner.solve(ps,
                      mplan_req,
                      params,
                      mplan_res);

  ASSERT_EQ(mplan_res.error_code.val, mplan_res.error_code.SUCCESS);
  EXPECT_GT(mplan_res.trajectory.joint_trajectory.points.size(), 0);
}  

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_chomp_planning");

  return RUN_ALL_TESTS();
}

