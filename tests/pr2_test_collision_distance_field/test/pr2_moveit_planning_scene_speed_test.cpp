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
#include <sstream>
#include <algorithm>
#include <ctype.h>
#include <planning_models/robot_model.h>
#include <planning_models/kinematic_state.h>
#include <planning_models/transforms.h>
#include <geometric_shapes/shape_operations.h>
#include <planning_scene_monitor/planning_scene_monitor.h>
#include <collision_distance_field_ros/collision_robot_hybrid_ros.h>
#include <collision_distance_field/collision_world_hybrid.h>

#include <boost/filesystem.hpp>
#include <ros/console.h>

static const unsigned int TRIALS = 10000;

class Pr2DistanceFieldPlanningSceneTester : public testing::Test{

protected:

  virtual void SetUp() 
  {
    
    robot_model_loader_.reset(new robot_model_loader::RDFLoader("robot_description"));

    planning_scene::PlanningScenePtr ps(new planning_scene::PlanningScene());
    ps->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorHybridROS::create());
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(ps, robot_model_loader_));
  }

  virtual void TearDown()
  {

  }

protected:

  boost::shared_ptr<robot_model_loader::RDFLoader> robot_model_loader_;
  boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
};

TEST_F(Pr2DistanceFieldPlanningSceneTester, SpeedTestSlow)
{
  planning_models::RobotState *kstate(planning_scene_monitor_->getPlanningScene()->getCurrentState());
  planning_models::RobotState *::JointStateGroup* jsg = kstate.getJointStateGroup("right_arm");

  Eigen::Affine3d id = Eigen::Affine3d::Identity();
  id.translation().x() = .5;
  id.translation().z() = .7;
  planning_scene_monitor_->getPlanningScene()->getWorldNonConst()->addToObject("sphere", shapes::ShapeConstPtr(new shapes::Sphere(0.4)), id);

  const collision_detection::CollisionWorldHybrid* hy_world 
    = dynamic_cast<const collision_detection::CollisionWorldHybrid*>(planning_scene_monitor_->getPlanningScene()->getCollisionWorld().get());
  ASSERT_TRUE(hy_world);

  const collision_detection::CollisionRobotHybrid* hy_robot
    = dynamic_cast<const collision_detection::CollisionRobotHybrid*>(planning_scene_monitor_->getPlanningScene()->getCollisionRobot().get());
  ASSERT_TRUE(hy_robot);

  ros::WallDuration total_speed_df;
  ros::WallDuration total_speed_fcl;

  unsigned int in_collision_df = 0;
  unsigned int in_collision_fcl = 0;
  unsigned int fcl_in_coll_df_not = 0;
  
  collision_detection::CollisionRequest req;
  req.group_name = "right_arm";
  req.contacts = true;

  collision_detection::CollisionResult res1;
  //first check with this group doesn't count
  hy_world->checkCollisionDistanceField(req, 
                                        res1,
                                        *hy_robot->getCollisionRobotDistanceField().get(), 
                                        kstate,
                                        planning_scene_monitor_->getPlanningScene()->getAllowedCollisionMatrix());
  planning_scene_monitor_->getPlanningScene()->checkCollision(req, 
                                                              res1, 
                                                              kstate,
                                                              planning_scene_monitor_->getPlanningScene()->getAllowedCollisionMatrix());  
  
  for(unsigned int i = 0; i < TRIALS; i++) {
    jsg->setToRandomValues();
    collision_detection::CollisionResult res;
    ros::WallTime before = ros::WallTime::now();
    hy_world->checkCollisionDistanceField(req, 
                                          res, 
                                          *hy_robot->getCollisionRobotDistanceField().get(), 
                                          kstate,
                                          planning_scene_monitor_->getPlanningScene()->getAllowedCollisionMatrix());
    total_speed_df += (ros::WallTime::now()-before);
    bool df_in_collision = false;
    if(res.collision) {
      in_collision_df++;
      df_in_collision = true;
    }
    res = collision_detection::CollisionResult();
    before = ros::WallTime::now();
    planning_scene_monitor_->getPlanningScene()->checkCollision(req, 
                                                                res, 
                                                                kstate,
                                                                planning_scene_monitor_->getPlanningScene()->getAllowedCollisionMatrix());  

    total_speed_fcl += (ros::WallTime::now()-before);
    if(res.collision) {
      in_collision_fcl++;
      if(!df_in_collision) {
        fcl_in_coll_df_not++;
      }
    }
  }
  std::cerr << "Average time df " << total_speed_df.toSec()/(1.0*TRIALS) << " hz " << (1.0/(total_speed_df.toSec()/(1.0*TRIALS))) << std::endl;
  std::cerr << "In collision df " << in_collision_df << std::endl;
  std::cerr << "Average time fcl " << total_speed_fcl.toSec()/(1.0*TRIALS) << " hz " << (1.0/(total_speed_fcl.toSec()/(1.0*TRIALS))) << std::endl;
  std::cerr << "In collision fcl " << in_collision_fcl << std::endl;
  std::cerr << "Fcl in collision df not " << fcl_in_coll_df_not << std::endl;
}

TEST_F(Pr2DistanceFieldPlanningSceneTester, SpeedTestFast)
{
  planning_models::RobotState *kstate(planning_scene_monitor_->getPlanningScene()->getCurrentState());
  planning_models::RobotState *::JointStateGroup* jsg = kstate.getJointStateGroup("right_arm");

  Eigen::Affine3d id = Eigen::Affine3d::Identity();
  id.translation().x() = .5;
  id.translation().z() = .7;
  planning_scene_monitor_->getPlanningScene()->getWorldNonConst()->addToObject("sphere", shapes::ShapeConstPtr(new shapes::Sphere(0.4)), id);

  const collision_detection::CollisionWorldHybrid* hy_world 
    = dynamic_cast<const collision_detection::CollisionWorldHybrid*>(planning_scene_monitor_->getPlanningScene()->getCollisionWorld().get());
  ASSERT_TRUE(hy_world);

  const collision_detection::CollisionRobotHybrid* hy_robot
    = dynamic_cast<const collision_detection::CollisionRobotHybrid*>(planning_scene_monitor_->getPlanningScene()->getCollisionRobot().get());
  ASSERT_TRUE(hy_robot);

  ros::WallDuration total_speed_df;
  ros::WallDuration total_speed_fcl;

  unsigned int in_collision_df = 0;
  unsigned int in_collision_fcl = 0;
  unsigned int fcl_in_coll_df_not = 0;
  
  collision_detection::CollisionRequest req;
  req.group_name = "right_arm";
  req.contacts = true;

  collision_detection::CollisionResult res1;
  //first check with this group doesn't count
  boost::shared_ptr<collision_detection::GroupStateRepresentation> gsr; 
  hy_world->checkCollisionDistanceField(req, 
                                        res1,
                                        *hy_robot->getCollisionRobotDistanceField().get(), 
                                        kstate,
                                        planning_scene_monitor_->getPlanningScene()->getAllowedCollisionMatrix(),
                                        gsr);
  planning_scene_monitor_->getPlanningScene()->checkCollision(req, 
                                                              res1, 
                                                              kstate,
                                                              planning_scene_monitor_->getPlanningScene()->getAllowedCollisionMatrix());  
  
  for(unsigned int i = 0; i < TRIALS; i++) {
    jsg->setToRandomValues();
    collision_detection::CollisionResult res;
    ros::WallTime before = ros::WallTime::now();
    hy_world->checkCollisionDistanceField(req, 
                                          res, 
                                          *hy_robot->getCollisionRobotDistanceField().get(), 
                                          kstate,
                                          gsr);
    ros::WallDuration dur(ros::WallTime::now()-before);
    total_speed_df += dur;
    //std::cerr << "Took " << dur << std::endl;
    bool df_in_collision = false;
    if(res.collision) {
      in_collision_df++;
      df_in_collision = true;
    }
    res = collision_detection::CollisionResult();
    before = ros::WallTime::now();
    planning_scene_monitor_->getPlanningScene()->checkCollision(req, 
                                                                res, 
                                                                kstate,
                                                                planning_scene_monitor_->getPlanningScene()->getAllowedCollisionMatrix());  

    total_speed_fcl += (ros::WallTime::now()-before);
    if(res.collision) {
      in_collision_fcl++;
      if(!df_in_collision) {
        fcl_in_coll_df_not++;
      }
    }
  }
  std::cerr << "Average time df " << total_speed_df.toSec()/(1.0*TRIALS) << " hz " << (1.0/(total_speed_df.toSec()/(1.0*TRIALS))) << std::endl;
  std::cerr << "In collision df " << in_collision_df << std::endl;
  std::cerr << "Average time fcl " << total_speed_fcl.toSec()/(1.0*TRIALS) << " hz " << (1.0/(total_speed_fcl.toSec()/(1.0*TRIALS))) << std::endl;
  std::cerr << "In collision fcl " << in_collision_fcl << std::endl;
  std::cerr << "Fcl in collision df not " << fcl_in_coll_df_not << std::endl;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pr2_moveit_planning_scene_speed_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
