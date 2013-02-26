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

#include <planning_models/robot_model.h>
#include <planning_models/kinematic_state.h>
#include <gtest/gtest.h>
#include <sstream>
#include <algorithm>
#include <ctype.h>
#include <planning_models/robot_model.h>
#include <planning_models/kinematic_state.h>
#include <planning_models/transforms.h>
#include <geometric_shapes/shape_operations.h>
#include <collision_distance_field/collision_distance_field_types.h>
#include <collision_distance_field/collision_robot_distance_field.h>
#include <collision_distance_field/collision_world_distance_field.h>
#include <collision_distance_field_ros/collision_distance_field_ros_helpers.h>
#include <collision_detection_fcl/collision_world_fcl.h>
#include <collision_detection_fcl/collision_robot_fcl.h>
#include <rdf_loader/rdf_loader.h>

#include <boost/filesystem.hpp>
#include <ros/console.h>

static const unsigned int TRIALS = 10000;

class Pr2DistanceFieldCollisionDetectionTester : public testing::Test{

protected:

  virtual void SetUp() 
  {
    
    rml_.reset(new rdf_loader::RDFLoader("robot_description"));
    kmodel_.reset(new planning_models::RobotModel(rml_->getURDF(), rml_->getSRDF()));

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
    
    std::map<std::string, std::vector<collision_detection::CollisionSphere> > coll_spheres;
    collision_detection::loadLinkBodySphereDecompositions(nh,
                                                          kmodel_,
                                                          coll_spheres);
    
    crobot_df_.reset(new collision_detection::CollisionRobotDistanceField(kmodel_, coll_spheres));
    cworld_df_.reset(new collision_detection::CollisionWorldDistanceField());

    crobot_fcl_.reset(new collision_detection::CollisionRobotFCL(kmodel_));
    cworld_fcl_.reset(new collision_detection::CollisionWorldFCL());
  }

  virtual void TearDown()
  {

  }

protected:

  boost::shared_ptr<rdf_loader::RDFLoader> rml_;
  
  planning_models::RobotModelPtr             kmodel_;
  
  planning_models::TransformsPtr                 ftf_;
  planning_models::TransformsConstPtr            ftf_const_;
  
  boost::shared_ptr<collision_detection::CollisionRobotDistanceField> crobot_df_;
  boost::shared_ptr<collision_detection::CollisionWorldDistanceField> cworld_df_;

  boost::shared_ptr<collision_detection::CollisionRobot> crobot_fcl_;
  boost::shared_ptr<collision_detection::CollisionWorld> cworld_fcl_;

  
  collision_detection::AllowedCollisionMatrixPtr acm_;

};

TEST_F(Pr2DistanceFieldCollisionDetectionTester, SpeedTest)
{
  planning_models::RobotState *kstate(kmodel_);
  kstate.setToDefaultValues();

  planning_models::RobotState *::JointStateGroup* jsg = kstate.getJointStateGroup("right_arm");

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
  crobot_df_->checkSelfCollision(req, res1, kstate, *acm_, gsr);  
  crobot_fcl_->checkSelfCollision(req, res1, kstate, *acm_);  

  for(unsigned int i = 0; i < TRIALS; i++) {
    jsg->setToRandomValues();
    collision_detection::CollisionResult res;
    ros::WallTime before = ros::WallTime::now();
    crobot_df_->checkSelfCollision(req, res, kstate, gsr);
    total_speed_df += (ros::WallTime::now()-before);
    bool df_in_collision = false;
    if(res.collision) {
      in_collision_df++;
      df_in_collision = true;
    }
    res = collision_detection::CollisionResult();
    before = ros::WallTime::now();
    crobot_fcl_->checkSelfCollision(req, res, kstate, *acm_);
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
  ros::init(argc, argv, "pr2_moveit_cd_speed_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

