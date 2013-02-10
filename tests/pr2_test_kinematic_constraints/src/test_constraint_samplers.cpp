/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#include <planning_scene_monitor/planning_scene_monitor.h>
#include <kinematic_constraints/kinematic_constraint.h>
#include <constraint_samplers/default_constraint_samplers.h>
#include <constraint_samplers/constraint_sampler_manager.h>
#include <constraint_samplers/constraint_sampler_tools.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <planning_models/conversions.h>
#include <visualization_msgs/MarkerArray.h>

#include <ros/ros.h>
#include <gtest/gtest.h>

static const std::string ROBOT_DESCRIPTION="robot_description";

class ConstraintSamplerTestBase : public testing::Test
{
protected:
  
  virtual void SetUp()
  {
    psm_.reset(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION));
    kmodel_ = psm_->getPlanningScene()->getRobotModel();
  };
  
  virtual void TearDown()
  {
  }
  
protected:
  
  ros::NodeHandle nh_;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  planning_models::RobotModelConstPtr kmodel_;
};


TEST_F(ConstraintSamplerTestBase, JointConstraintsSampler)
{
  planning_models::RobotState *ks(kmodel_);
  ks.setToDefaultValues();
  planning_models::TransformsPtr tf = psm_->getPlanningScene()->getTransforms();
  
  kinematic_constraints::JointConstraint jc1(kmodel_, tf);
  moveit_msgs::JointConstraint jcm1;
  jcm1.joint_name = "head_pan_joint";
  jcm1.position = 0.42;
  jcm1.tolerance_above = 0.01;
  jcm1.tolerance_below = 0.05;
  jcm1.weight = 1.0;
  EXPECT_TRUE(jc1.configure(jcm1));

  kinematic_constraints::JointConstraint jc2(kmodel_, tf);
  moveit_msgs::JointConstraint jcm2;
  jcm2.joint_name = "l_shoulder_pan_joint";
  jcm2.position = 0.9;
  jcm2.tolerance_above = 0.1;
  jcm2.tolerance_below = 0.05;
  jcm2.weight = 1.0;
  EXPECT_TRUE(jc2.configure(jcm2));
  
  kinematic_constraints::JointConstraint jc3(kmodel_, tf);
  moveit_msgs::JointConstraint jcm3;
  jcm3.joint_name = "r_wrist_roll_joint";
  jcm3.position = 0.7;
  jcm3.tolerance_above = 0.14;
  jcm3.tolerance_below = 0.005;
  jcm3.weight = 1.0;
  EXPECT_TRUE(jc3.configure(jcm3));
  
  kinematic_constraints::JointConstraint jc4(kmodel_, tf);
  moveit_msgs::JointConstraint jcm4;
  jcm4.joint_name = "torso_lift_joint";
  jcm4.position = 0.2;
  jcm4.tolerance_above = 0.09;
  jcm4.tolerance_below = 0.01;
  jcm4.weight = 1.0;
  EXPECT_TRUE(jc4.configure(jcm4));

  std::vector<kinematic_constraints::JointConstraint> js;
  js.push_back(jc1);
  js.push_back(jc2);
  js.push_back(jc3);
  js.push_back(jc4);
  
  constraint_samplers::JointConstraintSampler jcs(psm_->getPlanningScene(), "arms", js);
  EXPECT_EQ(jcs.getConstrainedJointCount(), 2);
  EXPECT_EQ(jcs.getUnconstrainedJointCount(), 12);

  for (int t = 0 ; t < 1000 ; ++t)
  {
    EXPECT_TRUE(jcs.sample(ks.getJointStateGroup("arms"), ks, 1));
    EXPECT_TRUE(jc2.decide(ks).satisfied);
    EXPECT_TRUE(jc3.decide(ks).satisfied);
  }

  // test the automatic construction of constraint sampler
  moveit_msgs::Constraints c;
  
  // no constraints should give no sampler
  constraint_samplers::ConstraintSamplerPtr s0 = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(psm_->getPlanningScene(), "arms", c);
  EXPECT_TRUE(s0.get() == NULL);

  // add the constraints
  c.joint_constraints.push_back(jcm1);
  c.joint_constraints.push_back(jcm2);
  c.joint_constraints.push_back(jcm3);
  c.joint_constraints.push_back(jcm4);
  
  constraint_samplers::ConstraintSamplerPtr s = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(psm_->getPlanningScene(), "arms", c);
  EXPECT_TRUE(s.get() != NULL);
  
  // test the generated sampler
  for (int t = 0 ; t < 1000 ; ++t)
  {
    EXPECT_TRUE(s->sample(ks.getJointStateGroup("arms"), ks, 1));
    EXPECT_TRUE(jc2.decide(ks).satisfied);
    EXPECT_TRUE(jc3.decide(ks).satisfied);
  }
}

TEST_F(ConstraintSamplerTestBase, OrientationConstraintsSampler)
{
  planning_models::RobotState *ks(kmodel_);
  ks.setToDefaultValues();
  planning_models::TransformsPtr tf = psm_->getPlanningScene()->getTransforms();
  
  kinematic_constraints::OrientationConstraint oc(kmodel_, tf);
  moveit_msgs::OrientationConstraint ocm;
  
  ocm.link_name = "r_wrist_roll_link";
  ocm.header.frame_id = ocm.link_name; 
  ocm.orientation.x = 0.5;
  ocm.orientation.y = 0.5;
  ocm.orientation.z = 0.5;
  ocm.orientation.w = 0.5;
  ocm.absolute_x_axis_tolerance = 0.01;
  ocm.absolute_y_axis_tolerance = 0.01;
  ocm.absolute_z_axis_tolerance = 0.01;
  ocm.weight = 1.0;
  
  EXPECT_TRUE(oc.configure(ocm));
  
  bool p1 = oc.decide(ks).satisfied;
  EXPECT_FALSE(p1);
  
  ocm.header.frame_id = kmodel_->getModelFrame();
  EXPECT_TRUE(oc.configure(ocm));
  
  constraint_samplers::IKConstraintSampler iks(psm_->getPlanningScene(), "right_arm", constraint_samplers::IKSamplingPose(oc));
  for (int t = 0 ; t < 100 ; ++t)
  {
    EXPECT_TRUE(iks.sample(ks.getJointStateGroup("right_arm"), ks, 100));
    EXPECT_TRUE(oc.decide(ks).satisfied);
  }
  
}

TEST_F(ConstraintSamplerTestBase, PoseConstraintsSampler)
{
  planning_models::RobotState *ks(kmodel_);
  ks.setToDefaultValues();
  planning_models::TransformsPtr tf = psm_->getPlanningScene()->getTransforms();
  
  kinematic_constraints::PositionConstraint pc(kmodel_, tf);
  moveit_msgs::PositionConstraint pcm;
  
  pcm.link_name = "l_wrist_roll_link";
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0;
  pcm.constraint_region.primitives.resize(1);
  pcm.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
  pcm.constraint_region.primitives[0].dimensions.x = 0.001;
  
  pcm.header.frame_id = kmodel_->getModelFrame();

  pcm.constraint_region.primitive_poses.resize(1);
  pcm.constraint_region.primitive_poses[0].position.x = 0.55;
  pcm.constraint_region.primitive_poses[0].position.y = 0.2;
  pcm.constraint_region.primitive_poses[0].position.z = 1.25;
  pcm.constraint_region.primitive_poses[0].orientation.x = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.y = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.z = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.w = 1.0;
  pcm.weight = 1.0;
  
  EXPECT_TRUE(pc.configure(pcm));
  
  kinematic_constraints::OrientationConstraint oc(kmodel_, tf);
  moveit_msgs::OrientationConstraint ocm;
  
  ocm.link_name = "l_wrist_roll_link";
  ocm.header.frame_id = kmodel_->getModelFrame();
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.4;
  ocm.weight = 1.0;
  
  EXPECT_TRUE(oc.configure(ocm));

  constraint_samplers::IKConstraintSampler iks1(psm_->getPlanningScene(), "left_arm", constraint_samplers::IKSamplingPose(pc, oc));
  for (int t = 0 ; t < 100 ; ++t)
  {
    EXPECT_TRUE(iks1.sample(ks.getJointStateGroup("left_arm"), ks, 100));
    EXPECT_TRUE(pc.decide(ks).satisfied);
    EXPECT_TRUE(oc.decide(ks).satisfied);
  }
  
  constraint_samplers::IKConstraintSampler iks2(psm_->getPlanningScene(), "left_arm", constraint_samplers::IKSamplingPose(pc));
  for (int t = 0 ; t < 100 ; ++t)
  {
    EXPECT_TRUE(iks2.sample(ks.getJointStateGroup("left_arm"), ks, 100));
    EXPECT_TRUE(pc.decide(ks).satisfied);
  }
  
  constraint_samplers::IKConstraintSampler iks3(psm_->getPlanningScene(), "left_arm", constraint_samplers::IKSamplingPose(oc));
  for (int t = 0 ; t < 100 ; ++t)
  {
    EXPECT_TRUE(iks3.sample(ks.getJointStateGroup("left_arm"), ks, 100));
    EXPECT_TRUE(oc.decide(ks).satisfied);
  }
  
  
  // test the automatic construction of constraint sampler
  moveit_msgs::Constraints c;
  c.position_constraints.push_back(pcm);
  c.orientation_constraints.push_back(ocm);
  
  constraint_samplers::ConstraintSamplerPtr s = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(psm_->getPlanningScene(), "left_arm", c);
  EXPECT_TRUE(s.get() != NULL);
  static const int NT = 1000;
  int succ = 0;
  for (int t = 0 ; t < NT ; ++t)
  {
    EXPECT_TRUE(s->sample(ks.getJointStateGroup("left_arm"), ks, 100));
    EXPECT_TRUE(pc.decide(ks).satisfied);
    EXPECT_TRUE(oc.decide(ks).satisfied);
    if (s->sample(ks.getJointStateGroup("left_arm"), ks, 1))
      succ++;
  }
  ROS_INFO("Success rate for IK Constraint Sampler with position & orientation constraints for one arm: %lf", (double)succ / (double)NT);
}

TEST_F(ConstraintSamplerTestBase, GenericConstraintsSampler)
{
  moveit_msgs::Constraints c;
  
  moveit_msgs::PositionConstraint pcm;
  pcm.link_name = "l_wrist_roll_link";
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0;

  pcm.constraint_region.primitives.resize(1);
  pcm.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
  pcm.constraint_region.primitives[0].dimensions.x = 0.001;
  
  pcm.header.frame_id = kmodel_->getModelFrame();

  pcm.constraint_region.primitive_poses.resize(1);
  pcm.constraint_region.primitive_poses[0].position.x = 0.55;
  pcm.constraint_region.primitive_poses[0].position.y = 0.2;
  pcm.constraint_region.primitive_poses[0].position.z = 1.25;
  pcm.constraint_region.primitive_poses[0].orientation.x = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.y = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.z = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.w = 1.0;
  pcm.weight = 1.0;
  c.position_constraints.push_back(pcm);
  
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "l_wrist_roll_link";
  ocm.header.frame_id = kmodel_->getModelFrame();
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.4;
  ocm.weight = 1.0;
  c.orientation_constraints.push_back(ocm);
  
  ocm.link_name = "r_wrist_roll_link";
  ocm.header.frame_id = kmodel_->getModelFrame();
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.01;
  ocm.absolute_y_axis_tolerance = 0.01;
  ocm.absolute_z_axis_tolerance = 0.01;
  ocm.weight = 1.0;
  c.orientation_constraints.push_back(ocm);
  
  planning_models::TransformsPtr tf = psm_->getPlanningScene()->getTransforms();
  constraint_samplers::ConstraintSamplerPtr s = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(psm_->getPlanningScene(), "arms", c);
  EXPECT_TRUE(s.get() != NULL);
  
  kinematic_constraints::KinematicConstraintSet kset(kmodel_, tf);
  kset.add(c);
  
  planning_models::RobotState *ks(kmodel_);
  ks.setToDefaultValues();  
  static const int NT = 1000;
  int succ = 0;
  for (int t = 0 ; t < 1000 ; ++t)
  {
    EXPECT_TRUE(s->sample(ks.getJointStateGroup("arms"), ks, 1000));
    EXPECT_TRUE(kset.decide(ks).satisfied);
    if (s->sample(ks.getJointStateGroup("arms"), ks, 1))
      succ++;
  } 
  ROS_INFO("Success rate for IK Constraint Sampler with position & orientation constraints for both arms: %lf", (double)succ / (double)NT);
}

TEST_F(ConstraintSamplerTestBase, DisplayGenericConstraintsSamples1)
{
  moveit_msgs::Constraints c;

  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "r_wrist_roll_link";
  ocm.header.frame_id = kmodel_->getModelFrame();
  ocm.orientation.x = 0;
  ocm.orientation.y = 0;
  ocm.orientation.z = 0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.01;
  ocm.absolute_y_axis_tolerance = 0.01;
  ocm.absolute_z_axis_tolerance = M_PI;
  ocm.weight = 1.0;
  c.orientation_constraints.push_back(ocm);
  
  ros::NodeHandle nh;
  ros::Publisher pub_state = nh.advertise<moveit_msgs::DisplayTrajectory>("display_motion_plan", 20);
  
  planning_models::TransformsPtr tf = psm_->getPlanningScene()->getTransforms();
  constraint_samplers::ConstraintSamplerPtr s = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(psm_->getPlanningScene(), "right_arm", c);
  EXPECT_TRUE(s.get() != NULL);
  
  kinematic_constraints::KinematicConstraintSet kset(kmodel_, tf);
  kset.add(c);
  
  planning_models::RobotState *ks(kmodel_);
  ks.setToDefaultValues();

  ros::WallTime start = ros::WallTime::now();
  unsigned int ns = 0;
  for (int t = 0 ; t < 500 ; ++t)
  {
    if ((s->sample(ks.getJointStateGroup("right_arm"), ks, 2)))
      ns++;
  }
  ROS_INFO("%lf samples per second", ns / (ros::WallTime::now() - start).toSec());
  ros::Publisher pub_markers = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 5); 
  sleep(1);
  visualization_msgs::MarkerArray arr;  
  constraint_samplers::visualizeDistribution(s, ks, "r_wrist_roll_link", 10, 1000, arr);
  pub_markers.publish(arr);
  ros::Duration(1.0).sleep();  
}

TEST_F(ConstraintSamplerTestBase, DisplayGenericConstraintsSamples2)
{
  moveit_msgs::Constraints c;

  moveit_msgs::PositionConstraint pcm;
  pcm.link_name = "l_wrist_roll_link";
  pcm.target_point_offset.x = 0;
  pcm.target_point_offset.y = 0;
  pcm.target_point_offset.z = 0;
  pcm.header.frame_id = kmodel_->getModelFrame();

  pcm.constraint_region.primitives.resize(1);
  pcm.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  pcm.constraint_region.primitives[0].dimensions.x = 0.2;
  pcm.constraint_region.primitives[0].dimensions.y = 0.3;
  pcm.constraint_region.primitives[0].dimensions.z = 0.4;
  
  pcm.constraint_region.primitive_poses.resize(1);
  pcm.constraint_region.primitive_poses[0].position.x = 0.5;
  pcm.constraint_region.primitive_poses[0].position.y = 0.2;
  pcm.constraint_region.primitive_poses[0].position.z = 0.6;
  pcm.constraint_region.primitive_poses[0].orientation.x = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.y = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.z = 0.0;
  pcm.constraint_region.primitive_poses[0].orientation.w = 1.0;
  pcm.weight = 1.0;
  c.position_constraints.push_back(pcm);

  moveit_msgs::PositionConstraint pcm2;
  pcm2.link_name = "r_wrist_roll_link";
  pcm2.target_point_offset.x = 0.7;
  pcm2.target_point_offset.y = 0;
  pcm2.target_point_offset.z = 0; 
  pcm2.header.frame_id = "l_wrist_roll_link";
  
  pcm2.constraint_region.primitives.resize(1);
  pcm2.constraint_region.primitive_poses.resize(1);

  pcm2.constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  pcm2.constraint_region.primitives[0].dimensions.x = 0.01;
  pcm2.constraint_region.primitives[0].dimensions.y = 0.01;
  pcm2.constraint_region.primitives[0].dimensions.z = 0.01;
  
  pcm2.constraint_region.primitive_poses[0].position.x = 0.0;
  pcm2.constraint_region.primitive_poses[0].position.y = 0.0;
  pcm2.constraint_region.primitive_poses[0].position.z = 0.0;
  pcm2.constraint_region.primitive_poses[0].orientation.x = 0.0;
  pcm2.constraint_region.primitive_poses[0].orientation.y = 0.0;
  pcm2.constraint_region.primitive_poses[0].orientation.z = 0.0;
  pcm2.constraint_region.primitive_poses[0].orientation.w = 1.0;
  pcm2.weight = 1.0;
  c.position_constraints.push_back(pcm2);


  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "l_wrist_roll_link";
  ocm.header.frame_id = kmodel_->getModelFrame();
  ocm.orientation.x = 0.5;
  ocm.orientation.y = 0.5;
  ocm.orientation.z = 0.5;
  ocm.orientation.w = 0.5;
  ocm.absolute_x_axis_tolerance = 0.01;
  ocm.absolute_y_axis_tolerance = M_PI;
  ocm.absolute_z_axis_tolerance = 0.01;
  ocm.weight = 1.0;
  c.orientation_constraints.push_back(ocm);

  ocm.link_name = "r_wrist_roll_link";
  ocm.header.frame_id = "l_wrist_roll_link";
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 1.0;
  ocm.orientation.w = 0.0;
  ocm.absolute_x_axis_tolerance = 0.01;
  ocm.absolute_y_axis_tolerance = 0.01;
  ocm.absolute_z_axis_tolerance = 0.01;
  ocm.weight = 1.0;
  c.orientation_constraints.push_back(ocm);
  
  ros::NodeHandle nh;
  ros::Publisher pub_state = nh.advertise<moveit_msgs::DisplayTrajectory>("display_motion_plan", 20);
  
  planning_models::TransformsPtr tf = psm_->getPlanningScene()->getTransforms();
  constraint_samplers::ConstraintSamplerPtr s = constraint_samplers::ConstraintSamplerManager::selectDefaultSampler(psm_->getPlanningScene(), "arms", c);
  EXPECT_TRUE(s.get() != NULL);
  
  kinematic_constraints::KinematicConstraintSet kset(kmodel_, tf);
  kset.add(c);
  
  planning_models::RobotState *ks(kmodel_);
  ks.setToDefaultValues();

  ros::WallTime start = ros::WallTime::now();
  unsigned int ns = 0;
  for (int t = 0 ; t < 500 ; ++t)
  {
    if ((s->sample(ks.getJointStateGroup("arms"), ks, 2)))
      ns++;
  }
  ROS_INFO("%lf samples per second", ns / (ros::WallTime::now() - start).toSec());
  
  for (int t = 0 ; t < 100 ; ++t)
  {
    if ((s->sample(ks.getJointStateGroup("arms"), ks, 3)))
    {
      bool valid = kset.decide(ks).satisfied;
      EXPECT_TRUE(valid);
      if (valid)
      {
        moveit_msgs::DisplayTrajectory d;
        d.model_id = kmodel_->getName();
        planning_models::robotStateToRobotStateMsg(ks, d.trajectory_start);
        pub_state.publish(d);
        ros::WallDuration(1.0).sleep();
      }
    }
  }
}

/*
TEST_F(ConstraintSamplerTestBase, VisibilityConstraint)
{
  moveit_msgs::AttachedCollisionObject aco;
  aco.link_name = "r_wrist_roll_link";
  aco.touch_links.push_back("r_wrist_roll_link");
  
  moveit_msgs::CollisionObject &co = aco.object;
  co.id = "attached";
  co.header.stamp = ros::Time::now();
  co.header.frame_id = aco.link_name;
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.shapes.resize(1);
  co.shapes[0].type = shape_msgs::Shape::BOX;
  co.shapes[0].dimensions.push_back(0.3);
  co.shapes[0].dimensions.push_back(0.01);
  co.shapes[0].dimensions.push_back(0.3);
  co.poses.resize(1);
  co.poses[0].position.x = 0.28;
  co.poses[0].position.y = 0;
  co.poses[0].position.z = 0;
  co.poses[0].orientation.w = 1.0;
  psm_->getPlanningScene()->processAttachedCollisionObjectMsg(aco);

  ros::NodeHandle nh;
  planning_models::TransformsPtr tf = psm_->getPlanningScene()->getTransforms();
  
  kinematic_constraints::VisibilityConstraint vc(kmodel_, tf);
  moveit_msgs::VisibilityConstraint vcm;
  vcm.target_radius = 0.1;
  vcm.cone_sides = 12;
  vcm.max_view_angle = 0.35;
  vcm.max_range_angle = 0.35;

  vcm.target_pose.header.frame_id = aco.object.id;
  vcm.target.primitive_poses[0].position.x = 0;
  vcm.target.primitive_poses[0].position.y = 0.05;
  vcm.target.primitive_poses[0].position.z = 0;
  vcm.target.primitive_poses[0].orientation.x = sqrt(2.0)/2.0;
  vcm.target.primitive_poses[0].orientation.y = 0.0;
  vcm.target.primitive_poses[0].orientation.z = 0.0;
  vcm.target.primitive_poses[0].orientation.w = sqrt(2.0)/2.0;
  
  vcm.sensor_pose.header.frame_id = "double_stereo_link";
  vcm.sensor.primitive_poses[0].position.x = 0.1;
  vcm.sensor.primitive_poses[0].position.y = 0;
  vcm.sensor.primitive_poses[0].position.z = 0;
  vcm.sensor.primitive_poses[0].orientation.x = 0;
  vcm.sensor.primitive_poses[0].orientation.y = 0;
  vcm.sensor.primitive_poses[0].orientation.z = 0;
  vcm.sensor.primitive_poses[0].orientation.w = 1;
  vcm.weight = 1.0;
  
  planning_models::RobotState& ks = psm_->getPlanningScene()->getCurrentState();
  
  double distance;
  EXPECT_TRUE(vc.configure(vcm));
  ros::Publisher pub = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 10);
  ros::Publisher pub_scene = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 20);
  ros::WallDuration(0.5).sleep();

  for (int i = 0 ; i < 100 ; ++i)
  {   
      do
      {
	  ks.setToRandomValues();
      }
      while (!vc.decide(ks, distance));
      //      ROS_INFO("Found");
      visualization_msgs::MarkerArray markers;
      vc.getMarkers(ks, markers);
      pub.publish(markers);
      moveit_msgs::PlanningScene ps;
      psm_->getPlanningScene()->getPlanningSceneMsg(ps);
      pub_scene.publish(ps);
      
      ros::WallDuration(.01).sleep();
  }  
}
*/

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_kinematic_constraints");
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  bool result = RUN_ALL_TESTS();
  sleep(1);
  return result;
  
}
