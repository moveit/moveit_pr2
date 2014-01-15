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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Sachin Chitta */

#include <ros/ros.h>

#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// KDL
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <gtest/gtest.h>

double gen_rand(double min, double max)
{
  int rand_num = rand()%100+1;
  double result = min + (double)((max-min)*rand_num)/101.0;
  return result;
}

bool NOT_NEAR(const double &v1, const double &v2, const double &NEAR)
{
   if(fabs(v1-v2) > NEAR)
      return true;
   return false;
}

void testJacobian(std::string group_name, std::string base_link, std::string tip_link)
{
 SCOPED_TRACE(group_name + ": " + base_link + " to " + tip_link);
 
 srand ( time(NULL) ); // initialize random seed:
 rdf_loader::RDFLoader model_loader("robot_description");
 robot_model::RobotModelPtr kinematic_model;
 kinematic_model.reset(new robot_model::RobotModel(model_loader.getURDF(),model_loader.getSRDF()));

 robot_state::RobotStatePtr kinematic_state;
 kinematic_state.reset(new robot_state::RobotState(kinematic_model));
 kinematic_state->setToDefaultValues();

 const moveit::core::JointModelGroup* joint_state_group = kinematic_state->getRobotModel()->getJointModelGroup(group_name);

 std::string link_name = tip_link;
 std::vector<double> joint_angles(7,0.0);
 geometry_msgs::Point ref_position;
 Eigen::MatrixXd jacobian;
 Eigen::Vector3d point(0.0,0.0,0.0);
 kinematic_state->setJointGroupPositions(joint_state_group, &joint_angles[0]);
 ASSERT_TRUE(kinematic_state->getJacobian(joint_state_group, kinematic_state->getRobotModel()->getLinkModel(link_name),point,jacobian));

 KDL::Tree tree;
 if (!kdl_parser::treeFromUrdfModel(*model_loader.getURDF(), tree))
 {
   ROS_ERROR("Could not initialize tree object");
 }
 KDL::Chain kdl_chain;
 std::string base_frame(base_link);
 std::string tip_frame(tip_link);
 if (!tree.getChain(base_frame, tip_frame, kdl_chain))
 {
   ROS_ERROR("Could not initialize chain object");
 }
 KDL::ChainJntToJacSolver kdl_solver(kdl_chain);
 KDL::Jacobian jacobian_kdl(7);
 KDL::JntArray q_in(7);
 EXPECT_TRUE(kdl_solver.JntToJac(q_in,jacobian_kdl) >= 0);

 unsigned int NUM_TESTS = 10000;
 for(unsigned int i=0; i < NUM_TESTS; i++)
 {
   for(int j=0; j < 7; j++)
   {
     q_in(j) = gen_rand(-M_PI,M_PI);
     joint_angles[j] = q_in(j);
   }
   EXPECT_TRUE(kdl_solver.JntToJac(q_in,jacobian_kdl) >= 0);
   kinematic_state->setJointGroupPositions(joint_state_group, &joint_angles[0]);
   EXPECT_TRUE(kinematic_state->getJacobian(joint_state_group, kinematic_state->getRobotModel()->getLinkModel(link_name), point, jacobian));
   for(unsigned int k=0; k < 6; k++)
   {
     for(unsigned int m=0; m < 7; m++)
     {
       EXPECT_FALSE(NOT_NEAR(jacobian_kdl(k,m),jacobian(k,m),1e-10));
     }
   }
 }
}

TEST(JacobianSolver, solver)
{
  testJacobian("right_arm", "torso_lift_link", "r_wrist_roll_link");
}

TEST(JacobianSolver, solver2)
{
  testJacobian("left_arm", "torso_lift_link", "l_wrist_roll_link");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_jacobian_solver");
  return RUN_ALL_TESTS();
}
