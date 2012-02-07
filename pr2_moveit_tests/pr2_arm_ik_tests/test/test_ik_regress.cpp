//Software License Agreement (BSD License)

//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.

//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:

// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <ros/ros.h>
#include <pr2_arm_kinematics/pr2_arm_ik_solver.h>
#include <pr2_arm_kinematics/pr2_arm_kinematics_utils.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <moveit_msgs/MoveItErrorCodes.h>

#include <gtest/gtest.h>



#define IK_NEAR 1e-4
#define IK_NEAR_TRANSLATE 1e-5
using namespace pr2_arm_kinematics;
using namespace KDL;

double gen_rand(double min, double max)
{
  int rand_num = rand()%100+1;
  double result = min + (double)((max-min)*rand_num)/101.0;
  return result;
}

TEST(PR2ArmIK, initialize)
{
  ros::NodeHandle nh("pr2_ik_regression_test");
  urdf::Model robot_model;
  std::string tip_name,root_name,xml_string;
  int free_angle;
  double sd;
  ROS_INFO("Running test");
  EXPECT_TRUE(loadRobotModel(nh,robot_model,xml_string));
  nh.param<int>("free_angle",free_angle,2);
  nh.param<double>("search_discretization",sd,0.01);
}

bool NOT_NEAR(const double &v1, const double &v2, const double &NEAR)
{
   if(fabs(v1-v2) > NEAR)
      return true;
   return false;
}

TEST(PR2ArmIK, inverseKinematics)
{
  ros::NodeHandle nh("/pr2_ik_regression_test");
  urdf::Model robot_model;
  std::string tip_name,root_name,xml_string;
  int free_angle;
  double sd;

  EXPECT_TRUE(loadRobotModel(nh,robot_model,xml_string));
  nh.param<int>("free_angle",free_angle,2);
  nh.param<double>("search_discretization",sd,0.01);

  if (!nh.getParam("root_name", root_name)){
    ROS_ERROR("PR2IK: No root name found on parameter server");
  }
  if (!nh.getParam("tip_name", tip_name)){
    ROS_ERROR("PR2IK: No tip name found on parameter server");
  }

  pr2_arm_kinematics::PR2ArmIKSolver ik(robot_model,root_name,tip_name,sd,free_angle);

  KDL::Chain kdl_chain;
  EXPECT_TRUE(pr2_arm_kinematics::getKDLChain(xml_string,root_name,tip_name,kdl_chain));

  int num_tests = 10000;
  EXPECT_TRUE(ik.active_);

  KDL::ChainFkSolverPos_recursive *jnt_to_pose_solver;
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_out;
  KDL::Frame p_out;
  KDL::Frame p_ik;

  srand ( time(NULL) ); // initialize random seed: 

  jnt_to_pose_solver = new KDL::ChainFkSolverPos_recursive(kdl_chain);
  jnt_pos_in.resize(7);
  jnt_pos_out.resize(7);

  kinematics_msgs::KinematicSolverInfo chain_info;
  ik.getSolverInfo(chain_info);

  int num_solutions(0);

  for(int kk = 0; kk < num_tests; kk++)
  {
    for(int i=0; i < 7; i++)
    {
      jnt_pos_in(i) = gen_rand(std::max(chain_info.limits[i].min_position,-M_PI),std::min(chain_info.limits[i].max_position,M_PI));
      EXPECT_TRUE((jnt_pos_in(i) <= chain_info.limits[i].max_position));
      EXPECT_TRUE((jnt_pos_in(i) >= chain_info.limits[i].min_position));
    }

    if(jnt_to_pose_solver->JntToCart(jnt_pos_in,p_out) >=0)
    {
      bool ik_valid = (ik.CartToJnt(jnt_pos_in,p_out,jnt_pos_out) >= 0);
      if(ik_valid)
      {
        num_solutions++;
        jnt_to_pose_solver->JntToCart(jnt_pos_out,p_ik);
        for(int j=0; j< 3; j++)
        {
          EXPECT_NEAR(p_ik.M(j,0),p_out.M(j,0),IK_NEAR);
          EXPECT_NEAR(p_ik.M(j,1),p_out.M(j,1),IK_NEAR); 
          EXPECT_NEAR(p_ik.M(j,2),p_out.M(j,2),IK_NEAR); 
          EXPECT_NEAR(p_ik.p(j),p_out.p(j),IK_NEAR_TRANSLATE);
        }
      }
    }
  }

  bool success = ((double)num_solutions)/((double) num_tests) > 0.99;
  ROS_INFO("Success rate is %f",((double)num_solutions)/((double) num_tests));
  EXPECT_TRUE(success);
  delete jnt_to_pose_solver;
}


TEST(PR2ArmIK, inverseKinematicsSearch)
{
  ros::NodeHandle nh("/pr2_ik_regression_test");
  urdf::Model robot_model;
  std::string tip_name,root_name,xml_string;
  int free_angle;
  double sd;

  EXPECT_TRUE(loadRobotModel(nh,robot_model,xml_string));
  nh.param<int>("free_angle",free_angle,2);
  nh.param<double>("search_discretization",sd,0.001);
  if (!nh.getParam("root_name", root_name)){
    ROS_ERROR("PR2IK: No root name found on parameter server");
  }
  if (!nh.getParam("tip_name", tip_name)){
    ROS_ERROR("PR2IK: No tip name found on parameter server");
  }


  pr2_arm_kinematics::PR2ArmIKSolver ik(robot_model,root_name,tip_name,sd,free_angle);

  KDL::Chain kdl_chain;
  EXPECT_TRUE(pr2_arm_kinematics::getKDLChain(xml_string,root_name,tip_name,kdl_chain));

  kinematics_msgs::KinematicSolverInfo chain_info;
  ik.getSolverInfo(chain_info);

  int num_tests = 10000;
  KDL::ChainFkSolverPos_recursive *jnt_to_pose_solver;
  KDL::JntArray jnt_pos_in;
  KDL::JntArray jnt_pos_test;
  std::vector<KDL::JntArray> jnt_pos_out;
  KDL::Frame p_out;
  KDL::Frame p_ik;

  srand ( time(NULL) );

  jnt_to_pose_solver = new KDL::ChainFkSolverPos_recursive(kdl_chain);
  jnt_pos_in.resize(7);
  jnt_pos_test.resize(7);

  int num_solutions(0);

  for(int kk = 0; kk < num_tests; kk++)
  {
    ROS_INFO("Test: %d",kk);
    for(int i=0; i < 7; i++)
    {
      jnt_pos_in(i) = gen_rand(std::max(chain_info.limits[i].min_position,-M_PI),std::min(chain_info.limits[i].max_position,M_PI));
      EXPECT_TRUE((jnt_pos_in(i) <= chain_info.limits[i].max_position));
      EXPECT_TRUE((jnt_pos_in(i) >= chain_info.limits[i].min_position));
    }
    for(int i=0; i < 7; i++)
    {
      jnt_pos_test(i) = gen_rand(std::max(chain_info.limits[i].min_position,-M_PI),std::min(chain_info.limits[i].max_position,M_PI));
      EXPECT_TRUE((jnt_pos_test(i) <= chain_info.limits[i].max_position));
      EXPECT_TRUE((jnt_pos_test(i) >= chain_info.limits[i].min_position));
      }
    if(jnt_to_pose_solver->JntToCart(jnt_pos_in,p_out) >=0)
    {
      moveit_msgs::MoveItErrorCodes error_code;
      bool ik_valid = (ik.CartToJntSearch(jnt_pos_test,p_out,jnt_pos_out,10) >= 0);
      if(ik_valid)
      {
        num_solutions++;
        for(int k=0; k < (int)jnt_pos_out.size(); k++)
        {
          jnt_to_pose_solver->JntToCart(jnt_pos_out[k],p_ik);
          for(int j=0; j< 3; j++)
          {
            EXPECT_NEAR(p_ik.M(j,0),p_out.M(j,0),IK_NEAR);
            EXPECT_NEAR(p_ik.M(j,1),p_out.M(j,1),IK_NEAR); 
            EXPECT_NEAR(p_ik.M(j,2),p_out.M(j,2),IK_NEAR); 
            EXPECT_NEAR(p_ik.p(j),p_out.p(j),IK_NEAR_TRANSLATE);
          }
        }
      }
      else
      {
        ROS_INFO("Failed solution");
        for(int m=0; m < 3; m++)
        {
          printf("%f %f %f %f\n",p_out.M(m,0),p_out.M(m,1),p_out.M(m,2),p_out.p(m));
        }
        printf("\n");
        ROS_INFO("Original joint values");
          for(int n = 0; n< 7; n++)
          {
            printf("%f ",jnt_pos_in(n));
          }
          printf("\n");
          ROS_INFO("Guess: %f",jnt_pos_test(2));
        ROS_INFO("\n\n\n");
      }
    }
  }
  bool success = ((double)num_solutions)/((double) num_tests) > 0.99;
  ROS_INFO("Success rate is %f",((double)num_solutions)/((double) num_tests));
  EXPECT_TRUE(success);
  delete jnt_to_pose_solver;
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init (argc, argv, "pr2_ik_regression_test");
  return RUN_ALL_TESTS();
}
