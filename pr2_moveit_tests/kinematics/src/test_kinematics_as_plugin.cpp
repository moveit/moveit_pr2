/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* Author: Sachin Chitta
*********************************************************************/

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>

// MoveIt!
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <urdf/model.h>
#include <srdfdom/model.h>

#define IK_NEAR 1e-4
#define IK_NEAR_TRANSLATE 1e-5

class MyTest
{
 public:
  bool initialize()
  {
    double search_discretization;
    ros::NodeHandle nh("~");
    kinematics_solver = NULL;
    kinematics_loader.reset(new pluginlib::ClassLoader<kinematics::KinematicsBase>("moveit_core", "kinematics::KinematicsBase"));
    std::string plugin_name;    
    if (!nh.getParam("plugin_name", plugin_name))
    {
      ROS_ERROR("No plugin name found on parameter server");
      EXPECT_TRUE(0);      
      return false;      
    }
    ROS_INFO("Plugin name: %s",plugin_name.c_str());    
    try
    {
      kinematics_solver = kinematics_loader->createClassInstance(plugin_name);
    }
    catch(pluginlib::PluginlibException& ex)//handle the class failing to load
    {
      ROS_ERROR("The plugin failed to load. Error: %s", ex.what()); 
      EXPECT_TRUE(0);            
      return false;
    }
    std::string root_name, tip_name;
    ros::WallTime start_time = ros::WallTime::now();
    bool done = true;    
    while((ros::WallTime::now()-start_time).toSec() <= 5.0)
    {      
      if (!nh.getParam("root_name", root_name))
      {
        ROS_ERROR("No root name found on parameter server");
        done = false;      
        EXPECT_TRUE(0);      
        continue;        
      }
      if (!nh.getParam("tip_name", tip_name))
      {
        ROS_ERROR("No tip name found on parameter server");
        done = false;      
        EXPECT_TRUE(0);      
        continue;        
      }
      if (!nh.getParam("search_discretization", search_discretization))
      {
        ROS_ERROR("No search discretization found on parameter server");
        done = false;      
        EXPECT_TRUE(0);      
        continue;        
      }
      done = true;      
    }
    
    if(!done)
      return false;
    
    if(kinematics_solver->initialize("robot_description","right_arm",root_name,tip_name,search_discretization))
      return true;
    else
    {
      EXPECT_TRUE(0);        
      return false;
    }
    
  };

  void joint_state_callback(const geometry_msgs::Pose &ik_pose, 
                            const std::vector<double> &joint_state, 
                            moveit_msgs::MoveItErrorCodes &error_code)
  {
    std::vector<std::string> link_names;
    link_names.push_back("r_elbow_flex_link");
    std::vector<geometry_msgs::Pose> solutions;
    solutions.resize(1);    
    if(!kinematics_solver->getPositionFK(link_names,joint_state,solutions))
      error_code.val = error_code.PLANNING_FAILED;    
    if(solutions[0].position.z > 0.0)
      error_code.val = error_code.SUCCESS;
    else
      error_code.val = error_code.PLANNING_FAILED;
  };
  kinematics::KinematicsBase* kinematics_solver;
  boost::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase> > kinematics_loader;
};

MyTest my_test;

TEST(ArmIKPlugin, initialize)
{
  ASSERT_TRUE(my_test.initialize());
  // Test getting chain information
  std::string root_name = my_test.kinematics_solver->getBaseFrame();
  EXPECT_TRUE(root_name == std::string("torso_lift_link"));
  std::string tool_name = my_test.kinematics_solver->getTipFrame();
  EXPECT_TRUE(tool_name == std::string("r_wrist_roll_link"));
  std::vector<std::string> joint_names = my_test.kinematics_solver->getJointNames();
  EXPECT_EQ((int)joint_names.size(), 7);
  
  EXPECT_EQ(joint_names[0], "r_shoulder_pan_joint");
  EXPECT_EQ(joint_names[1], "r_shoulder_lift_joint");
  EXPECT_EQ(joint_names[2], "r_upper_arm_roll_joint");
  EXPECT_EQ(joint_names[3], "r_elbow_flex_joint");
  EXPECT_EQ(joint_names[4], "r_forearm_roll_joint");
  EXPECT_EQ(joint_names[5], "r_wrist_flex_joint");
  EXPECT_EQ(joint_names[6], "r_wrist_roll_joint");  
}

TEST(ArmIKPlugin, getFK)
{
  rdf_loader::RDFLoader rdf_loader_;
  robot_model::RobotModelPtr kinematic_model;
  const boost::shared_ptr<srdf::Model> &srdf = rdf_loader_.getSRDF();
  const boost::shared_ptr<urdf::ModelInterface>& urdf_model = rdf_loader_.getURDF();
  kinematic_model.reset(new robot_model::RobotModel(urdf_model, srdf));
  robot_model::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(my_test.kinematics_solver->getGroupName());

  std::vector<double> seed, fk_values, solution;
  moveit_msgs::MoveItErrorCodes error_code;
  solution.resize(my_test.kinematics_solver->getJointNames().size(), 0.0);
  
  std::vector<std::string> fk_names;
  fk_names.push_back(my_test.kinematics_solver->getTipFrame());

  robot_state::RobotState kinematic_state(kinematic_model);
  robot_state::JointStateGroup* joint_state_group = kinematic_state.getJointStateGroup(my_test.kinematics_solver->getGroupName());

  ros::NodeHandle nh("~");
  int number_fk_tests;  
  nh.param("number_fk_tests", number_fk_tests, 100);  
  
  for(unsigned int i=0; i < (unsigned int) number_fk_tests; ++i)
  {
    seed.resize(my_test.kinematics_solver->getJointNames().size(), 0.0);
    fk_values.resize(my_test.kinematics_solver->getJointNames().size(), 0.0);

    joint_state_group->setToRandomValues();
    joint_state_group->getVariableValues(fk_values);
    
    std::vector<geometry_msgs::Pose> poses;    
    poses.resize(1);    
    bool result_fk = my_test.kinematics_solver->getPositionFK(fk_names, fk_values, poses);
    ASSERT_TRUE(result_fk);
  }
}

TEST(ArmIKPlugin, searchIK)
{
  rdf_loader::RDFLoader rdf_loader_;
  robot_model::RobotModelPtr kinematic_model;
  const boost::shared_ptr<srdf::Model> &srdf_model = rdf_loader_.getSRDF();
  const boost::shared_ptr<urdf::ModelInterface>& urdf_model = rdf_loader_.getURDF();
  kinematic_model.reset(new robot_model::RobotModel(urdf_model, srdf_model));
  robot_model::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(my_test.kinematics_solver->getGroupName());

  //Test inverse kinematics
  std::vector<double> seed, fk_values, solution;
  double timeout = 5.0;
  moveit_msgs::MoveItErrorCodes error_code;
  solution.resize(my_test.kinematics_solver->getJointNames().size(), 0.0);
  
  std::vector<std::string> fk_names;
  fk_names.push_back(my_test.kinematics_solver->getTipFrame());

  robot_state::RobotState kinematic_state(kinematic_model);
  robot_state::JointStateGroup* joint_state_group = kinematic_state.getJointStateGroup(my_test.kinematics_solver->getGroupName());

  ros::NodeHandle nh("~");
  int number_ik_tests;  
  nh.param("number_ik_tests", number_ik_tests, 10);  
  unsigned int success = 0;
  
  ros::WallTime start_time = ros::WallTime::now();  
  for(unsigned int i=0; i < (unsigned int) number_ik_tests; ++i)
  {
    seed.resize(my_test.kinematics_solver->getJointNames().size(), 0.0);
    fk_values.resize(my_test.kinematics_solver->getJointNames().size(), 0.0);

    joint_state_group->setToRandomValues();
    joint_state_group->getVariableValues(fk_values);
    
    std::vector<geometry_msgs::Pose> poses;    
    poses.resize(1);    
    bool result_fk = my_test.kinematics_solver->getPositionFK(fk_names, fk_values, poses);
    ASSERT_TRUE(result_fk);
    
    bool result = my_test.kinematics_solver->searchPositionIK(poses[0], seed, timeout, solution, error_code);
    ROS_DEBUG("Pose: %f %f %f",poses[0].position.x, poses[0].position.y, poses[0].position.z);
    ROS_DEBUG("Orient: %f %f %f %f",poses[0].orientation.x, poses[0].orientation.y, poses[0].orientation.z, poses[0].orientation.w);    
    if(result)
    {
      success++;      
      result = my_test.kinematics_solver->getPositionIK(poses[0], solution, solution, error_code);
      EXPECT_TRUE(result);      
    }    

    std::vector<geometry_msgs::Pose> new_poses;    
    new_poses.resize(1);    
    result_fk = my_test.kinematics_solver->getPositionFK(fk_names, solution, new_poses);

    EXPECT_NEAR(poses[0].position.x, new_poses[0].position.x, IK_NEAR);
    EXPECT_NEAR(poses[0].position.y, new_poses[0].position.y, IK_NEAR);
    EXPECT_NEAR(poses[0].position.z, new_poses[0].position.z, IK_NEAR);

    EXPECT_NEAR(poses[0].orientation.x, new_poses[0].orientation.x, IK_NEAR);
    EXPECT_NEAR(poses[0].orientation.y, new_poses[0].orientation.y, IK_NEAR);
    EXPECT_NEAR(poses[0].orientation.z, new_poses[0].orientation.z, IK_NEAR);
    EXPECT_NEAR(poses[0].orientation.w, new_poses[0].orientation.w, IK_NEAR);
  }
  ROS_INFO("Success Rate: %f",(double)success/number_ik_tests);  
  bool success_count = (success > 0.99 * number_ik_tests);
  EXPECT_TRUE(success_count);
  ROS_INFO("Elapsed time: %f", (ros::WallTime::now()-start_time).toSec());  
}

TEST(ArmIKPlugin, searchIKWithCallbacks)
{
  rdf_loader::RDFLoader rdf_loader_;
  robot_model::RobotModelPtr kinematic_model;
  const boost::shared_ptr<srdf::Model> &srdf = rdf_loader_.getSRDF();
  const boost::shared_ptr<urdf::ModelInterface>& urdf_model = rdf_loader_.getURDF();
  kinematic_model.reset(new robot_model::RobotModel(urdf_model, srdf));
  robot_model::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(my_test.kinematics_solver->getGroupName());

  //Test inverse kinematics
  std::vector<double> seed,fk_values,solution;
  double timeout = 5.0;
  moveit_msgs::MoveItErrorCodes error_code;
  solution.resize(my_test.kinematics_solver->getJointNames().size(), 0.0);
  
  std::vector<std::string> fk_names;
  fk_names.push_back(my_test.kinematics_solver->getTipFrame());

  robot_state::RobotState kinematic_state(kinematic_model);
  robot_state::JointStateGroup* joint_state_group = kinematic_state.getJointStateGroup(my_test.kinematics_solver->getGroupName());

  ros::NodeHandle nh("~");
  int number_ik_tests;  
  nh.param("number_ik_tests_with_callbacks", number_ik_tests, 10);  
  unsigned int success = 0;
  unsigned int num_actual_tests = 0;
  
  for(unsigned int i=0; i < (unsigned int) number_ik_tests; ++i)
  {
    seed.resize(my_test.kinematics_solver->getJointNames().size(), 0.0);
    fk_values.resize(my_test.kinematics_solver->getJointNames().size(), 0.0);

    joint_state_group->setToRandomValues();
    joint_state_group->getVariableValues(fk_values);
    
    std::vector<geometry_msgs::Pose> poses;    
    poses.resize(1);    
    bool result_fk = my_test.kinematics_solver->getPositionFK(fk_names, fk_values, poses);
    ASSERT_TRUE(result_fk);
    if(poses[0].position.z < 0.0)
      continue;
    
    num_actual_tests++;    
    bool result = my_test.kinematics_solver->searchPositionIK(poses[0], seed, timeout, solution,
                                                              boost::bind(&MyTest::joint_state_callback, &my_test, _1, _2, _3), error_code);

    if(result)
    {
      success++;      
      std::vector<geometry_msgs::Pose> new_poses;    
      new_poses.resize(1);    
      result_fk = my_test.kinematics_solver->getPositionFK(fk_names, solution, new_poses);
      
      EXPECT_NEAR(poses[0].position.x, new_poses[0].position.x, IK_NEAR);
      EXPECT_NEAR(poses[0].position.y, new_poses[0].position.y, IK_NEAR);
      EXPECT_NEAR(poses[0].position.z, new_poses[0].position.z, IK_NEAR);
      
      EXPECT_NEAR(poses[0].orientation.x, new_poses[0].orientation.x, IK_NEAR);
      EXPECT_NEAR(poses[0].orientation.y, new_poses[0].orientation.y, IK_NEAR);
      EXPECT_NEAR(poses[0].orientation.z, new_poses[0].orientation.z, IK_NEAR);
      EXPECT_NEAR(poses[0].orientation.w, new_poses[0].orientation.w, IK_NEAR);
    }
    
    if(!ros::ok())
      break;    
  }
  ROS_INFO("Success with callbacks (%%): %f",(double)success/num_actual_tests*100.0);  
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init (argc, argv, "right_arm_kinematics");
  return RUN_ALL_TESTS();
}
