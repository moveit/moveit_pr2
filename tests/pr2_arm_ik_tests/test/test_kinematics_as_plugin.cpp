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

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>

#include <kinematics_base/kinematics_base.h>


class MyTest
{
 public:
  bool initialize()
  {
    double search_discretization;
    ros::NodeHandle nh("~");
    pluginlib::ClassLoader<kinematics::KinematicsBase> kinematics_loader("kinematics_base", "kinematics::KinematicsBase");
    kinematics_solver = NULL;

    try
    {
      kinematics_solver = kinematics_loader.createClassInstance("pr2_arm_kinematics/PR2ArmKinematicsPlugin");
    }
    catch(pluginlib::PluginlibException& ex)//handle the class failing to load
    {
      ROS_ERROR("The plugin failed to load. Error: %s", ex.what());
      return false;
    }
    std::string root_name, tip_name;
    if (!nh.getParam("root_name", root_name)){
      ROS_ERROR("PR2IK: No root name found on parameter server");
    }
    if (!nh.getParam("tip_name", tip_name)){
      ROS_ERROR("PR2IK: No tip name found on parameter server");
    }
    if (!nh.getParam("search_discretization", search_discretization)){
      ROS_ERROR("PR2IK: No search discretization found on parameter server");
    }
    if(kinematics_solver->initialize("right_arm",root_name,tip_name,search_discretization))
      return true;
    else
      return false;
  };
  void pose_callback(const geometry_msgs::Pose &ik_pose, 
                     const std::vector<double> &joint_state, 
                     moveit_msgs::MoveItErrorCodes &error_code)
  {
    error_code.val = error_code.SUCCESS;
  };
  void joint_state_callback(const geometry_msgs::Pose &ik_pose, 
                            const std::vector<double> &joint_state, 
                            moveit_msgs::MoveItErrorCodes &error_code)
  {
    std::vector<std::string> link_names;
    link_names.push_back("r_elbow_flex_link");
    std::vector<geometry_msgs::Pose> solutions;
    kinematics_solver->getPositionFK(link_names,joint_state,solutions);
    if(solutions[0].position.z > 0.0)
      error_code.val = error_code.SUCCESS;
    else
      error_code.val = error_code.PLANNING_FAILED;
  };
  kinematics::KinematicsBase* kinematics_solver;
};

TEST(PR2ArmIKPlugin, testplugin)
{
  MyTest my_test;
  ASSERT_TRUE(my_test.initialize());
  // Test getting chain information
  std::string root_name = my_test.kinematics_solver->getBaseFrame();
  EXPECT_TRUE(root_name == std::string("torso_lift_link"));
  std::string tool_name = my_test.kinematics_solver->getTipFrame();
  EXPECT_TRUE(tool_name == std::string("r_wrist_roll_link"));
  std::vector<std::string> joint_names = my_test.kinematics_solver->getJointNames();
  EXPECT_EQ((int)joint_names.size(),7);
  
  EXPECT_EQ(joint_names[0],"r_shoulder_pan_joint");
  EXPECT_EQ(joint_names[1],"r_shoulder_lift_joint");
  EXPECT_EQ(joint_names[2],"r_upper_arm_roll_joint");
  EXPECT_EQ(joint_names[3],"r_elbow_flex_joint");
  EXPECT_EQ(joint_names[4],"r_forearm_roll_joint");
  EXPECT_EQ(joint_names[5],"r_wrist_flex_joint");
  EXPECT_EQ(joint_names[6],"r_wrist_roll_joint");
  
  //Test inverse kinematics
  geometry_msgs::Pose pose;
  pose.position.x = 0.75;
  pose.position.y = -0.188;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  std::vector<double> seed,solution;
  seed.resize(7,0.0);
  double timeout = 5.0;
  moveit_msgs::MoveItErrorCodes error_code;
  bool result = my_test.kinematics_solver->searchPositionIK(pose,seed,timeout,solution, error_code);
  EXPECT_TRUE(result);
  result = my_test.kinematics_solver->getPositionIK(pose,solution,solution, error_code);
  EXPECT_TRUE(result);  

  result = my_test.kinematics_solver->searchPositionIK(pose,seed,timeout,solution,
                                                       boost::bind(&MyTest::pose_callback, &my_test, _1, _2, _3), 
                                                       boost::bind(&MyTest::joint_state_callback, &my_test, _1, _2, _3), error_code);
  EXPECT_TRUE(result);    
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init (argc, argv, "pr2_right_arm_ik_test");
  return RUN_ALL_TESTS();
}
