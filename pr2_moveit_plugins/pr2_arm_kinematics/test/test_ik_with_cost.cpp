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

#include <kinematics_msgs/IKServiceWithCost.h>

#include <kinematics_msgs/IKQuery.h>
#include <kinematics_msgs/IKService.h>
#include <kinematics_msgs/FKService.h>

#include <gtest/gtest.h>

#define IK_NEAR 1e-4
#define IK_NEAR_TRANSLATE 1e-5

static const std::string ARM_FK_NAME = "/pr2_ik_right_arm/fk_service";
static const std::string ARM_IK_NAME = "/pr2_ik_right_arm/ik_with_cost_service";
static const std::string ARM_QUERY_NAME = "/pr2_ik_right_arm/ik_query";

static const int NUM_TESTS = 1000;

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

void test()
{
  srand ( time(NULL) ); // initialize random seed
  ros::NodeHandle rh;
  ros::service::waitForService(ARM_QUERY_NAME);
  ros::service::waitForService(ARM_FK_NAME);
  ros::ServiceClient query_client = rh.serviceClient<kinematics_msgs::IKQuery>(ARM_QUERY_NAME, true);
  ros::ServiceClient fk_client = rh.serviceClient<kinematics_msgs::FKService>(ARM_FK_NAME, true);
  ros::ServiceClient ik_client = rh.serviceClient<kinematics_msgs::IKServiceWithCost>(ARM_IK_NAME, true);

  // define the service messages
  kinematics_msgs::IKServiceWithCost::Request request;
  kinematics_msgs::IKServiceWithCost::Response response;

  request.data.pose_stamped.header.frame_id = "torso_lift_link";
  request.data.pose_stamped.header.stamp    = ros::Time::now();
  request.data.pose_stamped.pose.position.x = 0.75;
  request.data.pose_stamped.pose.position.y = -0.188;
  request.data.pose_stamped.pose.position.z = 0.0;

  request.data.pose_stamped.pose.orientation.x = 0.0;
  request.data.pose_stamped.pose.orientation.y = 0.0;
  request.data.pose_stamped.pose.orientation.z = 0.0;
  request.data.pose_stamped.pose.orientation.w = 1.0;

  request.data.positions.resize(7);
  request.data.joint_names.resize(7);

  request.data.joint_names[0] = "r_shoulder_pan_joint";
  request.data.joint_names[1] = "r_shoulder_lift_joint";
  request.data.joint_names[2] = "r_upper_arm_roll_joint";
  request.data.joint_names[3] = "r_elbow_flex_joint";
  request.data.joint_names[4] = "r_forearm_roll_joint";
  request.data.joint_names[5] = "r_wrist_flex_joint";
  request.data.joint_names[6] = "r_wrist_roll_joint";

  request.link_names.resize(1);
  request.link_names[0] = "r_wrist_roll_link";

  bool ik_service_call = ik_client.call(request,response);
  if(ik_service_call)
    {
      for(unsigned int i=0; i < response.solution.size(); i++)
	ROS_INFO("joint: %s, value: %f",request.data.joint_names[i].c_str(),response.solution[i]);
    }
  else
    {
      ROS_ERROR("Call was unsuccessful");
    }
  ros::shutdown();
}

int main(int argc, char **argv){
  ros::init (argc, argv, "pr2_ik_node_test");
  ros::spinOnce();
  test();
}
