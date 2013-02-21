/*********************************************************************
*
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
*********************************************************************/

/* Author: Ioan Sucan */

#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>

static const std::string ROBOT_DESCRIPTION="robot_description";

void pick(move_group_interface::MoveGroup &group)
{
  std::vector<manipulation_msgs::Grasp> grasps;
  
  geometry_msgs::PoseStamped p;
  p.header.frame_id = "base_footprint";
  p.pose.position.x = 0.22;
  p.pose.position.y = -0.7;
  p.pose.position.z = 0.5;
  p.pose.orientation.x = 0;
  p.pose.orientation.y = 0;
  p.pose.orientation.z = 0;
  p.pose.orientation.w = 1;
  manipulation_msgs::Grasp g;
  g.grasp_pose = p;
  g.approach.direction.vector.x = 1.0;
  g.retreat.direction.vector.z = 1.0;
  g.approach.direction.header.frame_id = "r_wrist_roll_link";
  g.retreat.direction.header.frame_id = "base_footprint";
  g.approach.min_distance = 0.2;
  g.approach.desired_distance = 0.4;
  g.retreat.min_distance = 0.1;
  g.retreat.desired_distance = 0.25;
  g.pre_grasp_posture.name.resize(1, "r_gripper_joint");
  g.pre_grasp_posture.position.resize(1);
  g.pre_grasp_posture.position[0] = 1;
  
  g.grasp_posture.name.resize(1, "r_gripper_joint");
  g.grasp_posture.position.resize(1);
  g.grasp_posture.position[0] = 0;
  
  grasps.push_back(g);
  
  group.pick("part", grasps);
}

void place(move_group_interface::MoveGroup &group)
{
  std::vector<manipulation_msgs::PlaceLocation> loc;
  for (std::size_t i = 0 ; i < 20 ; ++i)
  {
    geometry_msgs::PoseStamped p = group.getRandomPose();
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;
    manipulation_msgs::PlaceLocation g;
    g.place_pose = p;
    g.approach.direction.vector.x = 1.0;
    g.retreat.direction.vector.z = 1.0;
    g.retreat.direction.header.frame_id = "base_footprint";
    g.approach.direction.header.frame_id = "r_wrist_roll_link";
    g.approach.min_distance = 0.2;
    g.approach.desired_distance = 0.4;
    g.retreat.min_distance = 0.1;
    g.retreat.desired_distance = 0.27;
    
    g.post_place_posture.name.resize(1, "r_gripper_joint");
    g.post_place_posture.position.resize(1);
    g.post_place_posture.position[0] = 1;
    
    loc.push_back(g);
  }
  group.place("part", loc);
}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "right_arm_pick_place");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  ros::NodeHandle nh;
  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);

  ros::WallDuration(1.0).sleep();
  
  move_group_interface::MoveGroup group("right_arm");
  
  
  moveit_msgs::CollisionObject co;
  co.id = "pole";
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "base_footprint";
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.1;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.4;
  co.primitive_poses.resize(1);
  co.primitive_poses[0].position.x = 0.7;
  co.primitive_poses[0].position.y = -0.5;  
  co.primitive_poses[0].position.z = 0.7;
  co.primitive_poses[0].orientation.w = 1.0;
  pub_co.publish(co);
  
  
  co.id = "part";
  
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.2;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.01;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.3;
  
  co.primitive_poses[0].position.x = 0.5;
  co.primitive_poses[0].position.y = -0.7;  
  co.primitive_poses[0].position.z = 0.5;
  pub_co.publish(co);
  
  // wait a bit for ros things to initialize
  ros::WallDuration(1.0).sleep();
  
  pick(group);
  
  ros::WallDuration(10.0).sleep();
  
  place(group);  

  ros::waitForShutdown();
  return 0;
}
