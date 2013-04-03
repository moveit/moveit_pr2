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
*
* Author: Sachin Chitta
*********************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

 
int main(int argc, char **argv)
{
  ros::init (argc, argv, "planning_scene_ros_api_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node_handle;  

  /* ATTACH AN OBJECT*/
  /* First advertise and wait for a connection*/
  ros::Publisher attached_object_publisher = node_handle.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 1);
  while(attached_object_publisher.getNumSubscribers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();    
  }

  /* Define the attached object message*/
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "r_wrist_roll_link";
  /* The header must contain a valid TF frame*/
  attached_object.object.header.frame_id = "r_wrist_roll_link";
  /* The id of the object */
  attached_object.object.id = "box";

  /* A default pose */
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;

  /* Define a box to be attached */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;  
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.1;  

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);

  /* An attach operation requires an ADD */
  attached_object.object.operation = attached_object.object.ADD;

  /* Publish and sleep (to view the visualized results)*/
  attached_object_publisher.publish(attached_object);  
  ros::WallDuration sleep_time(10.0);
  sleep_time.sleep();    

  /* DETACH THE OBJECT*/
  /* Note that the object will now be added back into the collision world.*/
  moveit_msgs::AttachedCollisionObject detach_object;
  detach_object.object.id = "box";
  detach_object.link_name = "r_wrist_roll_link";
  detach_object.object.operation = attached_object.object.REMOVE;
  ROS_INFO("Detaching the object");  
  attached_object_publisher.publish(detach_object);  
  sleep_time.sleep();    


  /* REMOVE OBJECT FROM COLLISION WORLD*/
  /* Advertise the collision object message publisher*/
  ros::Publisher collision_object_publisher = node_handle.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
  while(collision_object_publisher.getNumSubscribers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();    
  }
  /* Define the message and publish the operation*/
  moveit_msgs::CollisionObject remove_object;
  remove_object.id = "box";
  remove_object.header.frame_id = "odom_combined";  
  remove_object.operation = remove_object.REMOVE;
  ROS_INFO("Removing the object");  
  collision_object_publisher.publish(remove_object);  
  sleep_time.sleep();    
  
  /* USE THE PLANNING SCENE MESSAGE FOR THE SAME SET OF OPERATIONS*/
  /* Advertise the planning scene message publisher*/
  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  while(planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();    
  }

  /* PUT THE OBJECT IN THE ENVIRONMENT*/
  ROS_INFO("Putting the object back into the environment");  
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);  
  sleep_time.sleep();  
  
  /* ATTACH THE OBJECT */
  ROS_INFO("Attaching the object and removing it from the collision world");  
  planning_scene.world.collision_objects.clear();  
  planning_scene.world.collision_objects.push_back(remove_object);
  planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
  planning_scene_diff_publisher.publish(planning_scene);
  
  sleep_time.sleep();  

  /* DETACH THE OBJECT */
  ROS_INFO("Detaching the object and returning it to the collision world");  
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
  planning_scene.world.collision_objects.clear();  
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene_diff_publisher.publish(planning_scene);

  sleep_time.sleep();  

  /* REMOVE THE OBJECT FROM THE COLLISION WORLD */
  ROS_INFO("Removing the object again");  
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.world.collision_objects.clear();  
  planning_scene.world.collision_objects.push_back(remove_object);
  planning_scene_diff_publisher.publish(planning_scene);

  ros::shutdown(); 
  return 0;
}
