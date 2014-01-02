/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
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
 *   * Neither the name of SRI International nor the names of its
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

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /* This sleep is ONLY to allow Rviz to come up */
  sleep(20.0);
  
  // BEGIN_TUTORIAL
  // 
  // Setup
  // ^^^^^
  // 
  // The :move_group_interface:`MoveGroup` class can be easily 
  // setup using just the name
  // of the group you would like to control and plan for.
  moveit::planning_interface::MoveGroup group("right_arm");

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the 
  // end-effector
  geometry_msgs::Pose pose_target;
  pose_target.orientation.w = 1.0;
  pose_target.position.x = 0.7;
  pose_target.position.y = -0.05;
  pose_target.position.z = 1.1;
  group.setPoseTarget(pose_target);

  // Setup for visualizing the plan
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  // Now, we call the planner to compute the plan
  // and visualize it if successful
  // Note that we are just planning, not asking move_group 
  // to actually move the robot
  moveit::planning_interface::MoveGroup::Plan p;
  if(group.plan(p))
  {
    ROS_INFO("Visualizing plan 1");    
    display_trajectory.trajectory_start = p.start_state_;
    display_trajectory.trajectory.push_back(p.trajectory_);
    display_publisher.publish(display_trajectory);
  }  

  /* This sleep is ONLY to give us enough time to see the trajectory 
     in Rviz before doing the next thing*/
  sleep(10.0);
  
  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // Moving to a pose goal is similar to the step above
  // except we now use the move() function. Note that
  // the pose goal we had set earlier is still active 
  // and so the robot will try to move to that goal. We will
  // not use that function in this tutorial since it is 
  // a blocking function and requires a controller to be active
  // and report success on execution of a trajectory.
 
  /* Uncomment below line when working with a real robot*/
  /* group.move() */

  // Planning to a joint-space goal 
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's set a joint space goal and move towards it. 
  // First, we will clear the pose target we had just set.

  group.clearPoseTargets();

  // Then, we will get the current set of joint values for the group

  std::vector<double> group_variable_values;
  group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
  
  // Now, let's modify one of the joints, plan to the new joint
  // space goal and visualize the plan
  group_variable_values[0] = -1.0;  
  group.setJointValueTarget(group_variable_values);
  moveit::planning_interface::MoveGroup::Plan p2;
  if(group.plan(p2))
  {
    ROS_INFO("Visualizing plan 2");    
    display_trajectory.trajectory_start = p2.start_state_;
    display_trajectory.trajectory.clear();    
    display_trajectory.trajectory.push_back(p2.trajectory_);
    display_publisher.publish(display_trajectory);
  }  

  /* This sleep is ONLY to give us enough time to see the trajectory 
     in Rviz before doing the next thing*/
  sleep(10.0);
  
  // Planning with Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Path constraints can easily be specified for a link on the robot.
  // Let's specify a path constraint and a pose goal for our group.
  // First define the constraint
  moveit_msgs::OrientationConstraint ocm;  
  ocm.link_name = "r_wrist_roll_link";  
  ocm.header.frame_id = "base_link";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;
  
  // Now, add it to the set of path constraints for the group
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);  
  group.setPathConstraints(test_constraints);

  // We will reuse the old goal that we had and plan to it.
  // Note that this will only work if the current state already 
  // satisfies the path constraints. So, we need to set the start
  // state to a new pose. 
  robot_state::RobotState start_state(*group.getCurrentState());
  geometry_msgs::Pose new_pose;
  new_pose.orientation.w = 1.0;
  new_pose.position.x = 0.65;
  new_pose.position.y = -0.05;
  new_pose.position.z = 0.7;
  start_state.setFromIK(start_state.getRobotModel()->getJointModelGroup(group.getName()),
                        new_pose);
  group.setStartState(start_state);
  
  // Now we will plan to the earlier pose target from the new 
  // start state that we have just created.
  group.setPoseTarget(pose_target);
  if(group.plan(p))
  {
    ROS_INFO("Visualizing plan 3");    
    display_trajectory.trajectory_start = p.start_state_;
    display_trajectory.trajectory.clear();    
    display_trajectory.trajectory.push_back(p.trajectory_);
    display_publisher.publish(display_trajectory);
  }  
  /* This sleep is ONLY to give us enough time to see the trajectory 
     in Rviz before doing the next thing*/
  sleep(10.0);

  // Adding/Removing Objects and Attaching/Detaching Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // First, we will define the collision object message
  moveit_msgs::CollisionObject collision_object;
  /* The header must contain a valid TF frame*/
  collision_object.header.frame_id = group.getEndEffectorLink();
  /* The id of the object */
  collision_object.id = "box";

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

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;  
  collision_objects.push_back(collision_object);  

  // Now, let's add the collision object into the world
  ROS_INFO("Add an object into the world");  
  planning_scene_interface.addCollisionObjects(collision_objects);
  
  /* This sleep is ONLY to give us enough time to see the object appear in Rviz*/
  sleep(2.0);

  // Now, let's attach the collision object to the robot
  ROS_INFO("Attach the object to the robot");  
  group.attachObject(collision_object.id);  
  sleep(4.0);

  // Now, let's detach the collision object from the robot
  ROS_INFO("Detach the object from the robot");  
  group.detachObject(collision_object.id);  
  sleep(4.0);

  // Now, let's remove the collision object from the world
  ROS_INFO("Remove the object from the world");  
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);  
  planning_scene_interface.removeCollisionObjects(object_ids);
  sleep(4.0);
// END_TUTORIAL

  ros::shutdown();  
  return 0;
}
