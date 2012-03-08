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

#include <moveit_msgs/Constraints.h>

static const std::string PFRAME = "odom";

inline moveit_msgs::Constraints getDualArmConstraints(double offset)
{    
  moveit_msgs::Constraints c;
  c.name = "dual_arm";
  
  moveit_msgs::PositionConstraint pcm2;
  pcm2.link_name = "r_wrist_roll_link";
  pcm2.target_point_offset.x = offset;
  pcm2.target_point_offset.y = 0;
  pcm2.target_point_offset.z = 0;
  pcm2.constraint_region_shape.type = moveit_msgs::Shape::BOX;
  pcm2.constraint_region_shape.dimensions.push_back(0.5);
  pcm2.constraint_region_shape.dimensions.push_back(0.01);
  pcm2.constraint_region_shape.dimensions.push_back(0.01);
  
  pcm2.constraint_region_pose.header.frame_id = "l_wrist_roll_link";
  pcm2.constraint_region_pose.pose.position.x = 0.0;
  pcm2.constraint_region_pose.pose.position.y = 0.0;
  pcm2.constraint_region_pose.pose.position.z = 0.0;
  pcm2.constraint_region_pose.pose.orientation.x = 0.0;
  pcm2.constraint_region_pose.pose.orientation.y = 0.0;
  pcm2.constraint_region_pose.pose.orientation.z = 0.0;
  pcm2.constraint_region_pose.pose.orientation.w = 1.0;
  pcm2.weight = 1.0;
  c.position_constraints.push_back(pcm2);


  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "l_wrist_roll_link";
  ocm.orientation.header.frame_id = PFRAME;
  ocm.orientation.quaternion.x = 0.5;
  ocm.orientation.quaternion.y = 0.5;
  ocm.orientation.quaternion.z = 0.5;
  ocm.orientation.quaternion.w = 0.5;
  ocm.absolute_x_axis_tolerance = 0.01;
  ocm.absolute_y_axis_tolerance = M_PI;
  ocm.absolute_z_axis_tolerance = 0.01;
  ocm.weight = 1.0;
  c.orientation_constraints.push_back(ocm);

  ocm.link_name = "r_wrist_roll_link";
  ocm.orientation.header.frame_id = "l_wrist_roll_link";
  ocm.orientation.quaternion.x = 0.0;
  ocm.orientation.quaternion.y = 0.0;
  ocm.orientation.quaternion.z = 1.0;
  ocm.orientation.quaternion.w = 0.0;
  ocm.absolute_x_axis_tolerance = 0.01;
  ocm.absolute_y_axis_tolerance = 0.01;
  ocm.absolute_z_axis_tolerance = 0.01;
  ocm.weight = 1.0;
  c.orientation_constraints.push_back(ocm);
  return c;
}

inline moveit_msgs::Constraints getSingleArmConstraints(void)
{
  moveit_msgs::Constraints constr1; 
  constr1.name = "single_arm";
  constr1.orientation_constraints.resize(1);
  moveit_msgs::OrientationConstraint &ocm1 = constr1.orientation_constraints[0];
  ocm1.link_name = "r_wrist_roll_link";
  ocm1.orientation.header.frame_id = PFRAME;
  ocm1.orientation.quaternion.x = 0.0;
  ocm1.orientation.quaternion.y = 0.0;
  ocm1.orientation.quaternion.z = 0.0;
  ocm1.orientation.quaternion.w = 1.0;
  ocm1.absolute_x_axis_tolerance = 0.1;
  ocm1.absolute_y_axis_tolerance = 0.1;
  ocm1.absolute_z_axis_tolerance = M_PI;
  ocm1.weight = 1.0;
  return constr1;
}

inline moveit_msgs::Constraints getVisibilityConstraints(const std::string &id)
{ 
    moveit_msgs::Constraints c;  
    c.name = "vis";
    c.visibility_constraints.resize(1);
    moveit_msgs::VisibilityConstraint &vcm = c.visibility_constraints[0];
    
    vcm.target_radius = 0.1;
    vcm.cone_sides = 12;
    vcm.max_view_angle = M_PI/6.0;
    vcm.max_range_angle = M_PI/6.0;
    
    vcm.target_pose.header.frame_id = id;
    vcm.target_pose.pose.position.x = 0;
    vcm.target_pose.pose.position.y = 0.05;
    vcm.target_pose.pose.position.z = 0;
    vcm.target_pose.pose.orientation.x = sqrt(2.0)/2.0;
    vcm.target_pose.pose.orientation.y = 0.0;
    vcm.target_pose.pose.orientation.z = 0.0;
    vcm.target_pose.pose.orientation.w = sqrt(2.0)/2.0;
    
    vcm.sensor_pose.header.frame_id = "double_stereo_link";
    vcm.sensor_pose.pose.position.x = 0.1;
    vcm.sensor_pose.pose.position.y = 0;
    vcm.sensor_pose.pose.position.z = 0;
    vcm.sensor_pose.pose.orientation.x = 0;
    vcm.sensor_pose.pose.orientation.y = 0;
    vcm.sensor_pose.pose.orientation.z = 0;
    vcm.sensor_pose.pose.orientation.w = 1;
    vcm.weight = 1.0;

    return c;
}

