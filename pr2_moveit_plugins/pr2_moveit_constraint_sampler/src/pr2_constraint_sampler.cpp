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

/* Author: Ioan Sucan */

#include <moveit/constraint_samplers/constraint_sampler_allocator.h>
#include <pluginlib/class_list_macros.h>

namespace pr2_constraint_sampler
{

// define the actual sampler
class PR2ConstraintSampler : public constraint_samplers::ConstraintSampler
{
public:

  PR2ConstraintSampler(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name) : constraint_samplers::ConstraintSampler(scene, group_name)
  {
  }
  
  virtual bool configure(const moveit_msgs::Constraints &constr)
  {
    return false;
  }
  
  virtual bool sample(planning_models::RobotState *::JointStateGroup *jsg, const planning_models::RobotState& reference_state, unsigned int max_attempts)
  {
    return false;
  }
 
};
 
// define the sampler allocator plugin interface
class PR2ConstraintSamplerAllocator : public constraint_samplers::ConstraintSamplerAllocator
{
public:
  
  virtual constraint_samplers::ConstraintSamplerPtr alloc(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name, const moveit_msgs::Constraints &constr) 
  {
    constraint_samplers::ConstraintSamplerPtr cs(new PR2ConstraintSampler(scene, group_name));
    cs->configure(constr);
    return cs;
  }
  
  virtual bool canService(const planning_scene::PlanningSceneConstPtr &scene, const std::string &group_name, const moveit_msgs::Constraints &constr) const
  {
    return false;
  }
  
};

}

PLUGINLIB_EXPORT_CLASS(pr2_constraint_sampler::PR2ConstraintSamplerAllocator,
                       constraint_samplers::ConstraintSamplerAllocator);
