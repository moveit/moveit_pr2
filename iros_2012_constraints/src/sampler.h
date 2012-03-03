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

#include <kinematic_constraints/constraint_samplers.h>
#include <planning_scene_monitor/planning_scene_monitor.h>
#include <kinematics_plugin_loader/kinematics_plugin_loader.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";

class ValidStateSampler
{
public:
    
    ValidStateSampler(const std::string &group, const moveit_msgs::Constraints &c) : psm_(ROBOT_DESCRIPTION), group_(group)
    {
	planning_scene::PlanningScene &scene = *psm_.getPlanningScene();
	
	// sample a start state that meets the path constraints
	kinematics_plugin_loader::KinematicsLoaderFn kinematics_allocator = kinematics_loader_.getLoaderFunction();
	
	if (group == "arms")
	{
	    kinematic_constraints::KinematicsSubgroupAllocator sa;
	    sa[scene.getKinematicModel()->getJointModelGroup("left_arm")] = kinematics_allocator;
	    sa[scene.getKinematicModel()->getJointModelGroup("right_arm")] = kinematics_allocator;
	    s_ = kinematic_constraints::ConstraintSampler::constructFromMessage
		(scene.getKinematicModel()->getJointModelGroup("arms"), c, scene.getKinematicModel(), scene.getTransforms(), kinematic_constraints::KinematicsAllocator(), sa);
	}
	else
	    s_ = kinematic_constraints::ConstraintSampler::constructFromMessage
		(scene.getKinematicModel()->getJointModelGroup(group), c, scene.getKinematicModel(), scene.getTransforms(), kinematics_allocator);
    }
    
    bool sample(planning_models::KinematicState &ks, unsigned int attempts = 1000)
    {
	for (unsigned int i = 0 ; i < attempts ; ++i)
	{
	    std::vector<double> values;
	    if (s_->sample(values, ks, 10))
	    {
		ks.getJointStateGroup(group_)->setStateValues(values);
		ks.getJointStateGroup(group_)->updateLinkTransforms();
		return true;
	    }
	}
	return false;
    }
    
private:
    
    planning_scene_monitor::PlanningSceneMonitor psm_;
    std::string group_;
    kinematics_plugin_loader::KinematicsPluginLoader kinematics_loader_;
    kinematic_constraints::ConstraintSamplerPtr s_;
};
