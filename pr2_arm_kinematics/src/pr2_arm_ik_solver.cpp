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

#include <pr2_arm_kinematics/pr2_arm_ik_solver.h>

using namespace Eigen;
using namespace pr2_arm_kinematics;

PR2ArmIKSolver::PR2ArmIKSolver(const urdf::Model &robot_model, 
                               const std::string &root_frame_name,
                               const std::string &tip_frame_name,
                               const double &search_discretization_angle, 
                               const int &free_angle):ChainIkSolverPos()
{
  search_discretization_angle_ = search_discretization_angle;
  free_angle_ = free_angle;
  root_frame_name_ = root_frame_name;
  if(!pr2_arm_ik_.init(robot_model,root_frame_name,tip_frame_name))
    active_ = false;
  else
    active_ = true;
}

void PR2ArmIKSolver::getSolverInfo(kinematics_msgs::KinematicSolverInfo &response)
{
  pr2_arm_ik_.getSolverInfo(response);
}

int PR2ArmIKSolver::CartToJnt(const KDL::JntArray& q_init, 
                              const KDL::Frame& p_in, 
                              KDL::JntArray &q_out)
{
  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  std::vector<std::vector<double> > solution_ik;
  if(free_angle_ == 0)
  {
    ROS_DEBUG("Solving with %f",q_init(0)); 
    pr2_arm_ik_.computeIKShoulderPan(b,q_init(0),solution_ik);
  }
  else
  {
    pr2_arm_ik_.computeIKShoulderRoll(b,q_init(2),solution_ik);
  }
  
  if(solution_ik.empty())
    return -1;

  double min_distance = 1e6;
  int min_index = -1;

  for(int i=0; i< (int) solution_ik.size(); i++)
  {     
    ROS_DEBUG("Solution : %d",(int)solution_ik.size());

    for(int j=0; j < (int)solution_ik[i].size(); j++)
    {   
      ROS_DEBUG("%d: %f",j,solution_ik[i][j]);
    }
    ROS_DEBUG(" ");
    ROS_DEBUG(" ");

    double tmp_distance = computeEuclideanDistance(solution_ik[i],q_init);
    if(tmp_distance < min_distance)
    {
      min_distance = tmp_distance;
      min_index = i;
    }
  }

  if(min_index > -1)
  {
    q_out.resize((int)solution_ik[min_index].size());
    for(int i=0; i < (int)solution_ik[min_index].size(); i++)
    {   
      q_out(i) = solution_ik[min_index][i];
    }
    return 1;
  }
  else
    return -1;
}

int PR2ArmIKSolver::CartToJnt(const KDL::JntArray& q_init, 
                              const KDL::Frame& p_in, 
                              std::vector<KDL::JntArray> &q_out)
{
  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  std::vector<std::vector<double> > solution_ik;
  KDL::JntArray q;

  if(free_angle_ == 0)
  {
    pr2_arm_ik_.computeIKShoulderPan(b,q_init(0),solution_ik);
  }
  else
  {
    pr2_arm_ik_.computeIKShoulderRoll(b,q_init(2),solution_ik);
  }
  
  if(solution_ik.empty())
    return -1;

  q.resize(7);
  q_out.clear();
  for(int i=0; i< (int) solution_ik.size(); i++)
  {     
    for(int j=0; j < 7; j++)
    {   
      q(j) = solution_ik[i][j];
    }
    q_out.push_back(q);
  }
  return 1;
}

bool PR2ArmIKSolver::getCount(int &count, 
                              const int &max_count, 
                              const int &min_count)
{
  if(count > 0)
  {
    if(-count >= min_count)
    {   
      count = -count;
      return true;
    }
    else if(count+1 <= max_count)
    {
      count = count+1;
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    if(1-count <= max_count)
    {
      count = 1-count;
      return true;
    }
    else if(count-1 >= min_count)
    {
      count = count -1;
      return true;
    }
    else
      return false;
  }
}

int PR2ArmIKSolver::CartToJntSearch(const KDL::JntArray& q_in, 
                                    const KDL::Frame& p_in, 
                                    std::vector<KDL::JntArray> &q_out, 
                                    const double &timeout)
{
  KDL::JntArray q_init = q_in;
  //  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  double initial_guess = q_init(free_angle_);

  ros::Time start_time = ros::Time::now();
  double loop_time = 0;
  int count = 0;

  int num_positive_increments = (int)((pr2_arm_ik_.solver_info_.limits[free_angle_].max_position-initial_guess)/search_discretization_angle_);
  int num_negative_increments = (int)((initial_guess-pr2_arm_ik_.solver_info_.limits[free_angle_].min_position)/search_discretization_angle_);
  ROS_DEBUG("%f %f %f %d %d \n\n",initial_guess,pr2_arm_ik_.solver_info_.limits[free_angle_].max_position,pr2_arm_ik_.solver_info_.limits[free_angle_].min_position,num_positive_increments,num_negative_increments);
  while(loop_time < timeout)
  {
    if(CartToJnt(q_init,p_in,q_out) > 0)
      return 1;
    if(!getCount(count,num_positive_increments,-num_negative_increments))
      return -1;
    q_init(free_angle_) = initial_guess + search_discretization_angle_ * count;
    ROS_DEBUG("%d, %f",count,q_init(free_angle_));
    loop_time = (ros::Time::now()-start_time).toSec();
  }
  if(loop_time >= timeout)
  {
    ROS_DEBUG("IK Timed out in %f seconds",timeout);
    return TIMED_OUT;
  }
  else
  {
    ROS_DEBUG("No IK solution was found");
    return NO_IK_SOLUTION;
  }
  return NO_IK_SOLUTION;
}

int PR2ArmIKSolver::CartToJntSearch(const KDL::JntArray& q_in, 
                                    const KDL::Frame& p_in, 
                                    KDL::JntArray &q_out, 
                                    const double &timeout)
{
  KDL::JntArray q_init = q_in;
  //  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  double initial_guess = q_init(free_angle_);

  ros::Time start_time = ros::Time::now();
  double loop_time = 0;
  int count = 0;

  int num_positive_increments = (int)((pr2_arm_ik_.solver_info_.limits[free_angle_].max_position-initial_guess)/search_discretization_angle_);
  int num_negative_increments = (int)((initial_guess-pr2_arm_ik_.solver_info_.limits[free_angle_].min_position)/search_discretization_angle_);
  ROS_DEBUG("%f %f %f %d %d \n\n",initial_guess,pr2_arm_ik_.solver_info_.limits[free_angle_].max_position,pr2_arm_ik_.solver_info_.limits[free_angle_].min_position,num_positive_increments,num_negative_increments);
  while(loop_time < timeout)
  {
    if(CartToJnt(q_init,p_in,q_out) > 0)
      return 1;
    if(!getCount(count,num_positive_increments,-num_negative_increments))
      return -1;
    q_init(free_angle_) = initial_guess + search_discretization_angle_ * count;
    ROS_DEBUG("%d, %f",count,q_init(free_angle_));
    loop_time = (ros::Time::now()-start_time).toSec();
  }
  if(loop_time >= timeout)
  {
    ROS_DEBUG("IK Timed out in %f seconds",timeout);
    return TIMED_OUT;
  }
  else
  {
    ROS_DEBUG("No IK solution was found");
    return NO_IK_SOLUTION;
  }
  return NO_IK_SOLUTION;
}

int PR2ArmIKSolver::CartToJntSearch(const KDL::JntArray& q_in, 
                                    const KDL::Frame& p_in, 
                                    KDL::JntArray &q_out, 
                                    const double &timeout, 
                                    moveit_msgs::MoveItErrorCodes &error_code,
                                    const KDLIKCallbackFn &desired_pose_callback,
                                    const KDLIKCallbackFn &solution_callback)
{
    //  Eigen::Matrix4f b = KDLToEigenMatrix(p_in);
  KDL::JntArray q_init = q_in;
  double initial_guess = q_init(free_angle_);

  ros::Time start_time = ros::Time::now();
  double loop_time = 0;
  int count = 0;

  int num_positive_increments = (int)((pr2_arm_ik_.solver_info_.limits[free_angle_].max_position-initial_guess)/search_discretization_angle_);
  int num_negative_increments = (int)((initial_guess-pr2_arm_ik_.solver_info_.limits[free_angle_].min_position)/search_discretization_angle_);
  ROS_DEBUG("%f %f %f %d %d \n\n",initial_guess,pr2_arm_ik_.solver_info_.limits[free_angle_].max_position,pr2_arm_ik_.solver_info_.limits[free_angle_].min_position,num_positive_increments,num_negative_increments);

  if(!desired_pose_callback.empty())
    desired_pose_callback(q_init,p_in,error_code);
  if(error_code.val != error_code.SUCCESS)
  {
    return -1;
  }
  bool callback_check = true;
  if(solution_callback.empty())
    callback_check = false;

  while(loop_time < timeout)
  {
    if(CartToJnt(q_init,p_in,q_out) > 0)
    {
      if(callback_check)
      {
        solution_callback(q_out,p_in,error_code);
        if(error_code.val == error_code.SUCCESS)
        {
          return 1;
        }
      }
      else
      {
        error_code.val = error_code.SUCCESS;
        return 1;
      }
    }
    if(!getCount(count,num_positive_increments,-num_negative_increments))
    {
      error_code.val = error_code.NO_IK_SOLUTION;
      return -1;
    }
    q_init(free_angle_) = initial_guess + search_discretization_angle_ * count;
    ROS_DEBUG("Redundancy search, index:%d, free angle value: %f",count,q_init(free_angle_));
    loop_time = (ros::Time::now()-start_time).toSec();
  }
  if(loop_time >= timeout)
  {
    ROS_DEBUG("IK Timed out in %f seconds",timeout);
    error_code.val = error_code.TIMED_OUT;
  }
  else
  {
    ROS_DEBUG("No IK solution was found");
    error_code.val = error_code.NO_IK_SOLUTION;
  }
  return -1;
}

std::string PR2ArmIKSolver::getFrameId()
{
  return root_frame_name_;
}
