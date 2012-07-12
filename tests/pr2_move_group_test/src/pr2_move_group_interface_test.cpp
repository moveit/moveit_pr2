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

#include <move_group_interface/move_group.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rw_move_group_interface_test", ros::init_options::AnonymousName);
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  move_group_interface::MoveGroup group("arm");
//  move_group_interface::MoveGroup group2("left_arm");
/*
  std::vector<double> v0(7,0.0);
  group.setJointValueTarget(v0);
  
  group.move();
  sleep(1);
  return 0;
*/
/*
  std::vector<double> v;
  v.push_back(0.4455);
  v.push_back(-0.1734);
  v.push_back(1.2177);
  v.push_back(-0.18);
  v.push_back(-1.37);
  v.push_back(0);
  v.push_back(0);
  
  group.setJointValueTarget(v);
  
  group.move();

  sleep(3);
  
  //    std::vector<double> v;

  v.clear();
  v.push_back(-1.1);
  v.push_back(0.5);
  v.push_back(-0.373);
  v.push_back(0.126);
  v.push_back(-1.196);
  v.push_back(0);
  v.push_back(0);
*/


/*
// safe plan
  std::vector<double> v;

  v.push_back(0.27);
  v.push_back(-0.68);
  v.push_back(-0.89);
  v.push_back(-1.09);
  v.push_back(-0.87);
  v.push_back(0); 
  v.push_back(0);

  group.setJointValueTarget(v);
  group.move();
  sleep(2); 

  std::vector<double> q;
  q.push_back(0.47);
  q.push_back(0.94);
  q.push_back(-0.04);
  q.push_back(0.95);
  q.push_back(-1.67);
  q.push_back(0);
  q.push_back(0);
    
  group.setJointValueTarget(q);
  group.move();
  sleep(1); 
*/


// evil plan
  std::vector<double> v;

  v.push_back(1.15);
  v.push_back(-0.105);
  v.push_back(0.38);
  v.push_back(0.055);
  v.push_back(-1.47);
  v.push_back(0); 
  v.push_back(0);

  group.setJointValueTarget(v);
  group.move();
  sleep(2); 

  std::vector<double> q;
  q.push_back(-0.69);
  q.push_back(0.52);
  q.push_back(1.09);
  q.push_back(0.8);
  q.push_back(0.23);
  q.push_back(0);
  q.push_back(0);
    
  group.setJointValueTarget(q);
  group.move();
  sleep(1); 



  return 0;
}
