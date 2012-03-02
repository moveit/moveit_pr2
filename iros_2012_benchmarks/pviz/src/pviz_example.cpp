#include <iostream>
#include <ros/ros.h>
#include <pviz/pviz.h>
#include <boost/lexical_cast.hpp>

int main(int argc, char** argv)
{
  ros::init(argc,argv,"pviz_example");
  PViz pviz;
  ros::NodeHandle ph("~");
  ROS_INFO("Sleeping for 5 seconds at startup...");
  sleep(5); 
  ROS_INFO("Here comes the PR2 train! choo choo!");

  std::vector<double> jnt0_pos(7,0);
  std::vector<double> jnt1_pos(7,0);
  std::vector<double> base_pos(3,0);

  /*
  int i = 0;
  double torso_pos = 0.10;
  while(i < 10)
  {
    ROS_INFO("Translating the PR2, raising the torso and visualizing...");
    base_pos[0] += 1.0;
    torso_pos += 0.05;
    pviz.visualizeRobot(jnt0_pos, jnt1_pos, base_pos, torso_pos, 30*i, "robot"+boost::lexical_cast<std::string>(i), i);
    i++;
    ros::spinOnce();
    sleep(2.0);
    ros::spinOnce();
  }
  */

  std::string filename;
  //ros::NodeHandle ph("~");
  sleep(2);
  printf("Getting Param");
  ph.param<std::string>("trajectory_filename",filename, "");
  sleep(1);
  printf("Filename: %s", filename.c_str());

  pviz.visualizeTrajectoryFromFile(filename);
  ros::spinOnce();
  sleep(10);
  return 0;
}

