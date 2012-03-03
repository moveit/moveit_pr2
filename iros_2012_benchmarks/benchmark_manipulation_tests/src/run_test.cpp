#include <benchmark_manipulation_tests/benchmark_manipulation_tests.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "benchmark_manipulation_test");
  BenchmarkManipulationTests exp;

  if(!exp.getParams())
  {
    ROS_ERROR("Failed to get all required params from param server.");
    return false;
  }
  exp.printParams();

  sleep(3);
  ROS_INFO("------------ READY TO ROCK ------------");
  exp.visualizeLocations();
  exp.visualizeStartPose();
  ros::spinOnce();
  sleep(2);
  if(!exp.performAllExperiments())
    ROS_ERROR("Experiments failed.");

  ros::spinOnce();
  sleep(2);
  ros::spinOnce();

  ROS_INFO("Party is over! Go home, people!");
  return 0;
}

             
