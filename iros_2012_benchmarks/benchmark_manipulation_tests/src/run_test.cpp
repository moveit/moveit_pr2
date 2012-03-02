#include <benchmark_manipulation_tests/benchmark_manipulation_tests.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "benchmark_manipulation_test");
  BenchmarkManipulationTests exp;

  if(!exp.getParams())
    return false;
  exp.printParams();

  sleep(3);
  ROS_INFO("------------ READY TO ROCK ------------");
  exp.visualizeLocations();
  exp.visualizeStartPose();
  ros::spinOnce();
  sleep(2);
  if(!exp.runExperiment("exp1"))
    ROS_ERROR("Experiments failed.");

  ros::spin();
  return 0;
}

             
