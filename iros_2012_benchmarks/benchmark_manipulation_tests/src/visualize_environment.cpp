#include <benchmark_manipulation_tests/benchmark_manipulation_tests.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "visualize_environment");
  BenchmarkManipulationTests exp;
  
  if(!exp.getParams())
  {
    ROS_ERROR("Failed to get all required params from param server.");
    return false;
  }
  exp.printParams();

  sleep(2);
  exp.visualizeLocations();
  //exp.visualizeddStartPose();
  exp.visualizeEnvironment();
  ros::spinOnce();
  sleep(2);
  ros::spinOnce();
  ROS_INFO("Party is over! Go home, people!");
  return 0;
}

             
