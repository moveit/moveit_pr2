#include <iostream>
#include <ros/ros.h>
#include <fstream>
#include <vector>
#include <iterator>
#include <list>
#include <map>
#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ComputePlanningBenchmark.h>
#include <pviz/pviz.h>
#include <planning_scene_monitor/planning_scene_monitor.h>

typedef struct
{
  std::vector<double> langles;
  std::vector<double> rangles;
  BodyPose body;
} RobotPose;

typedef struct
{
  int pre_action;  // 0:nothing, 1:attach, 2:detach
  int post_action; // 0:nothing, 1:attach, 2:detach

  std::string name;
  std::string goal;
  std::string sound_bite;
  RobotPose start;
} Experiment;

class BenchmarkManipulationTests
{
  public:
    
    BenchmarkManipulationTests();
    ~BenchmarkManipulationTests(){};

    bool getParams();
    bool getLocations();
    bool getExperiments();
    void printLocations();
    void printExperiments();
    void printParams();
    void printRobotPose(RobotPose &pose, std::string name);
    void startCompleteExperimentFile();
    void writeCompleteExperimentFile(Experiment e, RobotPose start);

    bool getCollisionObjects(std::string filename, std::vector<moveit_msgs::CollisionObject> &collision_objects);
    void removeCollisionObject(std::string id);
    bool getAttachedObject(std::string object_file, geometry_msgs::Pose rarm_object_pose, moveit_msgs::AttachedCollisionObject &att_object);

    bool requestPlan(RobotPose &start_state, std::string name);
    bool performAllExperiments();
    bool runExperiment(std::string name);

    void visualizeLocations();
    void visualizeRobotPose(RobotPose &pose, std::string name, int id);
    void visualizeStartPose();

    void fillSingleArmPlanningRequest(RobotPose &start_state, std::string name, moveit_msgs::MotionPlanRequest &req);
    void fillDualArmPlanningRequest(RobotPose &start_state, std::string name, moveit_msgs::MotionPlanRequest &req);

    bool setRobotPoseFromTrajectory(moveit_msgs::RobotTrajectory &trajectory, moveit_msgs::RobotState &trajectory_start, RobotPose &pose);
    bool getRobotPoseFromRobotState(const moveit_msgs::RobotState &state, RobotPose &pose);
    bool getBasePoseFromPlanningScene(const moveit_msgs::PlanningScene &scene, BodyPose &body);
    bool getJointPositionsFromTrajectory(const trajectory_msgs::JointTrajectory &traj, std::vector<std::string> &names, int waypoint, std::vector<double> &angles);

    void printPoseMsg(const geometry_msgs::Pose &p, std::string text);

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;
    ros::Publisher attached_object_pub_;
    ros::Publisher collision_object_pub_;
    ros::ServiceClient benchmark_client_;
    planning_scene_monitor::PlanningSceneMonitor* psm_;

    PViz pviz_;

    std::map<std::string, Experiment> exp_map_;
    std::map<std::string, int> action_map_;
    std::map<std::string, std::vector<double> > loc_map_;
    std::vector<std::string> action_list_;
    RobotPose start_pose_; 
    RobotPose current_pose_; 

    std::vector<std::string> planner_interfaces_;
    std::vector<std::string> rjoint_names_;
    std::vector<std::string> ljoint_names_;
    std::vector<double> goal_tolerance_;
    std::vector<double> rarm_object_offset_;
    std::vector<double> larm_object_offset_;
    geometry_msgs::Pose rarm_object_pose_;
    geometry_msgs::Pose larm_object_pose_;

    std::string known_objects_filename_;
    std::string attached_object_filename_;
    std::string trajectory_folder_name_;
    std::string trajectory_folder_path_;
    std::string trajectory_files_path_;

    bool use_current_state_as_start_;
    int experiment_type_; // 1: one arm, 2: two arm
    std::string group_name_;
    std::string planner_id_;
    std::string benchmark_service_name_;
    std::string spine_frame_;
    std::string world_frame_;
    std::string robot_model_root_frame_;
};

