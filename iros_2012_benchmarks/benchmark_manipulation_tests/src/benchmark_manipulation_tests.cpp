#include <benchmark_manipulation_tests/benchmark_manipulation_tests.h>
#include <sys/stat.h>
#include <dirent.h>

BenchmarkManipulationTests::BenchmarkManipulationTests() : ph_("~")
{
  action_list_.push_back("nothing");
  action_list_.push_back("attach");
  action_list_.push_back("detach");
  action_map_["nothing"] = 0;
  action_map_["attach"] = 1;
  action_map_["detach"] = 2;
  group_name_ = "right_arm";
  planner_id_ = "sbpl_arm_planner";
  experiment_type_ = 1;
  trajectory_folder_path_ = "/tmp";
  use_current_state_as_start_ = false;

  rjoint_names_.push_back("r_shoulder_pan_joint");
  rjoint_names_.push_back("r_shoulder_lift_joint");
  rjoint_names_.push_back("r_upper_arm_roll_joint");
  rjoint_names_.push_back("r_elbow_flex_joint");
  rjoint_names_.push_back("r_forearm_roll_joint");
  rjoint_names_.push_back("r_wrist_flex_joint");
  rjoint_names_.push_back("r_wrist_roll_joint");
  ljoint_names_.push_back("l_shoulder_pan_joint");
  ljoint_names_.push_back("l_shoulder_lift_joint");
  ljoint_names_.push_back("l_upper_arm_roll_joint");
  ljoint_names_.push_back("l_elbow_flex_joint");
  ljoint_names_.push_back("l_forearm_roll_joint");
  ljoint_names_.push_back("l_wrist_flex_joint");
  ljoint_names_.push_back("l_wrist_roll_joint");


  benchmark_service_name_="/benchmark_planning_problem";
  world_frame_="map";
  robot_model_root_frame_="odom";
  spine_frame_="torso_lift_link";

  pviz_.setReferenceFrame("base_footprint");
  ROS_INFO("[exp] Waiting for the benchmark service..Did you start it up first?.");
  ros::service::waitForService(benchmark_service_name_);
  benchmark_client_ = nh_.serviceClient<moveit_msgs::ComputePlanningBenchmark>(benchmark_service_name_, true);
  ROS_INFO("[exp] Connected to the benchmark service.");

  pscene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1, true);
  sbpl_display_path_pub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("sbpl_path", true);
  ompl_display_path_pub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>("ompl_path", true);

  psm_ = new planning_scene_monitor::PlanningSceneMonitor("robot_description");
  pscene_.configure(psm_->getPlanningScene()->getUrdfModel(),psm_->getPlanningScene()->getSrdfModel());

}

bool BenchmarkManipulationTests::getParams()
{
  XmlRpc::XmlRpcValue plist;
  std::string p;

  ph_.param<std::string>("known_objects_filename",known_objects_filename_, "");
  ph_.param<std::string>("ompl_planner_id",ompl_planner_id_, "");
  ph_.param<std::string>("attached_object_filename",attached_object_filename_, "");
  ph_.param<std::string>("trajectory_folder_path",trajectory_folder_path_, "/tmp");
  ph_.param("apply_offset_to_collision_objects",apply_offset_to_collision_objects_,false);

  time_t clock;
  time(&clock);
  std::string time(ctime(&clock));;
  time.erase(time.size()-1, 1);
  trajectory_files_path_ = trajectory_folder_path_ + "/" + time;
  if(mkdir(trajectory_files_path_.c_str(),  S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == 0)
    ROS_INFO("Successfully created the trajectory folder: %s", trajectory_files_path_.c_str());
  else
    ROS_WARN("Failed to create the trajectory folder: %s", trajectory_files_path_.c_str());
  
  if(ph_.hasParam("goal_tolerance/xyz") && ph_.hasParam("goal_tolerance/rpy"))
  {
    ph_.getParam("goal_tolerance/xyz", plist);
    std::stringstream ss(plist);
    while(ss >> p)
      goal_tolerance_.push_back(atof(p.c_str()));

    ph_.getParam("goal_tolerance/rpy", plist);
    std::stringstream ss1(plist);
    while(ss1 >> p)
      goal_tolerance_.push_back(atof(p.c_str()));
  }
  else
  {
    goal_tolerance_.resize(6,0.02);
    goal_tolerance_[3] = 0.05;
    goal_tolerance_[4] = 0.05;
    goal_tolerance_[5] = 0.05;
  }

  printf("0\n");
  ph_.param<std::string>("benchmark_service", benchmark_service_name_, "/benchmark_planning_problem"); 
  ph_.param<std::string>("world_frame", world_frame_, "map");
  ph_.param<std::string>("robot_model_root_frame", robot_model_root_frame_, "odom");
  ph_.param<std::string>("spine_frame", spine_frame_, "torso_lift_link");
  ph_.param<std::string>("benchmark_results_folder", benchmark_results_folder_, "/tmp");
  ph_.param<std::string>("experiment_group_name", experiment_group_name_, "no_group_name");
  ph_.param<std::string>("trajectory_description_for_display", display_trajectory_description_, "plan");
  ph_.param("average_count", average_count_, 2);
  
  if(ph_.hasParam("object_pose_in_gripper"))
  {
    rarm_object_offset_.clear();
    larm_object_offset_.clear();
    ph_.getParam("object_pose_in_gripper/right/xyz", plist);
    std::stringstream ss(plist);
    while(ss >> p)
      rarm_object_offset_.push_back(atof(p.c_str()));

    ph_.getParam("object_pose_in_gripper/right/rpy", plist); 
    std::stringstream ss1(plist);
    while(ss1 >> p)
      rarm_object_offset_.push_back(atof(p.c_str()));
    
    ph_.getParam("object_pose_in_gripper/left/xyz", plist);
    std::stringstream ss2(plist);
    while(ss2 >> p)
      larm_object_offset_.push_back(atof(p.c_str()));

    ph_.getParam("object_pose_in_gripper/left/rpy", plist); 
    std::stringstream ss3(plist);
    while(ss3 >> p)
      larm_object_offset_.push_back(atof(p.c_str()));
  }
 
  printf("0.5\n");
  if(ph_.hasParam("collision_object_offset"))
  {
    collision_object_offset_.clear();
    ph_.getParam("collision_object_offset/xyz", plist);
    std::stringstream ss(plist);
    while(ss >> p)
      collision_object_offset_.push_back(atof(p.c_str()));

    ph_.getParam("collision_object_offset/rpy", plist);
    std::stringstream ss1(plist);
    while(ss1 >> p)
      collision_object_offset_.push_back(atof(p.c_str()));

    tf::Quaternion btoffset;
    collision_object_offset_pose_.position.x = collision_object_offset_[0];
    collision_object_offset_pose_.position.y = collision_object_offset_[1];
    collision_object_offset_pose_.position.z = collision_object_offset_[2];
    btoffset.setRPY(collision_object_offset_[3],collision_object_offset_[4],collision_object_offset_[5]);
    tf::quaternionTFToMsg(btoffset,collision_object_offset_pose_.orientation);
  }
  
  tf::Quaternion btoffset;
  rarm_object_pose_.position.x = rarm_object_offset_[0];
  rarm_object_pose_.position.y = rarm_object_offset_[1];
  rarm_object_pose_.position.z = rarm_object_offset_[2];
  larm_object_pose_.position.x = larm_object_offset_[0];
  larm_object_pose_.position.y = larm_object_offset_[1];
  larm_object_pose_.position.z = larm_object_offset_[2];
  btoffset.setRPY(rarm_object_offset_[3],rarm_object_offset_[4],rarm_object_offset_[5]);
  tf::quaternionTFToMsg(btoffset,rarm_object_pose_.orientation);
  btoffset.setRPY(larm_object_offset_[3],larm_object_offset_[4],larm_object_offset_[5]);
  tf::quaternionTFToMsg(btoffset,larm_object_pose_.orientation);

  printf("1\n");
  if(ph_.hasParam("use_current_pose_as_start_state"))
    ph_.getParam("use_current_pose_as_start_state", use_current_state_as_start_);
 
  if(ph_.hasParam("group_name"))
  {
    ph_.getParam("group_name", group_name_);
    if(group_name_.compare("arms") == 0)
      experiment_type_ = 2;
    else if(group_name_.compare("right_arm") == 0)
      experiment_type_ = 1;
    else
    {
      ROS_ERROR("[exp] This infrastructure only supports group_names of {'right_arm', 'arms'}. Exiting.");
      return false;
    }
  }
  else
  {
    group_name_ = "right_arm";
    experiment_type_ = 1;
  }
  
  if(ph_.hasParam("initial_start_state"))
  {
    start_pose_.rangles.clear();
    start_pose_.langles.clear();
    std::vector<double> bpose;
    ph_.getParam("initial_start_state/right", plist);
    std::stringstream ss(plist);
    while(ss >> p)
      start_pose_.rangles.push_back(atof(p.c_str()));
    ph_.getParam("initial_start_state/left", plist);
    std::stringstream ss1(plist);
    ss.str(plist);
    while(ss1 >> p)
      start_pose_.langles.push_back(atof(p.c_str()));
    ph_.getParam("initial_start_state/base", plist);
    std::stringstream ss2(plist);
    while(ss2 >> p)
      bpose.push_back(atof(p.c_str()));
    if(bpose.size() == 3)
    {
      start_pose_.body.x = bpose[0];
      start_pose_.body.y = bpose[1];
      start_pose_.body.theta = bpose[2];
    }
    ph_.getParam("initial_start_state/spine", start_pose_.body.z);  
  }

  if(ph_.hasParam("planner_interfaces"))
  {
    ph_.getParam("planner_interfaces", plist);
    std::string planner_list = std::string(plist);
    std::stringstream ss(planner_list);
    while(ss >> p)
      planner_interfaces_.push_back(p);
  }

  if(goal_tolerance_.size() < 6 || 
      start_pose_.rangles.size() < 7 ||
      start_pose_.langles.size() < 7)
  {
    ROS_ERROR("[exp] Missing some params. Either start angles for the arms or the goal tolerance.");
    return false;
  }

  if((rarm_object_offset_.size() < 6 || larm_object_offset_.size() < 6) && experiment_type_ == 2)
    return false;


  if(!getLocations() || !getExperiments())
    return false;

  return true;
}

bool BenchmarkManipulationTests::getLocations()
{
  XmlRpc::XmlRpcValue loc_list;
  geometry_msgs::Pose p;
  std::string name;
  std::string loc_name = "locations";

  if(!ph_.hasParam(loc_name))
  {
    ROS_WARN("[exp] No list of locations found on the param server.");
    return false;
  }
  ph_.getParam(loc_name, loc_list);

  if(loc_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_WARN("[exp] Location list is not an array. Something is wrong...exiting.");
    return false;
  }

  if(loc_list.size() == 0)
  {
    ROS_ERROR("[exp] List of locations is empty.");
    return false;
  }

  for(int i = 0; i < loc_list.size(); ++i)
  {
    if(!loc_list[i].hasMember("name"))
    {
      ROS_ERROR("Each location must have a name.");
      return false;
    }
    name = std::string(loc_list[i]["name"]);
    std::stringstream ss(loc_list[i]["pose"]);
    std::string p;
    while(ss >> p)
      loc_map_[name].push_back(atof(p.c_str()));
  }

  ROS_INFO("[exp] Successfully fetched %d locations from param server.", int(loc_list.size()));
  return true;
}

bool BenchmarkManipulationTests::getExperiments()
{
  XmlRpc::XmlRpcValue exp_list;
  Experiment e;
  std::string exp_name = "experiments";
  XmlRpc::XmlRpcValue plist;
  std::string p;

  if(!ph_.hasParam(exp_name))
  {
    ROS_WARN("No list of experiments found on the param server.");
    return false;
  }
  ph_.getParam(exp_name, exp_list);

  if(exp_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_WARN("Experiment list is not an array. Something is wrong...exiting.");
    return false;
  }

  if(exp_list.size() == 0)
  {
    ROS_ERROR("List of experiments is empty.");
    return false;
  }

  for(int i = 0; i < exp_list.size(); ++i)
  {
    if(!exp_list[i].hasMember("name"))
    {
      ROS_ERROR("Each experiment must have a name.");
      return false;
    }
    e.name = std::string(exp_list[i]["name"]);

    if(!exp_list[i].hasMember("goal"))
    {
      ROS_ERROR("Each experiment must have a goal....duh.");
      return false;
    }
    e.goal = std::string(exp_list[i]["goal"]);

    if(!exp_list[i].hasMember("pre_action"))
      e.pre_action = 0;
    else
      e.pre_action = action_map_[exp_list[i]["pre_action"]];

    if(!exp_list[i].hasMember("post_action"))
      e.post_action = 0;
    else
      e.post_action = action_map_[exp_list[i]["post_action"]];

    if(!exp_list[i].hasMember("sound_bite"))
      e.sound_bite = "";
    else
      e.sound_bite = std::string(exp_list[i]["sound_bite"]);

    if(exp_list[i].hasMember("start"))
    {
      e.start.rangles.clear();
      e.start.langles.clear();
      std::vector<double> bpose;
      plist = exp_list[i]["start"]["right"];
      std::stringstream ss(plist);
      while(ss >> p)
        e.start.rangles.push_back(atof(p.c_str()));
      
      plist = exp_list[i]["start"]["left"];
      std::stringstream ss1(plist);
      while(ss1 >> p)
        e.start.langles.push_back(atof(p.c_str()));
      
      plist = exp_list[i]["start"]["base"];
      std::stringstream ss2(plist);
      while(ss2 >> p)
        bpose.push_back(atof(p.c_str()));
      if(bpose.size() == 3)
      {
        e.start.body.x = bpose[0];
        e.start.body.y = bpose[1];
        e.start.body.theta = bpose[2];
      }
      e.start.body.z = double(exp_list[i]["start"]["spine"]);
    }
    else
    {
      if(!use_current_state_as_start_)
      {
        ROS_ERROR("[exp] No start state defined for %s and it isn't configured to use the current state as the start state.",e.name.c_str());
        return false; 
      }
      else  
        ROS_DEBUG("No start state defined for %s but it's OK because it's configured to use the current state as the start.",e.name.c_str());
    }

    ROS_DEBUG("Adding experiment: %s", e.name.c_str());
    exp_map_[e.name] = e;
  }

  return true;  
}

bool BenchmarkManipulationTests::getCollisionObjects(std::string filename, std::vector<moveit_msgs::CollisionObject> &collision_objects)
{
  int num_obs;
  char sTemp[1024];
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray marker_array;
  std::vector<std::vector<double> > objects, object_colors;
  std::vector<std::string> object_ids;
  moveit_msgs::CollisionObject object;
  collision_objects.clear();

  FILE* fCfg = fopen(filename.c_str(), "r");
  if(fCfg == NULL)
    return false;

  // get number of objects
  if(fscanf(fCfg,"%d",&num_obs) < 1)
    ROS_INFO("[exp] Parsed string has length < 1.(number of obstacles)\n");

  ROS_INFO("[exp] Parsing collision object file with %i objects.",num_obs);

  //get {x y z dimx dimy dimz} for each object
  objects.resize(num_obs, std::vector<double>(6,0.0));
  object_colors.resize(num_obs, std::vector<double>(4,0.0));
  object_ids.clear();
  for (int i=0; i < num_obs; ++i)
  {
    if(fscanf(fCfg,"%s",sTemp) < 1)
      ROS_INFO("[exp] Parsed string has length < 1.\n");
    object_ids.push_back(sTemp);

    for(int j=0; j < 6; ++j)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_INFO("[exp] Parsed string has length < 1. (object parameters for %s)", object_ids.back().c_str());
      if(!feof(fCfg) && strlen(sTemp) != 0)
        objects[i][j] = atof(sTemp);
    }
    for(int j=0; j < 4; ++j)
    {
      if(fscanf(fCfg,"%s",sTemp) < 1)
        ROS_INFO("[exp] Parsed string has length < 1. (object colors for %s)", object_ids.back().c_str());
      if(!feof(fCfg) && strlen(sTemp) != 0)
        object_colors[i][j] = atof(sTemp);
    }
  }

  if(object_ids.size() != objects.size())
  {
    ROS_INFO("object id list is not same length as object list. exiting.");
    return false;
  }

  //pviz_.visualizeObstacles(objects);
  object.shapes.resize(1);
  object.poses.resize(1);
  object.shapes[0].dimensions.resize(3);
  //object.shapes[0].triangles.resize(4);
  for(size_t i = 0; i < objects.size(); i++)
  {
    object.id = object_ids[i];
    object.operation = moveit_msgs::CollisionObject::ADD;
    object.shapes[0].type = moveit_msgs::Shape::BOX;
    object.header.frame_id = "base_footprint";
    object.header.stamp = ros::Time::now();

    object.poses[0].position.x = objects[i][0];
    object.poses[0].position.y = objects[i][1];
    object.poses[0].position.z = objects[i][2];
    object.poses[0].orientation.x = 0; 
    object.poses[0].orientation.y = 0; 
    object.poses[0].orientation.z = 0; 
    object.poses[0].orientation.w = 1;  

    object.shapes[0].dimensions[0] = objects[i][3];
    object.shapes[0].dimensions[1] = objects[i][4];
    object.shapes[0].dimensions[2] = objects[i][5];

    // apply collision object offset
    // right now just translates and rotates about z
    if(apply_offset_to_collision_objects_)
    {
      if(!collision_object_offset_.empty()) 
      {
        geometry_msgs::Pose p, p2;
        Eigen::Affine3d a;
        a(0,0) = cos(collision_object_offset_[5]);
        a(1,0) = sin(collision_object_offset_[5]);
        a(0,1) = -sin(collision_object_offset_[5]);
        a(1,1) = cos(collision_object_offset_[5]);
        planning_models::msgFromPose(a, p2);
        multiplyPoses(p2, object.poses[0], p);
        p.position.x += collision_object_offset_pose_.position.x; 
        p.position.y += collision_object_offset_pose_.position.y; 
        p.position.z += collision_object_offset_pose_.position.z; 
        object.poses[0] = p;
      }
      else
        ROS_ERROR("[exp] Expecting to translate/rotate collision objects in robot frame but offset not found.");
    }

    collision_objects.push_back(object);
    ROS_DEBUG("[exp] [%d] id: %s xyz: %0.3f %0.3f %0.3f dims: %0.3f %0.3f %0.3f colors: %2.0f %2.0f %2.0f %2.0f",int(i),object_ids[i].c_str(),objects[i][0],objects[i][1],objects[i][2],objects[i][3],objects[i][4],objects[i][5], object_colors[i][0], object_colors[i][1], object_colors[i][2], object_colors[i][3]);

    std::vector<double> dim(3,0);
    dim[0] = objects[i][3];
    dim[1] = objects[i][4];
    dim[2] = objects[i][5];
    pviz_.getCubeMsg(object.poses[0], dim, object_colors[i], "collision_objects", int(i), marker);
    marker_array.markers.push_back(marker);
  }

  ROS_INFO("[exp] I gathered %d collision cubes", int(marker_array.markers.size()));
  pviz_.publishMarkerArray(marker_array);
  usleep(500);
  return true;
}

bool BenchmarkManipulationTests::getAttachedObject(std::string object_file, geometry_msgs::Pose rarm_object_pose, moveit_msgs::AttachedCollisionObject &att_object)
{
  char sTemp[1024];
  float temp[6];
  tf::Quaternion q;
  att_object.link_name = "r_wrist_roll_link";
  att_object.touch_links.push_back("r_gripper");
  att_object.touch_links.push_back("l_gripper");
  att_object.touch_links.push_back("r_gripper_palm_link");
  att_object.touch_links.push_back("r_gripper_r_finger_link");
  att_object.touch_links.push_back("r_gripper_l_finger_link");
  att_object.touch_links.push_back("r_gripper_l_finger_tip_link");
  att_object.touch_links.push_back("r_gripper_r_finger_tip_link");
  att_object.touch_links.push_back("r_wrist_roll_link");
  att_object.touch_links.push_back("l_gripper_palm_link");
  att_object.touch_links.push_back("l_gripper_r_finger_link");
  att_object.touch_links.push_back("l_gripper_l_finger_link");
  att_object.touch_links.push_back("l_gripper_l_finger_tip_link");
  att_object.touch_links.push_back("l_gripper_r_finger_tip_link");
  att_object.touch_links.push_back("l_wrist_roll_link");
  att_object.object.header.frame_id = "r_wrist_roll_link";
  att_object.object.operation = moveit_msgs::CollisionObject::ADD;
  att_object.object.header.stamp = ros::Time::now();
  att_object.object.shapes.resize(1);
  att_object.object.shapes[0].type = moveit_msgs::Shape::BOX;
  att_object.object.poses.resize(1);

  FILE* fid = fopen(object_file.c_str(), "r");
  if(fid == NULL)
  {
    ROS_ERROR("[exp] Failed to open object file. (%s)", object_file.c_str());
    return false;
  }

  // object name
  if(fscanf(fid,"%s",sTemp) < 1)
    ROS_WARN("Parsed string has length < 1. (%s)", sTemp);
  att_object.object.id = sTemp;
  // xyz in r_wrist_roll_link
  if(fscanf(fid,"%s",sTemp) < 1)
    ROS_WARN("Parsed string has length < 1. (%s)", sTemp); 
  if(strcmp(sTemp, "xyz:") == 0)
  {
    if(fscanf(fid,"%f %f %f",&(temp[0]),&(temp[1]),&(temp[2])) < 1)
      ROS_WARN("Failed to parse xyz.");
    att_object.object.poses[0].position.x = temp[0];
    att_object.object.poses[0].position.y = temp[1];
    att_object.object.poses[0].position.z = temp[2];
    ROS_DEBUG("xyz: %0.3f %0.3f %0.3f", temp[0], temp[1], temp[2]);
  }
  // rpy in r_wrist_roll_link
  if(fscanf(fid,"%s",sTemp) < 1)
    ROS_WARN("Parsed string has length < 1. (%s)", sTemp); 
  if(strcmp(sTemp, "rpy:") == 0)
  {
    if(fscanf(fid,"%f %f %f",&(temp[0]),&(temp[1]),&(temp[2])) < 1)
      ROS_WARN("Failed to parse xyz.");
    q.setRPY(temp[0],temp[1],temp[2]);
    tf::quaternionTFToMsg(q, att_object.object.poses[0].orientation);
    ROS_DEBUG("rpy: %0.3f %0.3f %0.3f", temp[0], temp[1], temp[2]);
  }
  // dims
  if(fscanf(fid,"%s",sTemp) < 1)
    ROS_WARN("Parsed string has length < 1. (%s)", sTemp); 
  if(strcmp(sTemp, "dims:") == 0)
  {
    if(fscanf(fid,"%f %f %f",&(temp[0]),&(temp[1]),&(temp[2])) < 1)
      ROS_WARN("Failed to parse dims.");
    att_object.object.shapes[0].dimensions.resize(3,0);
    att_object.object.shapes[0].dimensions[0] = temp[0];
    att_object.object.shapes[0].dimensions[1] = temp[1];
    att_object.object.shapes[0].dimensions[2] = temp[2];  
    ROS_DEBUG("dims: %0.3f %0.3f %0.3f", temp[0], temp[1], temp[2]);
  }
  return true;
}

void BenchmarkManipulationTests::visualizeEnvironment()
{
  moveit_msgs::ComputePlanningBenchmark::Request req;
  moveit_msgs::ComputePlanningBenchmark::Response res;

  req.average_count = average_count_;
  psm_->getPlanningScene()->getAllowedCollisionMatrix().getMessage(req.scene.allowed_collision_matrix);

  req.scene.robot_state.joint_state.header.frame_id = robot_model_root_frame_;
  req.scene.robot_state.joint_state.header.stamp = ros::Time::now();
  req.scene.robot_state.joint_state.name.resize(1);
  req.scene.robot_state.joint_state.name[0] = "torso_lift_joint";
  req.scene.robot_state.joint_state.position.resize(1);
  req.scene.robot_state.joint_state.position[0] = start_pose_.body.z;

  // fill in collision objects
  if(!known_objects_filename_.empty())
  {
    if(!getCollisionObjects(known_objects_filename_, req.scene.world.collision_objects))
    {
      ROS_ERROR("[exp] Failed to get the collision objects from the file.");
      return;
    }
  }

  ROS_INFO("set planning scene");
  pscene_.setPlanningSceneMsg(req.scene);
  ROS_INFO("get kinematic model");
  if(!initKinematicSolver(pscene_.getKinematicModel()))
  {
    ROS_ERROR("[exp] Failed to initialize the kinematic solver.");
    return;
  }

  // start state
  for(size_t i = 0; i < start_pose_.rangles.size(); ++i)
    req.motion_plan_request.start_state.joint_state.position.push_back(start_pose_.rangles[i]);
  for(size_t i = 0; i < start_pose_.langles.size(); ++i)
    req.motion_plan_request.start_state.joint_state.position.push_back(start_pose_.langles[i]);

  req.motion_plan_request.start_state.joint_state.name.push_back("r_shoulder_pan_joint");
  req.motion_plan_request.start_state.joint_state.name.push_back("r_shoulder_lift_joint");
  req.motion_plan_request.start_state.joint_state.name.push_back("r_upper_arm_roll_joint");
  req.motion_plan_request.start_state.joint_state.name.push_back("r_elbow_flex_joint");
  req.motion_plan_request.start_state.joint_state.name.push_back("r_forearm_roll_joint");
  req.motion_plan_request.start_state.joint_state.name.push_back("r_wrist_flex_joint");
  req.motion_plan_request.start_state.joint_state.name.push_back("r_wrist_roll_joint");
  req.motion_plan_request.start_state.joint_state.name.push_back("l_shoulder_pan_joint");
  req.motion_plan_request.start_state.joint_state.name.push_back("l_shoulder_lift_joint");
  req.motion_plan_request.start_state.joint_state.name.push_back("l_upper_arm_roll_joint");
  req.motion_plan_request.start_state.joint_state.name.push_back("l_elbow_flex_joint");
  req.motion_plan_request.start_state.joint_state.name.push_back("l_forearm_roll_joint");
  req.motion_plan_request.start_state.joint_state.name.push_back("l_wrist_flex_joint");
  req.motion_plan_request.start_state.joint_state.name.push_back("l_wrist_roll_joint");
  req.motion_plan_request.start_state.joint_state.position.push_back(start_pose_.body.z);
  req.motion_plan_request.start_state.joint_state.name.push_back("torso_lift_joint");


  // filling planning scene with start state of joints so the rviz plugin 
  // can display them
  for(size_t i = 0; i < req.motion_plan_request.start_state.joint_state.name.size(); ++i)
  {
    req.scene.robot_state.joint_state.name.push_back(req.motion_plan_request.start_state.joint_state.name[i]);
    req.scene.robot_state.joint_state.position.push_back(req.motion_plan_request.start_state.joint_state.position[i]);
  }
  ROS_INFO("[exp] Publishing the planning scene for visualization using the motion_planning_rviz_plugin.");
  pscene_pub_.publish(req.scene);
  visualizeRobotPose(start_pose_, "start", 0);
}

bool BenchmarkManipulationTests::requestPlan(RobotPose &start_state, std::string name)
{
  moveit_msgs::ComputePlanningBenchmark::Request req;
  moveit_msgs::ComputePlanningBenchmark::Response res;

  req.average_count = average_count_;
  req.filename = benchmark_results_folder_ + "/" + experiment_group_name_ + "/" + name + ".log";
  psm_->getPlanningScene()->getAllowedCollisionMatrix().getMessage(req.scene.allowed_collision_matrix);

  req.scene.name = name;
  req.scene.robot_state.joint_state.header.frame_id = robot_model_root_frame_;
  req.scene.robot_state.joint_state.header.stamp = ros::Time::now();
  req.scene.robot_state.joint_state.name.resize(1);
  req.scene.robot_state.joint_state.name[0] = "torso_lift_joint";
  req.scene.robot_state.joint_state.position.resize(1);
  req.scene.robot_state.joint_state.position[0] = start_state.body.z;

  // fill in collision objects
  if(!known_objects_filename_.empty())
  {
    if(!getCollisionObjects(known_objects_filename_, req.scene.world.collision_objects))
    {
      ROS_ERROR("[exp] Failed to get the collision objects from the file.");
      return false;
    }
  }

  // fill in attached object
  if(exp_map_[name].pre_action == action_map_["attach"])
  {
    req.scene.attached_collision_objects.resize(1);
    if(!getAttachedObject(attached_object_filename_, rarm_object_pose_, req.scene.attached_collision_objects[0]))
    {
      ROS_ERROR("[exp] Failed to add the attached object.");
      return false;
    }
  }
  else if(exp_map_[name].pre_action == action_map_["detach"])
  {
    req.scene.attached_collision_objects.resize(1);
    if(!getAttachedObject(attached_object_filename_, rarm_object_pose_, req.scene.attached_collision_objects[0]))
    {
      ROS_ERROR("[exp] Failed to remove the attached object.");
      return false;
    }
    req.scene.attached_collision_objects[0].object.operation = moveit_msgs::CollisionObject::REMOVE;
  }
  ROS_INFO("set planning scene");
  pscene_.setPlanningSceneMsg(req.scene);
  ROS_INFO("get kinematic model");
  if(!initKinematicSolver(pscene_.getKinematicModel()))
  {
    ROS_ERROR("[exp] Failed to initialize the kinematic solver.");
    return false;
  }

  // fill in goal
  if(experiment_type_ == 1)
    fillSingleArmPlanningRequest(start_state,name,req.motion_plan_request);
  else
    fillDualArmPlanningRequest(start_state,name,req.motion_plan_request);

  // filling planning scene with start state of joints so the rviz plugin can display them
  for(size_t i = 0; i < req.motion_plan_request.start_state.joint_state.name.size(); ++i)
  {
    req.scene.robot_state.joint_state.name.push_back(req.motion_plan_request.start_state.joint_state.name[i]);
    req.scene.robot_state.joint_state.position.push_back(req.motion_plan_request.start_state.joint_state.position[i]);
  }
  ROS_INFO("[exp] Publishing the planning scene for visualization using the motion_planning_rviz_plugin.");
  pscene_pub_.publish(req.scene);
  visualizeRobotPose(start_state, "start", 0);
  sleep(0.5);
  if(benchmark_client_.call(req, res))
  {
    ROS_INFO("[exp] Planner returned status code: %d.", res.error_code.val); 
  }
  else
  {
    ROS_ERROR("[exp] Planning service failed to respond. Exiting.");
    return false;
  }

  for(size_t i = 0; i < res.responses.size(); ++i)
  {
    for(size_t j = 0; j < res.responses[i].trajectory.size(); ++j)
      ROS_INFO("[exp] %s returned a path with %d waypoints with description, '%s'.", res.planner_interfaces[i].c_str(), int(res.responses[i].trajectory[j].joint_trajectory.points.size()), res.responses[i].description[j].c_str());
  }

  ROS_INFO("[exp] Visualizing trajectories using motion planning rviz plugin."); 
  visualizeTrajectories(res);
  sleep(2);

  ROS_INFO("[exp] Setting current pose using the sbpl path");
  if(res.responses.size() > 1)
  {
    if(res.responses[1].trajectory.size() > 1)
    {
      if(!res.responses[1].trajectory[1].joint_trajectory.points.empty())
      {
        if(!setRobotPoseFromTrajectory(res.responses[1].trajectory[1], res.responses[1].trajectory_start, current_pose_))
        {
          ROS_ERROR("[exp] Failed to set the current robot pose from the trajectory found.");
          return false;
        }
      }
    }
  }


  ROS_INFO("[exp] Recording trajectories to file...");
  if(!writeTrajectoriesToFile(res,name))
  {
    ROS_ERROR("[exp] Failed to write the trajectories to file.");
    return false;
  }
  ROS_INFO("[exp] Planning request was a success.");
  return true;
}

bool BenchmarkManipulationTests::runExperiment(std::string name)
{
  bool is_first_exp_ = false;
  RobotPose start;
  std::vector<std::vector<double> > traj;
 
  printf("************** %s **************\n", name.c_str()); 

  std::map<std::string, Experiment>::iterator name_iter = exp_map_.find(name);
  if(std::distance(exp_map_.begin(), name_iter) == 0)
    is_first_exp_ = true;

  if(!use_current_state_as_start_ && is_first_exp_)
  {
    if(exp_map_[name].start.rangles.empty())
      start = start_pose_;
    else
      start = exp_map_[name].start;
  }
  else if(use_current_state_as_start_)
  {
    if(is_first_exp_)
      start = start_pose_;
    else
      start = current_pose_;
  }
  else
    start = exp_map_[name].start;
  
  current_pose_ = start;

  if(use_current_state_as_start_)
    writeCompleteExperimentFile(exp_map_[name],start);

  printRobotPose(start, "start");
  ROS_INFO("[exp]  goal: %s", exp_map_[name].goal.c_str());
  if(!requestPlan(start, name))
  {
    ROS_ERROR("[exp] %s failed to plan.", name.c_str());
    return false; 
  }
  else
    ROS_INFO("[exp] It's a planning miracle!");

  visualizeLocations();
  return true;
}

bool BenchmarkManipulationTests::performAllExperiments()
{
  if(use_current_state_as_start_)
    startCompleteExperimentFile();

  if(!createExperimentGroupFolder(experiment_group_name_))
  {
    ROS_ERROR("[exp] Failed to create experiment group folder for %s", experiment_group_name_.c_str());
    return false;
  }

  for(std::map<std::string,Experiment>::iterator iter = exp_map_.begin(); iter != exp_map_.end(); ++iter)
  {
    if(!runExperiment(iter->first))
      return false;
  
    visualizeRobotPose(current_pose_, "start", 0);
  }

  return true;
}

void BenchmarkManipulationTests::visualizeStartPose()
{
  visualizeRobotPose(start_pose_, "start", 0);
}

void BenchmarkManipulationTests::visualizeRobotPose(RobotPose &pose, std::string name, int id)
{
  pviz_.visualizeRobot(pose.rangles, pose.langles, pose.body, 120, name, id);
}

void BenchmarkManipulationTests::visualizeLocations()
{
  std::vector<std::vector<double> > poses;
  poses.resize(std::distance(exp_map_.begin(),exp_map_.end()));
  for(std::map<std::string,Experiment >::iterator iter = exp_map_.begin(); iter != exp_map_.end(); ++iter)
  {
    int i = std::distance(exp_map_.begin(), iter);
    poses[i].resize(6,0.0);
    poses[i][0] = loc_map_[iter->second.goal].at(0);
    poses[i][1] = loc_map_[iter->second.goal].at(1);
    poses[i][2] = loc_map_[iter->second.goal].at(2);
    poses[i][3] = loc_map_[iter->second.goal].at(3);
    poses[i][4] = loc_map_[iter->second.goal].at(4);
    poses[i][5] = loc_map_[iter->second.goal].at(5);
  }

  ROS_INFO("[exp] Visualizing %d locations.", int(poses.size()));
  pviz_.visualizePoses(poses);
}

void BenchmarkManipulationTests::startCompleteExperimentFile(){
  FILE* fout = fopen("/tmp/completeExperiment.yaml","w");
  /*
  fprintf(fout,"goal_tolerance:\n  xyz: 0.02 0.02 0.02\n rpy: 0.05 0.05 0.05\n\n");
  fprintf(fout,"object_pose_in_gripper:\n  right:\n    xyz: -0.20 -0.1 0.0\n    rpy: 0.0 0.0 0.0\n  left:\n    xyz: -0.20 0.1 0.0\n    rpy: 0.0 0.0 0.0\n\n");
  fprintf(fout,"use_current_pose_as_start_state: false\n");
  fprintf(fout,"start_state:\n  right: 0.0 0.018 0.00 -0.43 0.00 -0.00 0.0\n  left: 0.0 0.018 0.00 -0.43 0.00 -0.00 0.0\n  base: 1.9 0.6 1.57\n  spine: 0.0\n\n");
  */
  fprintf(fout,"experiments:\n\n");
  fclose(fout);
}

void BenchmarkManipulationTests::writeCompleteExperimentFile(Experiment e, RobotPose start){
  FILE* fout = fopen("/tmp/completeExperiment.yaml","a");
  fprintf(fout,"  - name: %s\n",e.name.c_str());
  fprintf(fout,"    goal: %s\n",e.goal.c_str());
  if(e.pre_action==0)
    fprintf(fout,"    pre_action: nothing\n");
  else if(e.pre_action==1)
    fprintf(fout,"    pre_action: attach\n");
  else if(e.pre_action==2)
    fprintf(fout,"    pre_action: detach\n");
  if(e.post_action==0)
    fprintf(fout,"    post_action: nothing\n");
  else if(e.post_action==1)
    fprintf(fout,"    post_action: attach\n");
  else if(e.post_action==2)
    fprintf(fout,"    post_action: detach\n");
  fprintf(fout,"    sound_bite: \"%s\"\n",e.sound_bite.c_str());
  fprintf(fout,"    start:\n");
  fprintf(fout,"      right: %f %f %f %f %f %f %f\n",start.rangles[0],start.rangles[1],start.rangles[2],start.rangles[3],start.rangles[4],start.rangles[5],start.rangles[6]);
  fprintf(fout,"      left: %f %f %f %f %f %f %f\n",start.langles[0],start.langles[1],start.langles[2],start.langles[3],start.langles[4],start.langles[5],start.langles[6]);
  fprintf(fout,"      base: %f %f %f\n",start.body.x,start.body.y,start.body.theta);
  fprintf(fout,"      spine: %f\n",start.body.z);
  fclose(fout);
}

void BenchmarkManipulationTests::printLocations()
{
  if(loc_map_.begin() == loc_map_.end())
  {
    ROS_ERROR("[exp] No locations found.");
    return;
  }
  for(std::map<std::string,std::vector<double> >::const_iterator iter = loc_map_.begin(); iter != loc_map_.end(); ++iter)
  {
    ROS_INFO("name: %s", iter->first.c_str());
    ROS_INFO("  x: % 0.3f  y: % 0.3f  z: % 0.3f  roll: %0.3f  pitch: %0.3f  yaw: %0.3f", iter->second.at(0), iter->second.at(1), iter->second.at(2), iter->second.at(3), iter->second.at(4), iter->second.at(5));
  }
}

void BenchmarkManipulationTests::printExperiments()
{
  if(exp_map_.begin() == exp_map_.end())
  {
    ROS_ERROR("[exp] No experiments found.");
    return;
  }
  for(std::map<std::string,Experiment>::iterator iter = exp_map_.begin(); iter != exp_map_.end(); ++iter)
  {
    int p = std::distance(exp_map_.begin(), iter);
    ROS_INFO("------------------------------");
    ROS_INFO("[%d] name: %s", p, iter->second.name.c_str());
    ROS_INFO("[%d] goal: %s", p, iter->second.goal.c_str());
    ROS_INFO("[%d] pre_action: %s", p, action_list_[iter->second.pre_action].c_str());
    ROS_INFO("[%d] post_action: %s", p, action_list_[iter->second.post_action].c_str());
    ROS_INFO("[%d] sound_bite: %s", p, iter->second.sound_bite.c_str());
    if(!use_current_state_as_start_)
    {
      ROS_INFO("[%d] start:", p);
      ROS_INFO("[%d]   right: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", p, iter->second.start.rangles[0], iter->second.start.rangles[1], iter->second.start.rangles[2], iter->second.start.rangles[3], iter->second.start.rangles[4], iter->second.start.rangles[5], iter->second.start.rangles[6]);
      ROS_INFO("[%d]    left: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", p, iter->second.start.langles[0], iter->second.start.langles[1], iter->second.start.langles[2], iter->second.start.langles[3], iter->second.start.langles[4], iter->second.start.langles[5], iter->second.start.langles[6]);
      ROS_INFO("[%d]    base: % 0.3f % 0.3f % 0.3f", p, iter->second.start.body.x, iter->second.start.body.y, iter->second.start.body.theta);
      ROS_INFO("[%d]   torso: % 0.3f", p, iter->second.start.body.z);
    }
  }
}

void BenchmarkManipulationTests::printRobotPose(RobotPose &pose, std::string name)
{
  if(pose.rangles.size() < 7 || pose.langles.size() < 7)
  {
    ROS_ERROR("[exp] Trying to print RobotPose but size of rangles = %d and size of langles = %d.", int(pose.rangles.size()), int(pose.langles.size()));
    return;
  }
  ROS_INFO("[%s] right: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", name.c_str(), pose.rangles[0], pose.rangles[1], pose.rangles[2], pose.rangles[3], pose.rangles[4], pose.rangles[5], pose.rangles[6]);
  ROS_INFO("[%s]  left: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", name.c_str(), pose.langles[0], pose.langles[1], pose.langles[2], pose.langles[3], pose.langles[4], pose.langles[5], pose.langles[6]);
  ROS_INFO("[%s]  base: % 0.3f % 0.3f % 0.3f", name.c_str(), pose.body.x, pose.body.y, pose.body.theta);
  ROS_INFO("[%s] torso: % 0.3f", name.c_str(), pose.body.z);
}

void BenchmarkManipulationTests::printParams()
{
  for(size_t i = 0; i < planner_interfaces_.size(); ++i)
    ROS_INFO("[planner_interface] %d/%d %s", int(i+1), int(planner_interfaces_.size()), planner_interfaces_[i].c_str());

  ROS_INFO("     [goal tolerance] x: % 0.3f  y: % 0.3f  z: % 0.3f  roll: % 0.3f  pitch: %0.3f  yaw: %0.3f", goal_tolerance_[0], goal_tolerance_[1], goal_tolerance_[2], goal_tolerance_[3], goal_tolerance_[4], goal_tolerance_[5]);
  ROS_INFO("  [right object pose] x: % 0.3f  y: % 0.3f  z: % 0.3f  r: % 0.3f  p: % 0.3f  y: % 0.3f", rarm_object_offset_[0], rarm_object_offset_[1], rarm_object_offset_[2], rarm_object_offset_[3], rarm_object_offset_[4], rarm_object_offset_[5]);
  ROS_INFO("   [left object pose] x: % 0.3f  y: % 0.3f  z: % 0.3f  r: % 0.3f  p: % 0.3f  y: % 0.3f", larm_object_offset_[0], larm_object_offset_[1], larm_object_offset_[2], larm_object_offset_[3], larm_object_offset_[4], larm_object_offset_[5]);
  ROS_INFO("[initial start state] right: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", start_pose_.rangles[0], start_pose_.rangles[1], start_pose_.rangles[2], start_pose_.rangles[3], start_pose_.rangles[4], start_pose_.rangles[5], start_pose_.rangles[6]);
  ROS_INFO("[initial start state]  left: % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f % 0.3f", start_pose_.langles[0], start_pose_.langles[1], start_pose_.langles[2], start_pose_.langles[3], start_pose_.langles[4], start_pose_.langles[5], start_pose_.langles[6]);
  ROS_INFO("[initial start state]  base: % 0.3f % 0.3f % 0.3f", start_pose_.body.x, start_pose_.body.y, start_pose_.body.theta);
  ROS_INFO("[initial start state] torso: % 0.3f", start_pose_.body.z);

  printLocations();
  printExperiments();
}

void BenchmarkManipulationTests::fillSingleArmPlanningRequest(RobotPose &start_state, std::string name, moveit_msgs::MotionPlanRequest &req)
{
  tf::Quaternion q;
  req.group_name = group_name_;
  req.num_planning_attempts = 1;
  req.allowed_planning_time = ros::Duration(60.0);
  req.planner_id = ompl_planner_id_;

  // start state
  for(size_t i = 0; i < start_state.rangles.size(); ++i)
    req.start_state.joint_state.position.push_back(start_state.rangles[i]);
  for(size_t i = 0; i < start_state.langles.size(); ++i)
    req.start_state.joint_state.position.push_back(start_state.langles[i]);

  req.start_state.joint_state.name.push_back("r_shoulder_pan_joint");
  req.start_state.joint_state.name.push_back("r_shoulder_lift_joint");
  req.start_state.joint_state.name.push_back("r_upper_arm_roll_joint");
  req.start_state.joint_state.name.push_back("r_elbow_flex_joint");
  req.start_state.joint_state.name.push_back("r_forearm_roll_joint");
  req.start_state.joint_state.name.push_back("r_wrist_flex_joint");
  req.start_state.joint_state.name.push_back("r_wrist_roll_joint");
  req.start_state.joint_state.name.push_back("l_shoulder_pan_joint");
  req.start_state.joint_state.name.push_back("l_shoulder_lift_joint");
  req.start_state.joint_state.name.push_back("l_upper_arm_roll_joint");
  req.start_state.joint_state.name.push_back("l_elbow_flex_joint");
  req.start_state.joint_state.name.push_back("l_forearm_roll_joint");
  req.start_state.joint_state.name.push_back("l_wrist_flex_joint");
  req.start_state.joint_state.name.push_back("l_wrist_roll_joint");
  req.start_state.joint_state.position.push_back(start_state.body.z);
  req.start_state.joint_state.name.push_back("torso_lift_joint");

  // goal pose
  req.goal_constraints.resize(1);
  req.goal_constraints[0].position_constraints.resize(1);
  req.goal_constraints[0].position_constraints[0].constraint_region_pose.header.stamp = ros::Time::now();
  req.goal_constraints[0].position_constraints[0].constraint_region_pose.header.frame_id = "odom";

  req.goal_constraints[0].position_constraints[0].link_name = "r_wrist_roll_link";
  req.goal_constraints[0].position_constraints[0].constraint_region_pose.pose.position.x = loc_map_[exp_map_[name].goal].at(0);
  req.goal_constraints[0].position_constraints[0].constraint_region_pose.pose.position.y = loc_map_[exp_map_[name].goal].at(1);
  req.goal_constraints[0].position_constraints[0].constraint_region_pose.pose.position.z = loc_map_[exp_map_[name].goal].at(2);
  req.goal_constraints[0].position_constraints[0].constraint_region_pose.pose.orientation.w = 1;

  req.goal_constraints[0].position_constraints[0].constraint_region_shape.type = moveit_msgs::Shape::BOX;
  req.goal_constraints[0].position_constraints[0].constraint_region_shape.dimensions.push_back(goal_tolerance_[0]);
  req.goal_constraints[0].position_constraints[0].constraint_region_shape.dimensions.push_back(goal_tolerance_[1]);
  req.goal_constraints[0].position_constraints[0].constraint_region_shape.dimensions.push_back(goal_tolerance_[2]);
  req.goal_constraints[0].position_constraints[0].weight = 1.0;

  req.goal_constraints[0].orientation_constraints.resize(1);
  req.goal_constraints[0].orientation_constraints[0].orientation.header.stamp = ros::Time::now();
  req.goal_constraints[0].orientation_constraints[0].orientation.header.frame_id = "odom";
  req.goal_constraints[0].orientation_constraints[0].link_name = "r_wrist_roll_link";

  q.setRPY(loc_map_[exp_map_[name].goal].at(3), loc_map_[exp_map_[name].goal].at(4), loc_map_[exp_map_[name].goal].at(5));
  tf::quaternionTFToMsg(q, req.goal_constraints[0].orientation_constraints[0].orientation.quaternion);

  req.goal_constraints[0].orientation_constraints[0].absolute_x_axis_tolerance = goal_tolerance_[3];
  req.goal_constraints[0].orientation_constraints[0].absolute_y_axis_tolerance = goal_tolerance_[4];
  req.goal_constraints[0].orientation_constraints[0].absolute_z_axis_tolerance = goal_tolerance_[5];
  req.goal_constraints[0].orientation_constraints[0].weight = 1.0;

  ROS_INFO("[exp] [goal] rpy: %0.3f %0.3f %0.3f  quat: %0.3f %0.3f %0.3f %0.3f", loc_map_[exp_map_[name].goal].at(3), loc_map_[exp_map_[name].goal].at(4), loc_map_[exp_map_[name].goal].at(5), req.goal_constraints[0].orientation_constraints[0].orientation.quaternion.x, req.goal_constraints[0].orientation_constraints[0].orientation.quaternion.y, req.goal_constraints[0].orientation_constraints[0].orientation.quaternion.z, req.goal_constraints[0].orientation_constraints[0].orientation.quaternion.w);
}

void BenchmarkManipulationTests::fillDualArmPlanningRequest(RobotPose &start_state, std::string name, moveit_msgs::MotionPlanRequest &req)
{
  tf::Quaternion q;
  req.group_name = group_name_;
  req.num_planning_attempts = 1;
  req.allowed_planning_time = ros::Duration(60.0);
  req.planner_id = planner_id_;

  // start state
  for(size_t i = 0; i < start_state.rangles.size(); ++i)
    req.start_state.joint_state.position.push_back(start_state.rangles[i]);
  for(size_t i = 0; i < start_state.langles.size(); ++i)
    req.start_state.joint_state.position.push_back(start_state.langles[i]);

  req.start_state.joint_state.name.push_back("r_shoulder_pan_joint");
  req.start_state.joint_state.name.push_back("r_shoulder_lift_joint");
  req.start_state.joint_state.name.push_back("r_upper_arm_roll_joint");
  req.start_state.joint_state.name.push_back("r_elbow_flex_joint");
  req.start_state.joint_state.name.push_back("r_forearm_roll_joint");
  req.start_state.joint_state.name.push_back("r_wrist_flex_joint");
  req.start_state.joint_state.name.push_back("r_wrist_roll_joint");
  req.start_state.joint_state.name.push_back("l_shoulder_pan_joint");
  req.start_state.joint_state.name.push_back("l_shoulder_lift_joint");
  req.start_state.joint_state.name.push_back("l_upper_arm_roll_joint");
  req.start_state.joint_state.name.push_back("l_elbow_flex_joint");
  req.start_state.joint_state.name.push_back("l_forearm_roll_joint");
  req.start_state.joint_state.name.push_back("l_wrist_flex_joint");
  req.start_state.joint_state.name.push_back("l_wrist_roll_joint");
  req.start_state.joint_state.position.push_back(start_state.body.z);
  req.start_state.joint_state.name.push_back("torso_lift_joint");


  ROS_INFO("[exp] Start state %d angles %d joint names", int(req.start_state.joint_state.position.size()), int(req.start_state.joint_state.name.size()));
  // goal pose
  req.goal_constraints.resize(1);
  req.goal_constraints[0].position_constraints.resize(1);
  req.goal_constraints[0].position_constraints[0].constraint_region_pose.header.stamp = ros::Time::now();
  req.goal_constraints[0].position_constraints[0].constraint_region_pose.header.frame_id = "odom";

  req.goal_constraints[0].position_constraints[0].link_name = "two_arms_object";
  req.goal_constraints[0].position_constraints[0].constraint_region_pose.pose.position.x = loc_map_[exp_map_[name].goal].at(0);
  req.goal_constraints[0].position_constraints[0].constraint_region_pose.pose.position.y = loc_map_[exp_map_[name].goal].at(1);
  req.goal_constraints[0].position_constraints[0].constraint_region_pose.pose.position.z = loc_map_[exp_map_[name].goal].at(2);

  req.goal_constraints[0].position_constraints[0].constraint_region_shape.type = moveit_msgs::Shape::BOX;
  req.goal_constraints[0].position_constraints[0].constraint_region_shape.dimensions.push_back(goal_tolerance_[0]);
  req.goal_constraints[0].position_constraints[0].constraint_region_shape.dimensions.push_back(goal_tolerance_[1]);
  req.goal_constraints[0].position_constraints[0].constraint_region_shape.dimensions.push_back(goal_tolerance_[2]);
  req.goal_constraints[0].position_constraints[0].weight = 1.0;

  req.goal_constraints[0].orientation_constraints.resize(1);
  req.goal_constraints[0].orientation_constraints[0].orientation.header.stamp = ros::Time::now();
  req.goal_constraints[0].orientation_constraints[0].orientation.header.frame_id = "odom";
  req.goal_constraints[0].orientation_constraints[0].link_name = "two_arms_object";
  q.setRPY(loc_map_[exp_map_[name].goal].at(3), loc_map_[exp_map_[name].goal].at(4), loc_map_[exp_map_[name].goal].at(5));
  tf::quaternionTFToMsg(q, req.goal_constraints[0].orientation_constraints[0].orientation.quaternion);
  req.goal_constraints[0].orientation_constraints[0].absolute_x_axis_tolerance = goal_tolerance_[3];
  req.goal_constraints[0].orientation_constraints[0].absolute_y_axis_tolerance = goal_tolerance_[4];
  req.goal_constraints[0].orientation_constraints[0].absolute_z_axis_tolerance = goal_tolerance_[5];
  req.goal_constraints[0].orientation_constraints[0].weight = 1.0;

  // path constraint
  req.path_constraints.orientation_constraints.resize(1);
  req.path_constraints.orientation_constraints[0].orientation.header.stamp = ros::Time::now();
  req.path_constraints.orientation_constraints[0].orientation.header.frame_id = "odom";
  req.path_constraints.orientation_constraints[0].link_name = "two_arms_object";
  req.path_constraints.orientation_constraints[0].orientation.quaternion.x = 0;
  req.path_constraints.orientation_constraints[0].orientation.quaternion.y = 0;
  req.path_constraints.orientation_constraints[0].orientation.quaternion.z = 0;
  req.path_constraints.orientation_constraints[0].orientation.quaternion.w = 1;
  req.path_constraints.orientation_constraints[0].absolute_x_axis_tolerance = 0.04;
  req.path_constraints.orientation_constraints[0].absolute_y_axis_tolerance = 0.04;
  req.path_constraints.orientation_constraints[0].absolute_z_axis_tolerance = 3.14;

  req.start_state.multi_dof_joint_state.poses.resize(2);
  req.start_state.multi_dof_joint_state.frame_ids.resize(2);
  req.start_state.multi_dof_joint_state.child_frame_ids.resize(2);
  req.start_state.multi_dof_joint_state.frame_ids[0] = "two_arms_object";
  req.start_state.multi_dof_joint_state.frame_ids[1] = "two_arms_object";
  req.start_state.multi_dof_joint_state.child_frame_ids[0] = "r_wrist_roll_link";
  req.start_state.multi_dof_joint_state.child_frame_ids[1] = "l_wrist_roll_link";
  req.start_state.multi_dof_joint_state.poses[0] = rarm_object_pose_;
  req.start_state.multi_dof_joint_state.poses[1] = larm_object_pose_;
}

bool BenchmarkManipulationTests::setRobotPoseFromTrajectory(moveit_msgs::RobotTrajectory &trajectory, moveit_msgs::RobotState &trajectory_start, RobotPose &pose)
{
  // update the current pose with the previous start state of the robot
  getRobotPoseFromRobotState(trajectory_start, pose);

  if(trajectory.joint_trajectory.points.empty())
  {
    ROS_INFO("[exp] Trajectory is empty....unable to set robot pose.");
    return true;
  }

  if(experiment_type_ == 1)
  {
    if(!getJointPositionsFromTrajectory(trajectory.joint_trajectory, rjoint_names_, int(trajectory.joint_trajectory.points.size())-1, pose.rangles))
    {
      ROS_ERROR("[exp] Failed to get the joint positions for the right arm from waypoint %d", int(trajectory.joint_trajectory.points.size())-1);
      return false;
    }
  }
  else
  {
    if(!getJointPositionsFromTrajectory(trajectory.joint_trajectory, rjoint_names_, trajectory.joint_trajectory.points.size()-1, pose.rangles))
      return false;
    if(!getJointPositionsFromTrajectory(trajectory.joint_trajectory, ljoint_names_, trajectory.joint_trajectory.points.size()-1, pose.langles))
      return false;
  }
  return true;
}

bool BenchmarkManipulationTests::getJointPositionsFromTrajectory(const trajectory_msgs::JointTrajectory &traj, std::vector<std::string> &names, int waypoint, std::vector<double> &angles)
{
  unsigned int ind = 0;
  angles.resize(names.size());

  for(size_t i = 0; i < traj.joint_names.size(); i++)
  {
    if(names[ind].compare(traj.joint_names[i]) == 0)
    {
      angles[ind] = traj.points[waypoint].positions[i];
      ind++;
      ROS_DEBUG("[exp] Found %s in trajectory, ind = %d", traj.joint_names[i].c_str(), ind);
    }
    if(ind == names.size())
      break;
  }
  if(ind != names.size())
  {
    ROS_WARN("[exp] Not all of the expected joints were found in the trajectory.");
    return false;
  }
  return true;
}

bool BenchmarkManipulationTests::getRobotPoseFromRobotState(const moveit_msgs::RobotState &state, RobotPose &pose)
{
  unsigned int lind = 0, rind = 0;
  pose.langles.resize(ljoint_names_.size());
  pose.rangles.resize(rjoint_names_.size());

  // arms
  for(size_t i = 0; i < state.joint_state.name.size(); i++)
  {
    if(rind < rjoint_names_.size())
    {
      if(rjoint_names_[rind].compare(state.joint_state.name[i]) == 0)
      {
        ROS_DEBUG("[exp] [right-start] %-20s: %0.3f", rjoint_names_[rind].c_str(), state.joint_state.position[i]);
        pose.rangles[rind] = state.joint_state.position[i];
        rind++;
      }
    }
    if(lind < ljoint_names_.size())
    {
      if(ljoint_names_[lind].compare(state.joint_state.name[i]) == 0)
      {
        ROS_DEBUG("[exp] [left-start] %-20s: %0.3f", ljoint_names_[lind].c_str(), state.joint_state.position[i]);
        pose.langles[lind] = state.joint_state.position[i];
        lind++;
      }
    }
    if(rind == rjoint_names_.size() && lind == ljoint_names_.size())
      break;
  }
  if(rind != rjoint_names_.size() || lind != ljoint_names_.size())
  {
    ROS_DEBUG("[exp] Not all of the expected joints were found in the RobotState.");
    return false;
  }

  // torso
  for(size_t i = 0; i < state.joint_state.name.size(); i++)
  {
    if(state.joint_state.name[i].compare(spine_frame_) == 0)
    {
      pose.body.z = state.joint_state.position[i];
      break;
    }
  }

  return true;
}

bool BenchmarkManipulationTests::getBasePoseFromPlanningScene(const moveit_msgs::PlanningScene &scene, BodyPose &body)
{
  // base - if there is a map frame
  if(!scene.fixed_frame_transforms.empty())
  {
    for(size_t i = 0; i < scene.fixed_frame_transforms.size(); ++i)
    {
      if(scene.fixed_frame_transforms[i].header.frame_id.compare(world_frame_) == 0 && 
          scene.fixed_frame_transforms[i].child_frame_id.compare(robot_model_root_frame_) == 0)
      {
        body.x = scene.fixed_frame_transforms[i].transform.translation.x;
        body.y = scene.fixed_frame_transforms[i].transform.translation.y;
        body.theta =  2 * atan2(scene.fixed_frame_transforms[i].transform.rotation.z, scene.fixed_frame_transforms[i].transform.rotation.w);
        ROS_WARN("WARNING: The formula to compute the yaw of the base pose is untested.");
        break;
      }
    }
  }
  return true;
}

void BenchmarkManipulationTests::printPoseMsg(const geometry_msgs::Pose &p, std::string text)
{
  ROS_INFO("[%s] xyz: %0.3f %0.3f %0.3f  quat: %0.3f %0.3f %0.3f %0.3f", text.c_str(), p.position.x, p.position.y, p.position.z, p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
}

void BenchmarkManipulationTests::publishDisplayTrajectoryMsg(moveit_msgs::RobotTrajectory &traj, moveit_msgs::RobotState &state, std::string id)
{
  moveit_msgs::DisplayTrajectory disp;

  disp.model_id = "pr2";
  disp.trajectory = traj;
  disp.trajectory_start = state;

  //display_path_pub_.publish(disp);
}

bool BenchmarkManipulationTests::printPathToFile(FILE** file, const trajectory_msgs::JointTrajectory &traj)
{
  if(*file == NULL)
  {
    ROS_ERROR("[exp] File pointer is null.");
    return false;
  }

  double roll,pitch,yaw;
  tf::Pose tf_pose;
  geometry_msgs::Pose pose;
  std::vector<double> jnt_pos(7,0);

  for(size_t i = 0; i < traj.points.size(); ++i)
  {
    for(size_t j = 0; j < 7; ++j)
      jnt_pos[j] = traj.points[i].positions[j];

    if(!computeFK(jnt_pos, pose))
    {
      ROS_ERROR("[node] IK failed when printing path to file.");
      return false;
    }
    tf::poseMsgToTF(pose, tf_pose);
    tf_pose.getBasis().getRPY(roll,pitch,yaw);

    if(experiment_type_ == 1)
      fprintf(*file, "%1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %2.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f\n", traj.points[i].positions[0],traj.points[i].positions[1],traj.points[i].positions[2],traj.points[i].positions[3],traj.points[i].positions[4],traj.points[i].positions[5],traj.points[i].positions[6],pose.position.x, pose.position.y, pose.position.z, roll, pitch, yaw, pose.orientation.x,pose.orientation.y, pose.orientation.z, pose.orientation.w);
    else
      fprintf(*file, "%1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %2.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f\n", traj.points[i].positions[0],traj.points[i].positions[1],traj.points[i].positions[2],traj.points[i].positions[3],traj.points[i].positions[4],traj.points[i].positions[5],traj.points[i].positions[6], traj.points[i].positions[7],traj.points[i].positions[8],traj.points[i].positions[9],traj.points[i].positions[10],traj.points[i].positions[11],traj.points[i].positions[12],traj.points[i].positions[13], pose.position.x, pose.position.y, pose.position.z, roll, pitch, yaw, pose.orientation.x,pose.orientation.y, pose.orientation.z, pose.orientation.w);

  }
  fflush(*file);

  return true;
}

bool BenchmarkManipulationTests::initKinematicSolver(const planning_models::KinematicModelConstPtr &kmodel)
{
  kinematics_plugin_loader::KinematicsPluginLoader kinematics_loader;
  kinematics_plugin_loader::KinematicsLoaderFn kinematics_allocator = kinematics_loader.getLoaderFunction();
  
  kmodel_ = kmodel;
  tf_ = pscene_.getTransforms();

  if(!pscene_.isConfigured())
  {
    ROS_ERROR("[exp] Planning scene is not yet configured.");
    return false;
  }

  if(!kinematics_allocator)
  {
    ROS_ERROR("[exp] Failed to get the kinematics loader function.");
    return false;
  }
  ROS_INFO("[exp] Successfully got the kinematics loader function.");

  if(!kmodel_->hasJointModelGroup("right_arm"))
  {
    ROS_ERROR("[exp] right_arm group is not found in the kinematic model (%s).", kmodel_->getName().c_str());
    return false;
  }
  ROS_DEBUG("[exp] right_arm group is found in the kinematic model (%s).", kmodel_->getName().c_str());

  jmg_ = kmodel_->getJointModelGroup("right_arm");
  for(size_t i = 0; i < jmg_->getJointModelNames().size(); ++i)
    ROS_INFO("[exp] [%d] %s", int(i), jmg_->getJointModelNames()[i].c_str());

  kb_ = kinematics_allocator(jmg_);

  if (!kb_)
  {
    ROS_ERROR("[exp] Failed to allocate IK solver for right_arm group.");
    return false;
  }
  else
    ROS_INFO("[exp] BenchmarkManipulationTests successfully loaded the kinematics solvers.");

  /*
  // the ik solver must cover the same joints as the group
  const std::vector<std::string> &kb_jnames = kb_->getJointNames();
  const std::map<std::string, unsigned int> &g_map = jmg_->getJointVariablesIndexMap();

  ROS_DEBUG("KB joint names:");
  for(size_t i = 0; i < kb_jnames.size(); ++i)
    ROS_DEBUG("[node] [%d] %s", int(i), kb_jnames[i].c_str());

  ROS_DEBUG("JMG joint names:");
  for(std::map<std::string, unsigned int>::const_iterator iter = g_map.begin(); iter != g_map.end(); ++iter)
    ROS_DEBUG("[node] %s -> %d", iter->first.c_str(), iter->second);

  if (kb_jnames.size() != g_map.size())
  {
    ROS_ERROR_STREAM("[node] Group '" << jmg_->getName() << "' does not have the same set of joints as the employed IK solver");
    return false;
  }

  // compute a mapping between the group state and the IK solution
  kb_joint_bijection_.clear();
  for (std::size_t i = 0 ; i < kb_jnames.size() ; ++i)
  {
    std::map<std::string, unsigned int>::const_iterator it = g_map.find(kb_jnames[i]);
    if (it == g_map.end())
    {
      ROS_ERROR_STREAM("[node] IK solver computes joint values for joint '" << kb_jnames[i] << "' but group '" << jmg_->getName() << "' does not contain such a joint.");
      return false;
    }
    const planning_models::KinematicModel::JointModel *jm = jmg_->getJointModel(kb_jnames[i]);
    for (unsigned int k = 0 ; k < jm->getVariableCount() ; ++k)
      kb_joint_bijection_.push_back(it->second + k);
  }

  ROS_DEBUG("KB <-> JMG Mapping:");
  for(size_t i = 0; i < kb_joint_bijection_.size(); ++i)
    ROS_DEBUG("[node] [KB %d] -> [JMG %d]", int(i), int(kb_joint_bijection_[i]));
 */
 
  fk_link_.resize(1);
  fk_link_[0] = kb_->getTipFrame();
  return true;
}

bool BenchmarkManipulationTests::computeFK(std::vector<double> &angles, geometry_msgs::Pose &pose)
{
  Eigen::Affine3d t;
  std::vector<geometry_msgs::Pose> poses;
  if(!kb_->getPositionFK(fk_link_, angles, poses))
    return false;

  // transform into planning frame
  planning_models::poseFromMsg(poses[0],t);
  t = tf_->getTransform(pscene_.getCurrentState(), kb_->getBaseFrame()) * t;
  planning_models::msgFromPose(t,pose);
  return true;
}

bool BenchmarkManipulationTests::createTrajectoryFile(std::string exp_name, std::string planner, std::string planner_id, std::string description, FILE** file)
{
  std::string filename;

  size_t slash = planner.find("/");
  planner = planner.substr(slash+1);
  if(planner_id.empty())
    filename  = exp_name + "-" + planner + "_" + description + ".csv";
  else
    filename  = exp_name + "-" + planner + "_" + planner_id + "_" + description + ".csv";

  filename = trajectory_files_path_ + "/" +  filename;
  if((*file = fopen(filename.c_str(), "w")) == NULL)
  {
    ROS_ERROR("[node] Failed to create trajectory file. (%s)", filename.c_str());
    return false;
  }
  ROS_INFO("[exp] Successfully created trajectory file: %s", filename.c_str());
  return true;
}

bool BenchmarkManipulationTests::createExperimentGroupFolder(std::string exp_group_name)
{
  std::string folder = benchmark_results_folder_ + exp_group_name;
  if(!createFolder(folder))
    return false;
  
  folder = folder + "/" + "trajectories/";
  if(!createFolder(folder))
    return false;
   
  return true;
}

bool BenchmarkManipulationTests::createTrajectoryFolder(std::string exp_name)
{
  struct stat st;
  time_t clock;
  time(&clock);
  std::string time(ctime(&clock));;
  time.erase(time.size()-1, 1);

  std::string folder = benchmark_results_folder_ + experiment_group_name_ + "/trajectories/" + exp_name; // + "_" + time;
  if(mkdir(folder.c_str(),  S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == 0)
    ROS_INFO("[exp] Successfully created the trajectory folder: %s", folder.c_str());
  else
  {
    if(stat(folder.c_str(), &st) == 0)
      ROS_INFO("[exp] folder is present. Not creating.");
    else
    {
      ROS_WARN("[exp] Failed to create the trajectory folder: %s. Maybe it exists already?", folder.c_str());
      return false;
    }
  }
  trajectory_files_path_ = folder;
  return true;
}

bool BenchmarkManipulationTests::createFolder(std::string name)
{
  struct stat st;
  if(mkdir(name.c_str(),  S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == 0)
    ROS_INFO("[exp] Successfully created the trajectory folder: %s", name.c_str());
  else
  {
    if(stat(name.c_str(), &st) == 0)
      ROS_INFO("[exp] folder is present. Not creating.");
    else
    {
      ROS_WARN("[exp] Failed to create the trajectory folder: %s. Maybe it exists already?", name.c_str());
      return false;
    }
  }
  return true;
}

bool BenchmarkManipulationTests::writeTrajectoriesToFile(const moveit_msgs::ComputePlanningBenchmark::Response &res, std::string exp_name)
{
  FILE* fptr = NULL;

  if(res.planner_interfaces.size() != res.responses.size())
    return false;
  if(res.planner_interfaces.empty())
    return false;

  if(!createTrajectoryFolder(exp_name))
  {
    ROS_ERROR("[exp] Failed to create a trajectory folder for %s", exp_name.c_str());
    return false;
  }

  for(size_t i = 0; i < res.planner_interfaces.size(); ++i)
  {
    for(size_t j = 0; j < res.responses[i].trajectory.size(); ++j)
    {
      if(!createTrajectoryFile(exp_name, res.planner_interfaces[i], "", res.responses[i].description[j], &fptr))
      {
        ROS_ERROR("[exp] Failed to create a trajectory file for %s + %s", res.planner_interfaces[i].c_str(), res.responses[i].description[j].c_str());
        return false;
      }
  
      if(!printPathToFile(&fptr, res.responses[i].trajectory[j].joint_trajectory))
      {
        ROS_ERROR("[exp] Failed to print the trajectory to file.");
        return false;
      }
      fclose(fptr);
    }
  }
  return true;
}

void BenchmarkManipulationTests::multiplyPoses(geometry_msgs::Pose &p1, geometry_msgs::Pose &p2, geometry_msgs::Pose &p)
{
  Eigen::Affine3d p_e, p1_e, p2_e;
  planning_models::poseFromMsg(p1, p1_e);
  planning_models::poseFromMsg(p2, p2_e);
  p_e = p1_e * p2_e;
  planning_models::msgFromPose(p_e, p);
}

bool BenchmarkManipulationTests::getTrajectoryDisplayMsg(const moveit_msgs::ComputePlanningBenchmark::Response &res, std::string planner_interface, std::string description, moveit_msgs::DisplayTrajectory &disp)
{
  for(size_t i = 0; i < res.planner_interfaces.size(); ++i)
  {
    if(res.planner_interfaces[i].compare(planner_interface) == 0)
    {
      for(size_t j = 0; j < res.responses[i].description.size(); ++j)
      {
        if(res.responses[i].description[j].compare(description) == 0)
        {
          if(!res.responses[i].trajectory[j].joint_trajectory.points.empty())
          {
            disp.model_id = "pr2";
            disp.trajectory = res.responses[i].trajectory[j];
            disp.trajectory_start = res.responses[i].trajectory_start;
            return true;
          }
          else
            ROS_ERROR("[exp] Not displaying %s path with description, %s because it's empty.", planner_interface.c_str(), description.c_str());
        }
      }
    }
  }
  ROS_WARN("[exp] Got to the end of getTrajectoryDisplayMsg(). The desired path must not have been found. Display failed.");
  return false;
}

void BenchmarkManipulationTests::visualizeTrajectories(const moveit_msgs::ComputePlanningBenchmark::Response &res)
{
  moveit_msgs::DisplayTrajectory disp;

  // display OMPL path
  if(getTrajectoryDisplayMsg(res, "ompl_interface_ros/OMPLPlanner", display_trajectory_description_, disp))
    ompl_display_path_pub_.publish(disp);
  usleep(1000);
  // display SBPL path
  if(getTrajectoryDisplayMsg(res, "sbpl_arm_planner_interface_ros/SBPLPlanner", display_trajectory_description_, disp))
    sbpl_display_path_pub_.publish(disp);
  usleep(1000);

  ROS_INFO("[exp] Finished publishing trajectories.");
}

