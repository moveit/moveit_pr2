/* \author Ben Cohen */

#include <pviz/pviz.h>
#define Z_TORSO_LIFT  -0.802
#define X_TORSO_LIFT 0.05

static std::string RIGHT_FK_SERVICE_NAME = "pr2_right_arm_kinematics/get_fk";
static std::string LEFT_FK_SERVICE_NAME = "pr2_left_arm_kinematics/get_fk";
static std::string RIGHT_IK_SERVICE_NAME = "pr2_right_arm_kinematics/get_ik";
static std::string LEFT_IK_SERVICE_NAME = "pr2_left_arm_kinematics/get_ik";

static std::string RIGHT_CHAIN_RTIP_NAME = "r_gripper_r_finger_tip_link";
static std::string RIGHT_CHAIN_LTIP_NAME = "r_gripper_l_finger_tip_link";
static std::string LEFT_CHAIN_RTIP_NAME = "l_gripper_r_finger_tip_link";
static std::string LEFT_CHAIN_LTIP_NAME = "l_gripper_l_finger_tip_link";


void HSVtoRGB( double *r, double *g, double *b, double h, double s, double v )
{
	int i;
	double f, p, q, t;
	if( s == 0 ) {
		// achromatic (grey)
		*r = *g = *b = v;
		return;
	}
	h /= 60;        // sector 0 to 5
	i = floor(h);
	f = h - i;			// factorial part of h
	p = v * ( 1 - s );
	q = v * ( 1 - s * f );
	t = v * ( 1 - s * ( 1 - f ) );
	switch( i ) {
		case 0:
			*r = v;
			*g = t;
			*b = p;
			break;
		case 1:
			*r = q;
			*g = v;
			*b = p;
			break;
		case 2:
			*r = p;
			*g = v;
			*b = t;
			break;
		case 3:
			*r = p;
			*g = q;
			*b = v;
			break;
		case 4:
			*r = t;
			*g = p;
			*b = v;
			break;
    default:
			*r = v;
			*g = p;
			*b = q;
			break;
	}
}

PViz::PViz() : ph_("~")
{
  num_joints_ = 8; //arm + torso
  reference_frame_ = "map"; // maybe should be base_footprint?

  srand (time(NULL));

  fk_service_name_.resize(2);
  fk_service_name_[RIGHT] = RIGHT_FK_SERVICE_NAME;
  fk_service_name_[LEFT] = LEFT_FK_SERVICE_NAME;

  arm_joint_names_.push_back("_shoulder_pan_joint");
  arm_joint_names_.push_back("_shoulder_lift_joint");
  arm_joint_names_.push_back("_upper_arm_roll_joint");
  arm_joint_names_.push_back("_elbow_flex_joint");
  arm_joint_names_.push_back("_forearm_roll_joint");
  arm_joint_names_.push_back("_wrist_flex_joint");
  arm_joint_names_.push_back("_wrist_roll_joint");

  arm_link_names_.push_back("_shoulder_pan_link");
  arm_link_names_.push_back("_shoulder_lift_link");
  arm_link_names_.push_back("_upper_arm_roll_link");
  arm_link_names_.push_back("_elbow_flex_link");
  arm_link_names_.push_back("_forearm_roll_link");
  arm_link_names_.push_back("_wrist_flex_link");
  arm_link_names_.push_back("_wrist_roll_link");

  torso_link_names_.push_back("torso_lift_link");
  torso_joint_names_.push_back("torso_lift_joint");

  // 0: right, 1: left
  side_.push_back("r");
  side_.push_back("l");
  side_full_.push_back("right");
  side_full_.push_back("left");

  // arm meshes
  arm_meshes_.push_back("package://pr2_description/meshes/shoulder_v0/shoulder_yaw.stl");
  arm_meshes_.push_back("package://pr2_description/meshes/shoulder_v0/shoulder_lift.stl");
  arm_meshes_.push_back("package://pr2_description/meshes/shoulder_v0/upper_arm_roll.stl");
  arm_meshes_.push_back("package://pr2_description/meshes/upper_arm_v0/upper_arm.stl");
  arm_meshes_.push_back("package://pr2_description/meshes/upper_arm_v0/elbow_flex.stl");
  arm_meshes_.push_back("package://pr2_description/meshes/upper_arm_v0/forearm_roll.stl");  
  arm_meshes_.push_back("package://pr2_description/meshes/forearm_v0/forearm.stl");
  arm_meshes_.push_back("package://pr2_description/meshes/forearm_v0/wrist_flex.stl");
  arm_meshes_.push_back("package://pr2_description/meshes/forearm_v0/wrist_roll.stl");
  arm_meshes_.push_back("package://pr2_description/meshes/gripper_v0/gripper_palm.stl");

  // gripper meshes
  gripper_meshes_.push_back("package://pr2_description/meshes/gripper_v0/upper_finger_r.stl");
  gripper_meshes_.push_back("package://pr2_description/meshes/gripper_v0/finger_tip_r.stl");
  gripper_meshes_.push_back("package://pr2_description/meshes/gripper_v0/upper_finger_l.stl");
  gripper_meshes_.push_back("package://pr2_description/meshes/gripper_v0/finger_tip_l.stl");

  // torso_meshes
  torso_meshes_.push_back("package://pr2_description/meshes/torso_v0/torso_lift.stl");
  
  // base meshes
  base_meshes_.push_back("package://pr2_description/meshes/base_v0/base.stl");

  robot_meshes_.insert(robot_meshes_.end(),base_meshes_.begin(), base_meshes_.end());  // 1
  robot_meshes_.insert(robot_meshes_.end(),torso_meshes_.begin(), torso_meshes_.end()); // 1
  robot_meshes_.insert(robot_meshes_.end(),arm_meshes_.begin(), arm_meshes_.end());  // 10
  robot_meshes_.insert(robot_meshes_.end(),gripper_meshes_.begin(), gripper_meshes_.end());  // 4
  robot_meshes_.insert(robot_meshes_.end(),arm_meshes_.begin(), arm_meshes_.end());  // 10
  robot_meshes_.insert(robot_meshes_.end(),gripper_meshes_.begin(), gripper_meshes_.end());  // 4

  initKDLChain();

  marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 500);
  marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
}

PViz::~PViz()
{
  for(size_t i = 0; i < fk_rsolver_.size(); ++i)
    delete fk_rsolver_[i];
  for(size_t i = 0; i < fk_lsolver_.size(); ++i)
    delete fk_lsolver_[i];
}

bool PViz::initKDLChain()
{
  std::string robot_description;
  std::string robot_param;
  nh_.searchParam("robot_description",robot_param);
  nh_.param<std::string>(robot_param,robot_description,"robot_description");

  if (!kdl_parser::treeFromString(robot_description, kdl_tree_))
  {
    ROS_ERROR("Failed to parse tree from robot description file.");
    return false;
  }

  // a total of 4 FK solvers are used for both arms & torso
  fk_rsolver_.resize(2);
  fk_lsolver_.resize(2);

  // right arm - right finger (from base_footprint)
  if (!kdl_tree_.getChain("base_footprint", RIGHT_CHAIN_RTIP_NAME, chain_))
  {
    ROS_ERROR("Error: could not fetch the KDL chain for the desired manipulator. Exiting."); 
    return false;
  }
  fk_rsolver_[RIGHT] = new KDL::ChainFkSolverPos_recursive(chain_);
  ROS_DEBUG("[pviz] the right arm-right finger chain has %d segments with %d joints", chain_.getNrOfSegments(), chain_.getNrOfJoints());
  //printKDLChain("right arm - right finger", chain_);

  // right arm - left finger (from base_footprint)
  if (!kdl_tree_.getChain("base_footprint", RIGHT_CHAIN_LTIP_NAME, chain_))
  {
    ROS_ERROR("Error: could not fetch the KDL chain for the desired manipulator. Exiting."); 
    return false;
  }
  fk_lsolver_[RIGHT] = new KDL::ChainFkSolverPos_recursive(chain_);
  ROS_DEBUG("[pviz] the right arm-left finger chain has %d segments with %d joints", chain_.getNrOfSegments(), chain_.getNrOfJoints());
  //printKDLChain("right arm - left finger", chain_);

  // left arm - right finger (from base_footprint)
  if (!kdl_tree_.getChain("base_footprint", LEFT_CHAIN_RTIP_NAME, chain_))
  {
    ROS_ERROR("Error: could not fetch the KDL chain for the desired manipulator. Exiting."); 
    return false;
  }
  fk_rsolver_[LEFT] = new KDL::ChainFkSolverPos_recursive(chain_);
  ROS_DEBUG("[pviz] the left arm-right finger chain has %d segments with %d joints", chain_.getNrOfSegments(), chain_.getNrOfJoints());
  //printKDLChain("left arm - right finger", chain_);

  // left arm - left finger (from base_footprint)
  if (!kdl_tree_.getChain("base_footprint", LEFT_CHAIN_LTIP_NAME, chain_))
  {
    ROS_ERROR("Error: could not fetch the KDL chain for the desired manipulator. Exiting."); 
    return false;
  }
  fk_lsolver_[LEFT] = new KDL::ChainFkSolverPos_recursive(chain_);
  ROS_DEBUG("[pviz] the left arm-right finger chain has %d segments with %d joints", chain_.getNrOfSegments(), chain_.getNrOfJoints());
  //printKDLChain("left arm - left finger", chain_);

  jnt_pos_in_.resize(chain_.getNrOfJoints());
  jnt_pos_out_.resize(chain_.getNrOfJoints());

  num_joints_ = chain_.getNrOfJoints();

  ROS_DEBUG("[pviz] jnt_pos_in: rows: %d cols: %d", jnt_pos_in_.rows(), jnt_pos_out_.columns());

  return true;
}

bool PViz::computeFKwithKDL(const std::vector<double> &angles, std::vector<double> &base_pos, double torso_pos, int arm, int frame_num, geometry_msgs::Pose &pose)
{
  KDL::Frame frame_out;
  jnt_pos_in_(0) = torso_pos;
  for(size_t i = 0; i < angles.size(); ++i)
    jnt_pos_in_(i+1) = angles[i];
  jnt_pos_in_(8) = 0;
  jnt_pos_in_(9) = 0;

  //ROS_WARN("jnt_pos_in:"); 
  //for(int i = 0; i < jnt_pos_in_.rows(); ++i)
  //  ROS_WARN("%d: %0.3f", i, jnt_pos_in_(i));

  // a negative frame number means that frame is in the left finger chain
  if(frame_num > 0)
  {
    if(fk_rsolver_[arm]->JntToCart(jnt_pos_in_, frame_out, frame_num) < 0)
    {
      ROS_ERROR("JntToCart returned < 0. Exiting.");
      return false;
    }
  }
  else
  {
    if(fk_lsolver_[arm]->JntToCart(jnt_pos_in_, frame_out, -1*frame_num) < 0)
    {
      ROS_ERROR("JntToCart returned < 0. Exiting.");
      return false;
    }
  }

  KDL::Frame map_to_robot;
  getMaptoRobotTransform(base_pos[0],base_pos[1],base_pos[2],map_to_robot);

  frame_out = map_to_robot*frame_out;
  
  pose.position.x = frame_out.p[0];
  pose.position.y = frame_out.p[1];
  pose.position.z = frame_out.p[2];

  frame_out.M.GetQuaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
  return true;
}

void PViz::visualizeObstacles(const std::vector<std::vector<double> > &obstacles)
{
  marker_array_.markers.clear();
  marker_array_.markers.resize(obstacles.size());

  ROS_INFO("[pviz] Displaying %d obstaclesin the %s frame", (int)obstacles.size(), reference_frame_.c_str());

  std::string ns = "obstacles"+boost::lexical_cast<std::string>(rand());

  for(int i = 0; i < int(obstacles.size()); i++)
  {
    if(obstacles[i].size() < 6)
    {
      ROS_INFO("[pviz] Obstacle description doesn't have length = 6");
      continue;
    }

    //TODO: Change this to use a CUBE_LIST
    marker_array_.markers[i].header.stamp = ros::Time::now();
    marker_array_.markers[i].header.frame_id = reference_frame_;
    marker_array_.markers[i].ns = ns;
    marker_array_.markers[i].id = rand();
    marker_array_.markers[i].type = visualization_msgs::Marker::CUBE;
    marker_array_.markers[i].action = visualization_msgs::Marker::ADD;
    marker_array_.markers[i].pose.position.x = obstacles[i][0];
    marker_array_.markers[i].pose.position.y = obstacles[i][1];
    marker_array_.markers[i].pose.position.z = obstacles[i][2];
    marker_array_.markers[i].scale.x = obstacles[i][3];
    marker_array_.markers[i].scale.y = obstacles[i][4];
    marker_array_.markers[i].scale.z = obstacles[i][5];
    marker_array_.markers[i].color.r = 0.0;
    marker_array_.markers[i].color.g = 0.0;
    marker_array_.markers[i].color.b = 0.5;
    marker_array_.markers[i].color.a = 0.9;
    marker_array_.markers[i].lifetime = ros::Duration(180.0);
  }

  marker_array_publisher_.publish(marker_array_);
}

void PViz::getCubeMsg(std::vector<double> &cube, std::vector<double> &color, std::string ns, int id, visualization_msgs::Marker& marker)
{
  if(cube.size() < 6)
  {
    ROS_INFO("[pviz] Three dimensions are needed to visualize a cube.");
    return;
  }
  if(color.size() < 4)
  {
    ROS_ERROR("[pviz] No color specified.");
    return;
  }

  for(size_t i = 0; i < color.size(); ++i)
  {
    if(color[i] > 1)
      color[i] = color[i] / 255.0;
  }

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "base_footprint";
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = cube[0];
  marker.pose.position.y= cube[1];
  marker.pose.position.z = cube[2];
  marker.pose.orientation.w = 1;
  marker.scale.x = cube[3];
  marker.scale.y = cube[4];
  marker.scale.z = cube[5];
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];
  marker.lifetime = ros::Duration(1000.0);
}

void PViz::getCubeMsg(geometry_msgs::Pose &pose, std::vector<double> &dim, std::vector<double> &color, std::string ns, int id, visualization_msgs::Marker& marker)
{
  if(dim.size() < 3)
  {
    ROS_INFO("[pviz] Three dimensions are needed to visualize a cube.");
    return;
  }
  if(color.size() < 4)
  {
    ROS_ERROR("[pviz] No color specified.");
    return;
  }

  for(size_t i = 0; i < color.size(); ++i)
  {
    if(color[i] > 1)
      color[i] = color[i] / 255.0;
  }

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "base_footprint";
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose;
  marker.scale.x = dim[0];
  marker.scale.y = dim[1];
  marker.scale.z = dim[2];
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];
  marker.lifetime = ros::Duration(1000.0);
}

void PViz::publishMarker(visualization_msgs::Marker& marker)
{
  marker_publisher_.publish(marker);
}

void PViz::publishMarkerArray(visualization_msgs::MarkerArray &marker_array)
{
  marker_array_publisher_.publish(marker_array);
  usleep(1000);
}

/*
void PViz::visualizeArmConfigurations(const std::vector<std::vector<double> > &traj, int arm, int throttle)
{
  double color_inc = 260/traj.size();   //260 is blue
  std::vector<double> zero_config(num_joints_,0);

  ROS_INFO("visualizeArmConfigurations is not yet fully functional - torso is set to 0");
  //always print out first & last configs
  visualizeArmConfiguration(color_inc*(1), arm, traj[0], 0.0);
  visualizeArmConfiguration(color_inc*((traj.size()-1)+1), arm, traj[traj.size()-1], 0.0);
  
  for(int i = 1; i < (int)traj.size(); i++)
  {
    //hack: some trajectories get parsed with an additional vector of zeroes 
    if(traj[i] == zero_config)
    {
      ROS_WARN("[visualizeArmConfigurations] Not displaying arm configurations of all zeroes.");
      continue;
    }

    if(i % throttle != 0)
      continue;

    visualizeArmConfiguration(color_inc*(i+1), arm, traj[i], 0.0);
  }
}
*/

void PViz::visualizePoses(const std::vector<std::vector<double> > &poses)
{
  marker_array_.markers.clear();
  marker_array_.markers.resize(poses.size()*3);
  tf::Quaternion pose_quaternion;
  geometry_msgs::Quaternion quaternion_msg;

  int mind = -1;

  ros::Time time = ros::Time::now();

  for(int i = 0; i < (int)poses.size(); ++i)
  {
    pose_quaternion.setRPY(poses[i][3],poses[i][4],poses[i][5]);
    tf::quaternionTFToMsg(pose_quaternion, quaternion_msg);

    mind++;
    marker_array_.markers[mind].header.stamp = time;
    marker_array_.markers[mind].header.frame_id = reference_frame_;
    marker_array_.markers[mind].ns = "pose_arrows";
    marker_array_.markers[mind].type = visualization_msgs::Marker::ARROW;
    marker_array_.markers[mind].id = i;
    marker_array_.markers[mind].action = visualization_msgs::Marker::ADD;
    marker_array_.markers[mind].pose.position.x = poses[i][0];
    marker_array_.markers[mind].pose.position.y = poses[i][1];
    marker_array_.markers[mind].pose.position.z = poses[i][2];
    marker_array_.markers[mind].pose.orientation = quaternion_msg;
    marker_array_.markers[mind].scale.x = 0.1;
    marker_array_.markers[mind].scale.y = 0.1;
    marker_array_.markers[mind].scale.z = 0.1;
    marker_array_.markers[mind].color.r = 0.0;
    marker_array_.markers[mind].color.g = 0.7;
    marker_array_.markers[mind].color.b = 0.6;
    marker_array_.markers[mind].color.a = 0.7;
    marker_array_.markers[mind].lifetime = ros::Duration(600.0);

    mind++;
    marker_array_.markers[mind].header.stamp = time;
    marker_array_.markers[mind].header.frame_id = reference_frame_;
    marker_array_.markers[mind].ns = "pose_spheres";
    marker_array_.markers[mind].id = i;
    marker_array_.markers[mind].type = visualization_msgs::Marker::SPHERE;
    marker_array_.markers[mind].action = visualization_msgs::Marker::ADD;
    marker_array_.markers[mind].pose.position.x = poses[i][0];
    marker_array_.markers[mind].pose.position.y = poses[i][1];
    marker_array_.markers[mind].pose.position.z = poses[i][2];
    marker_array_.markers[mind].pose.orientation = quaternion_msg;
    marker_array_.markers[mind].scale.x = 0.07;
    marker_array_.markers[mind].scale.y = 0.07;
    marker_array_.markers[mind].scale.z = 0.07;
    marker_array_.markers[mind].color.r = 1.0;
    marker_array_.markers[mind].color.g = 0.0;
    marker_array_.markers[mind].color.b = 0.6;
    marker_array_.markers[mind].color.a = 0.6;
    marker_array_.markers[mind].lifetime = ros::Duration(600.0);

    mind++;
    marker_array_.markers[mind].header.stamp = time;
    marker_array_.markers[mind].header.frame_id = reference_frame_;
    marker_array_.markers[mind].ns = "pose_text_blocks";
    marker_array_.markers[mind].id = i;
    marker_array_.markers[mind].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_array_.markers[mind].action = visualization_msgs::Marker::ADD;
    marker_array_.markers[mind].pose.position.x = poses[i][0];
    marker_array_.markers[mind].pose.position.y = poses[i][1];
    marker_array_.markers[mind].pose.position.z = poses[i][2];
    marker_array_.markers[mind].scale.x = 0.03;
    marker_array_.markers[mind].scale.y = 0.03;
    marker_array_.markers[mind].scale.z = 0.03;
    marker_array_.markers[mind].color.r = 1.0;
    marker_array_.markers[mind].color.g = 1.0;
    marker_array_.markers[mind].color.b = 1.0;
    marker_array_.markers[mind].color.a = 0.9;
    marker_array_.markers[mind].text = boost::lexical_cast<std::string>(i+1);
    marker_array_.markers[mind].lifetime = ros::Duration(600.0);
  }

  ROS_INFO("%d markers in the array",(int)marker_array_.markers.size());
  marker_array_publisher_.publish(marker_array_);
}

void PViz::visualizePose(const std::vector<double> &pose, std::string text)
{
  tf::Quaternion pose_quaternion;
  geometry_msgs::Pose pose_msg;

  pose_msg.position.x = pose[0];
  pose_msg.position.y = pose[1];
  pose_msg.position.z = pose[2];

  pose_quaternion.setRPY(pose[3],pose[4],pose[5]);
  tf::quaternionTFToMsg(pose_quaternion, pose_msg.orientation);

  ROS_DEBUG("[visualizing: %s] position: %0.3f %0.3f %0.3f quaternion: %0.3f %0.3f %0.3f %0.3f (frame: %s)", text.c_str(), pose[0], pose[1], pose[2], pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w, reference_frame_.c_str());

  visualizePose(pose_msg, text);
}

void PViz::visualizePose(const geometry_msgs::Pose &pose, std::string text)
{
  int mind = -1;
  marker_array_.markers.clear();
  marker_array_.markers.resize(3);
  ros::Time time = ros::Time::now();

  ROS_DEBUG("[visualizing: %s] position: %0.3f %0.3f %0.3f quaternion: %0.3f %0.3f %0.3f %0.3f (frame: %s)", text.c_str(), pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, reference_frame_.c_str());
  
  mind++;
  marker_array_.markers[mind].header.stamp = time;
  marker_array_.markers[mind].header.frame_id = reference_frame_;
  marker_array_.markers[mind].ns = text + "_arrow";
  marker_array_.markers[mind].type = visualization_msgs::Marker::ARROW;
  marker_array_.markers[mind].id = 0;
  marker_array_.markers[mind].action = visualization_msgs::Marker::ADD;
  marker_array_.markers[mind].pose = pose;
  marker_array_.markers[mind].scale.x = 0.125;
  marker_array_.markers[mind].scale.y = 0.125;
  marker_array_.markers[mind].scale.z = 0.125;
  marker_array_.markers[mind].color.r = 0.0;
  marker_array_.markers[mind].color.g = 0.7;
  marker_array_.markers[mind].color.b = 0.6;
  marker_array_.markers[mind].color.a = 0.7;
  marker_array_.markers[mind].lifetime = ros::Duration(500.0);

  mind++;
  marker_array_.markers[mind].header.stamp = time;
  marker_array_.markers[mind].header.frame_id = reference_frame_;
  marker_array_.markers[mind].ns = text + "_sphere";
  marker_array_.markers[mind].id = 1;
  marker_array_.markers[mind].type = visualization_msgs::Marker::SPHERE;
  marker_array_.markers[mind].action = visualization_msgs::Marker::ADD;
  marker_array_.markers[mind].pose = pose;
  marker_array_.markers[mind].scale.x = 0.10;
  marker_array_.markers[mind].scale.y = 0.10;
  marker_array_.markers[mind].scale.z = 0.10;
  marker_array_.markers[mind].color.r = 1.0;
  marker_array_.markers[mind].color.g = 0.0;
  marker_array_.markers[mind].color.b = 0.6;
  marker_array_.markers[mind].color.a = 0.6;
  marker_array_.markers[mind].lifetime = ros::Duration(500.0);

  mind++;
  marker_array_.markers[mind].header.stamp = time;
  marker_array_.markers[mind].header.frame_id = reference_frame_;
  marker_array_.markers[mind].ns = text;
  marker_array_.markers[mind].id = 2;
  marker_array_.markers[mind].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker_array_.markers[mind].action = visualization_msgs::Marker::ADD;
  marker_array_.markers[mind].pose = pose;
  marker_array_.markers[mind].scale.x = 0.03;
  marker_array_.markers[mind].scale.y = 0.03;
  marker_array_.markers[mind].scale.z = 0.03;
  marker_array_.markers[mind].color.r = 1.0;
  marker_array_.markers[mind].color.g = 1.0;
  marker_array_.markers[mind].color.b = 1.0;
  marker_array_.markers[mind].color.a = 0.9;
  marker_array_.markers[mind].text = text;
  marker_array_.markers[mind].lifetime = ros::Duration(500.0);

  marker_array_publisher_.publish(marker_array_);
}

void PViz::visualizeSphere(std::vector<double> pose, int color, std::string text, double radius)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = text + "-sphere";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = pose[0];
  marker.pose.position.y = pose[1];
  marker.pose.position.z = pose[2];
  marker.scale.x = radius*2;
  marker.scale.y = radius*2;
  marker.scale.z = radius*2;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(500.0);

  marker_publisher_.publish(marker);
}

void PViz::visualizeSpheres(const std::vector<std::vector<double> > &pose, int color, std::string text, double radius)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = "spheres-" + text;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = radius*2.0;
  marker.scale.y = radius*2.0;
  marker.scale.z = radius*2.0;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.6;
  marker.lifetime = ros::Duration(500.0);
  marker.id = 1;

  marker.points.resize(pose.size());
  for(size_t i = 0; i < pose.size(); i++)
  {
    marker.points[i].x = pose[i][0];
    marker.points[i].y = pose[i][1];
    marker.points[i].z = pose[i][2];
  }

  marker_publisher_.publish(marker);
}

void PViz::visualizeSpheres(const std::vector<std::vector<double> > &pose, int color, std::string text, std::vector<double> &radius)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

  for(size_t i = 0; i < pose.size(); ++i)
  {
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = reference_frame_;
    marker.ns = text;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = radius[i]*2.0;
    marker.scale.y = radius[i]*2.0;
    marker.scale.z = radius[i]*2.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.6;
    marker.lifetime = ros::Duration(500.0);
    marker.id = i;

    marker.pose.position.x = pose[i][0];
    marker.pose.position.y = pose[i][1];
    marker.pose.position.z = pose[i][2];

    marker_publisher_.publish(marker);
    usleep(100);
  }
}

void PViz::visualizeSpheres(const std::vector<std::vector<double> > &pose, int color, std::string text)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

  for(size_t i = 0; i < pose.size(); ++i)
  {
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = reference_frame_;
    marker.ns = text;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = pose[i][3]*2.0;
    marker.scale.y = pose[i][3]*2.0;
    marker.scale.z = pose[i][3]*2.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.6;
    marker.lifetime = ros::Duration(500.0);
    marker.id = i;

    marker.pose.position.x = pose[i][0];
    marker.pose.position.y = pose[i][1];
    marker.pose.position.z = pose[i][2];

    marker_publisher_.publish(marker);
    usleep(100);
  }
}

void PViz::deleteVisualizations(std::string ns, int max_id)
{
  marker_array_.markers.clear();
  marker_array_.markers.resize(max_id);

  for(int j = 0; j < max_id; j++)
  {
    marker_array_.markers[j].header.stamp = ros::Time::now();
    marker_array_.markers[j].header.frame_id = reference_frame_;
    marker_array_.markers[j].ns = ns;
    marker_array_.markers[j].action = visualization_msgs::Marker::DELETE;
    marker_array_.markers[j].id = j;
  }
  marker_array_publisher_.publish(marker_array_);
}

void PViz::visualize3DPath(std::vector<std::vector<double> > &dpath)
{
  if(dpath.empty())
  {
    ROS_INFO("[visualizeShortestPath] The shortest path is empty.");
    return;
  }
  else
    ROS_INFO("[visualizeShortestPath] There are %i waypoints in the shortest path.",int(dpath.size()));

  visualization_msgs::Marker obs_marker;
  obs_marker.header.frame_id = reference_frame_;
  obs_marker.header.stamp = ros::Time();
  obs_marker.header.seq = 0;
  obs_marker.ns = "path";
  obs_marker.id = 0;
  obs_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  obs_marker.action = 0;
  obs_marker.scale.x = 3*0.02;
  obs_marker.scale.y = 3*0.02;
  obs_marker.scale.z = 3*0.02;
  obs_marker.color.r = 0.45;
  obs_marker.color.g = 0.3;
  obs_marker.color.b = 0.4;
  obs_marker.color.a = 0.8;
  obs_marker.lifetime = ros::Duration(500.0);

  obs_marker.points.resize(dpath.size());

  for (int k = 0; k < int(dpath.size()); k++)
  {
    if(int(dpath[k].size()) < 3)
      continue;

    obs_marker.points[k].x = dpath[k][0];
    obs_marker.points[k].y = dpath[k][1];
    obs_marker.points[k].z = dpath[k][2];
  }

  marker_publisher_.publish(obs_marker);
}

void PViz::visualizeBasicStates(const std::vector<std::vector<double> > &states, const std::vector<double> &color, std::string name, double size)
{
  unsigned int inc = 1;
  visualization_msgs::Marker marker;
  
  //check if the list is empty
  if(states.empty())
  {
    ROS_DEBUG("[visualizeBasicStates] There are no states in the %s states list.", name.c_str());
    return;
  }

  //if there are too many states, rviz will crash and burn when drawing
  if(states.size() > 50000)
    inc = 4;
  else if(states.size() > 10000)
    inc = 1;   //2
  else
    inc = 1;

  marker.points.resize(states.size()/inc + 1);

  marker.header.seq = 0;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;

  marker.ns = name;
  marker.id = 1;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];
  marker.lifetime = ros::Duration(500.0);

  unsigned int m_ind = 0;
  for(unsigned int i = 0; i < states.size(); i=i+inc)
  {
    if(states[i].size() >= 3)
    {
      marker.points[m_ind].x = states[i][0];
      marker.points[m_ind].y = states[i][1];
      marker.points[m_ind].z = states[i][2];
      ++m_ind;
    }
  }

  marker_publisher_.publish(marker);
  ROS_DEBUG("[visualizeBasicStates] published %d markers for %s states", int(marker.points.size()), name.c_str());
}

void PViz::visualizeDetailedStates(const std::vector<std::vector<double> > &states, const std::vector<std::vector<double> >&color, std::string name, double size)
{
  unsigned int inc = 1;
  std::vector<double> scaled_color(4,0);
  visualization_msgs::MarkerArray marker_array;

  //check if the list is empty
  if(states.empty())
  {
    ROS_INFO("[aviz] There are no states in the %s states list", name.c_str());
    return;
  } 
  else
    ROS_INFO("[aviz] There are %i states in the %s states list.",int(states.size()),name.c_str());
    
  if(color.size()<2)
  {
    ROS_INFO("[aviz] Not enough colors specified.");
    return;
  } 
  
  if(color[0].size() < 4 || color[1].size() < 4)
  {
    ROS_INFO("[aviz] RGBA must be specified for each color.");
    return;
  } 
  
  //if there are too many states, rviz will crash and burn when drawing
  /*
  if(states.size() > 50000)
    inc = 20;
  else if(states.size() > 5000)
    inc = 10;
  else if(states.size() > 500)
    inc = 1;   //changed  8/31/11
  else
    inc = 1;
  */

  unsigned int mind = 0;
  for(unsigned int i = 0; i < states.size(); i=i+inc)
  {
    marker_array.markers.resize(marker_array.markers.size()+1);
    marker_array.markers[mind].header.frame_id = reference_frame_;
    marker_array.markers[mind].header.stamp = ros::Time::now();
    marker_array.markers[mind].ns = "expanded_states";
    marker_array.markers[mind].id = mind;
    marker_array.markers[mind].type = visualization_msgs::Marker::CUBE;
    marker_array.markers[mind].action =  visualization_msgs::Marker::ADD;
    marker_array.markers[mind].scale.x = size;
    marker_array.markers[mind].scale.y = size;
    marker_array.markers[mind].scale.z = size;
    
    for(unsigned int j = 0; j < 4; ++j)
      scaled_color[j] = color[0][j] - ((color[0][j] - color[1][j]) * (double(i)/double(states.size()/inc)));
      
    marker_array.markers[mind].color.r = scaled_color[0];
    marker_array.markers[mind].color.g = scaled_color[1];
    marker_array.markers[mind].color.b = scaled_color[2];
    marker_array.markers[mind].color.a = 1;
    marker_array.markers[mind].lifetime = ros::Duration(500.0);
    
    marker_array.markers[mind].pose.position.x = states[i][0];
    marker_array.markers[mind].pose.position.y = states[i][1];
    marker_array.markers[mind].pose.position.z = states[i][2];
    
    ++mind;
  } 
  
  ROS_DEBUG("[aviz] published %d markers for %s states", (int)marker_array.markers.size(), name.c_str());
  marker_array_publisher_.publish(marker_array);
}

void PViz::visualizeLine(const std::vector<geometry_msgs::Point> points, std::string ns, int id, int hue, double thickness)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.points = points;
  marker.scale.x = thickness;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.5;
  marker.lifetime = ros::Duration(500.0);

  ROS_INFO("Visualizing a line with %d points", int(points.size()));
  marker_publisher_.publish(marker);
}

void PViz::visualizeText(geometry_msgs::Pose pose, std::string text, std::string ns, int id, int hue)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.pose = pose;
  
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  marker.text = text;
  marker.lifetime = ros::Duration(10.0);

  marker_publisher_.publish(marker);
}

void PViz::visualizeText(geometry_msgs::Pose pose, std::string text, std::string ns, int id, std::vector<double> color, double size)
{
  visualization_msgs::Marker marker;

  if(color.size() < 4)
    color.resize(4,1);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;
  marker.pose = pose;
  
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];
  marker.text = text;
  marker.lifetime = ros::Duration(360.0);

  marker_publisher_.publish(marker);
}

void PViz::visualizeCube(geometry_msgs::PoseStamped pose, int color, std::string ns, int id, std::vector<double> dim)
{
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  if(dim.size() < 3)
  {
    ROS_INFO("[aviz] Three dimensions are needed to visualize a cube.");
    if(dim.size() > 1)
      dim.resize(3,dim[0]);
    else
      return;
  }

  HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = pose.header.frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose.pose;
  marker.scale.x = dim[0];
  marker.scale.y = dim[1];
  marker.scale.z = dim[2];
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(500.0);

  marker_publisher_.publish(marker);
}

void PViz::printKDLChain(std::string name, KDL::Chain &chain)
{
  ROS_INFO("chain: %s", name.c_str());
  for(unsigned int j = 0; j < chain.getNrOfSegments(); ++j)
  {
    ROS_INFO("  frame %2d: segment: %0.3f %0.3f %0.3f  joint: %0.3f %0.3f %0.3f   joint_type: %s",j,
        chain.getSegment(j).pose(0).p.x(),
        chain.getSegment(j).pose(0).p.y(),
        chain.getSegment(j).pose(0).p.z(),
        chain.getSegment(j).getJoint().pose(0).p.x(),
        chain.getSegment(j).getJoint().pose(0).p.y(),
        chain.getSegment(j).getJoint().pose(0).p.z(),
        chain.getSegment(j).getJoint().getTypeName().c_str());
  }
  ROS_INFO(" ");
}

bool PViz::computeFKforVisualizationWithKDL(const std::vector<double> &jnt0_pos, std::vector<double> &jnt1_pos, std::vector<double> &base_pos, double torso_pos, std::vector<geometry_msgs::PoseStamped> &poses)
{
  //output is PoseStamped only so it has same interface as computeFKforVisualization
  //KDL::Frame map_to_robot; 
  geometry_msgs::Pose pose;

/*
[ INFO]:   frame  0: segment: 0.000 0.000 0.051  joint: 0.000 0.000 0.000   joint_type: None
[ INFO]:   frame  1: segment: -0.050 0.000 0.740  joint: -0.050 0.000 0.740   joint_type: TransAxis
[ INFO]:   frame  2: segment: 0.000 0.188 0.000  joint: 0.000 0.188 0.000   joint_type: RotAxis
[ INFO]:   frame  3: segment: 0.100 0.000 0.000  joint: 0.100 0.000 0.000   joint_type: RotAxis
[ INFO]:   frame  4: segment: 0.000 0.000 0.000  joint: 0.000 0.000 0.000   joint_type: RotAxis
[ INFO]:   frame  5: segment: 0.000 0.000 0.000  joint: 0.000 0.000 0.000   joint_type: None
[ INFO]:   frame  6: segment: 0.400 0.000 0.000  joint: 0.400 0.000 0.000   joint_type: RotAxis
[ INFO]:   frame  7: segment: 0.000 0.000 0.000  joint: 0.000 0.000 0.000   joint_type: RotAxis
[ INFO]:   frame  8: segment: 0.000 0.000 0.000  joint: 0.000 0.000 0.000   joint_type: None
[ INFO]:   frame  9: segment: 0.321 0.000 0.000  joint: 0.321 0.000 0.000   joint_type: RotAxis
[ INFO]:   frame 10: segment: 0.000 0.000 0.000  joint: 0.000 0.000 0.000   joint_type: RotAxis
[ INFO]:   frame 11: segment: 0.000 0.000 0.000  joint: 0.000 0.000 0.000   joint_type: None
[ INFO]:   frame 12: segment: 0.077 0.010 0.000  joint: 0.077 0.010 0.000   joint_type: RotAxis
[ INFO]:   frame 13: segment: 0.091 0.005 0.000  joint: 0.091 0.005 0.000   joint_type: RotAxis

  0 base_v0/base.stl
  1 torso_v0/torso_lift.stl
  2 shoulder_v0/shoulder_yaw.stl
  3 shoulder_v0/shoulder_lift.stl
  4 upper_arm_roll
  5 upper_arm_v0/upper_arm.stl
  6 upper_arm_v0/elbow_flex.stl
  7 upper_arm_v0/forearm_roll.stl
  8 forearm_v0/forearm.stl
  9 forearm_v0/wrist_flex.stl
  10 forearm_v0/wrist_roll.stl
  11 gripper_v0/gripper_palm.stl
  12 gripper_v0/upper_finger_r.stl
  13 gripper_v0/finger_tip_r.stl
 
  12 gripper_v0/upper_finger_l.stl
  13 gripper_v0/finger_tip_l.stl
  

  2 shoulder_v0/shoulder_yaw.stl
  3 shoulder_v0/shoulder_lift.stl
  4 upper_arm_roll
  5 upper_arm_v0/upper_arm.stl
  6 upper_arm_v0/elbow_flex.stl
  7 upper_arm_v0/forearm_roll.stl
  8 forearm_v0/forearm.stl
  9 forearm_v0/wrist_flex.stl
  10 forearm_v0/wrist_roll.stl
  11 gripper_v0/gripper_palm.stl
  12 gripper_v0/upper_finger_r.stl
  13 gripper_v0/finger_tip_r.stl
  12 gripper_v0/upper_finger_l.stl
  13 gripper_v0/finger_tip_l.stl
 
  30 meshes total

*/

  //getMapToRobotTransform(base_pos[0],base_pos[1],base_pos[2],map_to_robot)

  poses.resize(robot_meshes_.size());
  for(int i=0; i <  int(poses.size()); i++)
  {
    // base, torso, right arm thru the right finger
    if(i < 14)
    {
      if(!computeFKwithKDL(jnt0_pos, base_pos, torso_pos, RIGHT, i+1, poses[i].pose))
        return false;
    }
    // right arm - left finger only
    else if(13 < i && i < 16)
    {
      if(!computeFKwithKDL(jnt0_pos, base_pos, torso_pos, RIGHT, -1*(i-1), poses[i].pose))
        return false;
    }
    // left arm thru the right finger
    else if(15 < i && i < 28)
     {
      if(!computeFKwithKDL(jnt1_pos, base_pos, torso_pos, LEFT, i-13, poses[i].pose))
        return false;
    }
    // left arm - left finger only
    else /* if(27 < i) */
    {
      if(!computeFKwithKDL(jnt1_pos, base_pos, torso_pos, LEFT, -1*(i-15), poses[i].pose))
        return false;
    }
   
    //ROS_INFO("pose: %d: %0.2f %0.2f %0.2f",i, poses[i].pose.position.x, poses[i].pose.position.y,poses[i].pose.position.z);
  }

  return true;
}

void PViz::visualizeRobotMeshes(double hue, std::string ns, int id, std::vector<geometry_msgs::PoseStamped> &poses)
{
  double r,g,b;
  marker_array_.markers.clear();
  marker_array_.markers.resize(robot_meshes_.size());
  ros::Time time = ros::Time();

  HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  for(int i = 0; i < (int)marker_array_.markers.size(); ++i)
  {
    marker_array_.markers[i].header.stamp = time;
    marker_array_.markers[i].header.frame_id = reference_frame_;
    marker_array_.markers[i].ns = ns;
    marker_array_.markers[i].type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_array_.markers[i].id = i;
    marker_array_.markers[i].action = visualization_msgs::Marker::ADD;
    marker_array_.markers[i].pose = poses.at(i).pose;
    marker_array_.markers[i].scale.x = 1.0;  
    marker_array_.markers[i].scale.y = 1.0;
    marker_array_.markers[i].scale.z = 1.0;

    marker_array_.markers[i].color.r = r;
    marker_array_.markers[i].color.g = g;
    marker_array_.markers[i].color.b = b;
    marker_array_.markers[i].color.a = 0.4;
    marker_array_.markers[i].lifetime = ros::Duration(120.0);

    //ROS_INFO("i=%d",i);
    marker_array_.markers[i].mesh_resource = robot_meshes_[i];
  }

  marker_array_publisher_.publish(marker_array_);
}

void PViz::getMaptoRobotTransform(double x, double y, double theta, KDL::Frame &frame)
{
  KDL::Rotation r1;
  r1.DoRotZ(theta);
  KDL::Vector t1(x,y,0.0);
  KDL::Frame base_footprint_in_map(r1,t1);
  frame = base_footprint_in_map;
}

void PViz::visualizeRobot(std::vector<double> &jnt0_pos, std::vector<double> &jnt1_pos, std::vector<double> &base_pos, double torso_pos, double hue, std::string ns, int id)
{
  std::vector<geometry_msgs::PoseStamped> poses;
  if(!computeFKforVisualizationWithKDL(jnt0_pos, jnt1_pos, base_pos, torso_pos, poses))

    ROS_WARN("Unable to compute forward kinematics.");
  else
    visualizeRobotMeshes(hue, ns, id, poses);
}

void PViz::visualizeRobot(std::vector<double> &jnt0_pos, std::vector<double> &jnt1_pos, BodyPose &body_pos, double hue, std::string ns, int id)
{
  double torso_pos;
  std::vector<double> base_pos(3,0);

  base_pos[0] = body_pos.x;
  base_pos[1] = body_pos.y;
  base_pos[2] = body_pos.theta;
  torso_pos = body_pos.z;

  std::vector<geometry_msgs::PoseStamped> poses;
  if(!computeFKforVisualizationWithKDL(jnt0_pos, jnt1_pos, base_pos, torso_pos, poses))

    ROS_WARN("Unable to compute forward kinematics.");
  else
    visualizeRobotMeshes(hue, ns, id, poses);
}

bool PViz::parseCSVFile(std::string filename, int num_cols, std::vector<std::vector<double> > &data)
{
  std::ifstream input_file(filename.c_str());
  if(!input_file.good())
  {
    printf("[pviz] Unable to open '%s' for reading.\n",filename.c_str());
    return false;
  }

  int row(0), col(0);
  char line[256];
  input_file.seekg(0);

  std::vector<std::vector<double> > raw_data;

  row = -1;
  raw_data.clear();

  while (input_file.getline(line, 256, ','))
  {
    raw_data.resize(raw_data.size()+1);
    raw_data[++row].resize(num_cols);
    raw_data[row][0] = atof(line);

    for(col = 1; col < num_cols; col++)
    {
      input_file.getline(line, 256, ',');
      raw_data[row][col] = atof(line);
    }
  }

  std::vector<double> zero_line(num_cols,0);
  for(int i = 0; i < int(raw_data.size()); i++)
  {
    if(raw_data[i] != zero_line)
      data.push_back(raw_data[i]);
  }

  ROS_INFO("[pviz] raw_data: num rows: %d",(int)raw_data.size());
  ROS_INFO("[pviz] data: num rows: %d num_cols: %d",(int)data.size(), int(data[0].size()));

  printf("Trajectory:\n");
  for(size_t i = 0; i < data.size(); ++i)
  {
    printf("%d: ", int(i));
    for(size_t j = 0; j < data[i].size(); ++j)
    {
      printf("% 0.2f ", data[i][j]);
    }
    printf("\n");
  }

  return true;
}

bool PViz::visualizeTrajectoryFromFile(std::string filename)
{
  double torso = 0;
  std::vector<double> jnt0_pos(7,0), jnt1_pos(7,0), base_pos(3,0);
  std::vector<std::vector<double> > traj;
  if(!parseCSVFile(filename, 18, traj))
  {
    ROS_ERROR("Failed to parse trajectory file: %s", filename.c_str());
    return false;
  }

  for(size_t i = 0; i < traj.size(); ++i)
  {
    for(size_t j = 0; j < 7; ++j)
    {
      jnt0_pos[j] = traj[i][4+j];
      jnt1_pos[j] = traj[i][12+j];
    }

    base_pos[0] = traj[i][0];
    base_pos[1] = traj[i][1];
    base_pos[2] = traj[i][2];
    torso = traj[i][3];

    ROS_INFO("[pviz] %d: visualizing robot...", int(i));
    visualizeRobot(jnt0_pos,jnt1_pos,base_pos,torso,(30*i)%300,"full_body_waypoint_"+boost::lexical_cast<std::string>(i),i);
    ros::spinOnce();
    sleep(1.0);
  }

  return true;
}

void PViz::visualizeRobotWithTitle(std::vector<double> &jnt0_pos, std::vector<double> &jnt1_pos, BodyPose &body_pos, double hue, std::string ns, int id, std::string title)
{
  visualizeRobot(jnt0_pos, jnt1_pos, body_pos, hue, ns, id);

  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = ns + "_title";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.pose.position.x = body_pos.x;
  marker.pose.position.y = body_pos.y;
  marker.pose.position.z = body_pos.z+1.5;
  
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.5;
  marker.text = title;
  marker.lifetime = ros::Duration(180.0);
  marker_publisher_.publish(marker);
}


