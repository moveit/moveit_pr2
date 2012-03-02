/* \author Benjamin Cohen */

#ifndef _PVIZ_
#define _PVIZ_

#include <string>
#include <fstream>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <boost/lexical_cast.hpp>
#include <kinematics_msgs/GetPositionIK.h>
#include <kinematics_msgs/GetPositionFK.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <pviz/body_pose.h>

enum{ RIGHT, LEFT };

class PViz
{
  public:
    
    PViz();

    ~PViz();

    /* \brief set reference frame of visualizations */
    void setReferenceFrame(std::string frame) {reference_frame_ = frame;};

    /**************** Kinematics ****************/

    /* \brief initialize the KDL chain for the robot arm */
    bool initKDLChain();

    /* \brief compute FK for the pr2 arm meshes using the KDL chain */
    bool computeFKforVisualizationWithKDL(const std::vector<double> &jnt_pos, double torso_pos, int arm, std::vector<geometry_msgs::PoseStamped> &poses);

    /* \brief compute FK for a joint configuration using the KDL chain */
    bool computeFKwithKDL(const std::vector<double> &angles, std::vector<double> &base_pos, double torso_pos, int arm, int frame_num, geometry_msgs::Pose &pose);

    /* \brief compute FK for the pr2 arm meshes using the kinematic service*/
    bool computeFKforVisualization(const std::vector<double> &jnt_pos, double torso_pos, int arm, std::vector<geometry_msgs::PoseStamped> &poses);

    bool computeFKforVisualizationWithKDL(const std::vector<double> &jnt0_pos, std::vector<double> &jnt1_pos, std::vector<double> &base_pos, double torso_pos, std::vector<geometry_msgs::PoseStamped> &poses);

    void getMaptoRobotTransform(double x, double y, double theta, KDL::Frame &frame);

    /**************** Robot Meshes ****************/
    
    /* \brief display a throttled set of arm configurations in a trajectory
     * by default throttle = 5 */
    void visualizeArmConfigurations(const std::vector<std::vector<double> > &traj, int arm, int throttle);

    void visualizeRobotMeshes(double hue, std::string ns, int id, std::vector<geometry_msgs::PoseStamped> &poses);

    void visualizeRobot(std::vector<double> &jnt0_pos, std::vector<double> &jnt1_pos, std::vector<double> &base_pos, double torso_pos, double hue, std::string ns, int id);

    void visualizeRobot(std::vector<double> &jnt0_pos, std::vector<double> &jnt1_pos, BodyPose &body_pos, double hue, std::string ns, int id);

    void visualizeRobotWithTitle(std::vector<double> &jnt0_pos, std::vector<double> &jnt1_pos, BodyPose &body_pos, double hue, std::string ns, int id, std::string title);
    /**************** Shapes, Text & Lines ****************/

    /* \brief visualize a pose (sphere, arrow, string of text) */
    void visualizePose(const std::vector<double> &pose, std::string text);

    void visualizePose(const geometry_msgs::Pose &pose, std::string text);

    /* \brief visualize a list of poses (sphere, arrow, pose index number) */
    void visualizePoses(const std::vector<std::vector<double> > &poses);
 
    /* \brief visualize cuboids */
    void visualizeObstacles(const std::vector<std::vector<double> > &obstacles);
   
    void visualize3DPath(std::vector<std::vector<double> > &dpath);

     /* \brief display a sphere */
    void visualizeSphere(std::vector<double> pose, int color, std::string text, double radius);
    
    /* \brief display a list of spheres of the same radius and color */
    void visualizeSpheres(const std::vector<std::vector<double> > &pose, int color, std::string text, double radius);

    void visualizeSpheres(const std::vector<std::vector<double> > &pose, int color, std::string text, std::vector<double> &radius);
 
    void visualizeSpheres(const std::vector<std::vector<double> > &pose, int color, std::string text);

    void visualizeAttachedObject(const std::vector<double> angles);

    void visualizeLine(const std::vector<geometry_msgs::Point> points, std::string ns, int id, int hue, double thickness);

    void visualizeText(geometry_msgs::Pose pose, std::string text, std::string ns, int id, int hue);

    void visualizeText(geometry_msgs::Pose pose, std::string text, std::string ns, int id, std::vector<double> color, double size);

    void visualizeCube(geometry_msgs::PoseStamped pose, int color, std::string ns, int id, std::vector<double> dim);

    /* \brief display a list of states (xyz coordinates) (intended for use with sbpl) */
    void visualizeBasicStates(const std::vector<std::vector<double> > &states, const std::vector<double> &color, std::string name, double size);
    
    /* \brief display a list of states (xyz coordinates with rpy arrows) (intended for use with sbpl) */
    void visualizeDetailedStates(const std::vector<std::vector<double> > &states, const std::vector<std::vector<double> >&color, std::string name, double size);

    void deleteVisualizations(std::string ns, int max_id);

    void printKDLChain(std::string name, KDL::Chain &chain);

    bool parseCSVFile(std::string filename, int num_cols, std::vector<std::vector<double> > &data);
  
    bool visualizeTrajectoryFromFile(std::string filename);

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;
    ros::Publisher marker_array_publisher_;
    ros::Publisher marker_publisher_;

    int num_joints_;

    visualization_msgs::MarkerArray marker_array_;
    visualization_msgs::Marker marker_;

    std::vector<std::string> arm_joint_names_;
    std::vector<std::string> arm_link_names_;
    std::vector<std::string> torso_joint_names_;
    std::vector<std::string> torso_link_names_;

    std::vector<std::string> arm_meshes_;
    std::vector<std::string> gripper_meshes_;
    std::vector<std::string> torso_meshes_;
    std::vector<std::string> base_meshes_;
    std::vector<std::string> robot_meshes_;
    std::vector<geometry_msgs::Vector3> pr2_arm_meshes_scale_;

    std::vector<std::string> side_;
    std::vector<std::string> side_full_;
    std::vector<std::string> fk_service_name_;
    std::vector<std::string> ik_service_name_;

    KDL::JntArray jnt_pos_in_;
    KDL::JntArray jnt_pos_out_;
    KDL::Frame p_out_;
    KDL::Chain chain_;
    KDL::Tree kdl_tree_;
    std::vector<KDL::ChainFkSolverPos_recursive *> fk_rsolver_;
    std::vector<KDL::ChainFkSolverPos_recursive *> fk_lsolver_;

    std::string arm_name_;
    std::string arm_config_file_;
    std::string reference_frame_;

    std::vector<double> start_config_;
    std::vector<double> goal_pose_;
    std::vector<std::vector<double> > cubes_;
    double position_tolerance_;
    double orientation_tolerance_;
    bool goal_is_6dof_;
};

#endif

