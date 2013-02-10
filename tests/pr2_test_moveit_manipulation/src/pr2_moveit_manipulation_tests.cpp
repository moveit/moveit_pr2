#include <planning_scene_monitor/planning_scene_monitor.h>
#include <kinematics_constraint_aware/kinematics_solver_constraint_aware.h>
#include <kinematics_plugin_loader/kinematics_plugin_loader.h>
#include <grasp_place_evaluation/grasp_evaluator_fast.h>

#include <gtest/gtest.h>


class MoveitManipulationTester : public testing::Test {

protected:


  virtual void SetUp() {
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    
    planning_scene_diff_.reset(new planning_scene::PlanningScene(planning_scene_monitor_->getPlanningScene()));

    boost::shared_ptr<kinematics_plugin_loader::KinematicsPluginLoader> 
      kinematics_plugin_loader(new kinematics_plugin_loader::KinematicsPluginLoader());

    kinematics_plugin_loader::KinematicsLoaderFn kinematics_allocator = kinematics_plugin_loader->getLoaderFunction();

    const planning_models::RobotModel::JointModelGroup* right_arm_group 
      = planning_scene_diff_->getRobotModel()->getJointModelGroup("right_arm"); 

    const planning_models::RobotModel::JointModelGroup* left_arm_group 
      = planning_scene_diff_->getRobotModel()->getJointModelGroup("left_arm"); 

    std::map<std::string, kinematics::KinematicsBasePtr> solver_map;
    solver_map["right_arm"] = kinematics_allocator(right_arm_group);
    solver_map["left_arm"] = kinematics_allocator(left_arm_group);

    grasp_evaluator_fast_.reset(new grasp_place_evaluation::GraspEvaluatorFast(planning_scene_diff_->getRobotModel(),
                                                                               solver_map));

    moveit_msgs::CollisionObject obj;
    obj.operation = moveit_msgs::CollisionObject::ADD;
    obj.id = "obj";
    obj.shapes.resize(1);
    obj.shapes[0].type = shape_msgs::Shape::CYLINDER;
    obj.shapes[0].dimensions.resize(2);
    obj.shapes[0].dimensions[0] = .02;
    obj.shapes[0].dimensions[1] = .1;
    obj.poses.resize(1);
    obj.poses.position.x = .5;
    obj.poses.position.y = -.15;
    obj.poses.position.z = .6;
    obj.poses.orientation.w = 1.0;

    planning_scene_diff_->processCollisionObjectMsg(obj);
  }
  
  virtual void TearDown() {
    grasp_evaluator_fast_.reset();
    planning_scene_diff_.reset();
    planning_scene_monitor_.reset();
  }

protected:

  boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
  boost::shared_ptr<planning_scene::PlanningScene> planning_scene_diff_;
  boost::shared_ptr<grasp_place_evaluation::GraspEvaluatorFast> grasp_evaluator_fast_;
};

// TEST_F(MoveitManipulationTester, GraspOK) {

//   moveit_manipulation_msgs::
  
// }

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "pr2_moveit_manipulation_tests");
  return RUN_ALL_TESTS();
}

