#include <planning_scene_monitor/planning_scene_monitor.h>
#include <kinematics_constraint_aware/kinematics_solver_constraint_aware.h>
#include <kinematics_plugin_loader/kinematics_plugin_loader.h>
#include <grasp_place_evaluation/grasp_evaluator_fast.h>

#include <gtest/gtest.h>


class MoveitManipulationTester : public ::testing::Test {

protected:


  virtual void SetUp() {

    ros::NodeHandle nh;
    
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    
    planning_scene_diff_.reset(new planning_scene::PlanningScene(planning_scene_monitor_->getPlanningScene()));
    
    kinematics_plugin_loader_.reset(new kinematics_plugin_loader::KinematicsPluginLoader());

    kinematics_plugin_loader::KinematicsLoaderFn kinematics_allocator = kinematics_plugin_loader_->getLoaderFunction();
    
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
    obj.header.frame_id = planning_scene_diff_->getPlanningFrame();
    obj.operation = moveit_msgs::CollisionObject::ADD;
    obj.id = "obj";
    obj.shapes.resize(1);
    obj.shapes[0].type = shape_msgs::Shape::CYLINDER;
    obj.shapes[0].dimensions.resize(2);
    obj.shapes[0].dimensions[0] = .02;
    obj.shapes[0].dimensions[1] = .1;
    obj.poses.resize(1);
    obj.poses[0].position.x = .65;
    obj.poses[0].position.y = -.15;
    obj.poses[0].position.z = 1.0;
    obj.poses[0].orientation.w = 1.0;

    planning_scene_diff_->processCollisionObjectMsg(obj);
  }
  
  virtual void TearDown() {
    grasp_evaluator_fast_.reset();
    planning_scene_diff_.reset();
    planning_scene_monitor_.reset();
    kinematics_plugin_loader_.reset();
  }

protected:

  boost::shared_ptr<kinematics_plugin_loader::KinematicsPluginLoader> kinematics_plugin_loader_;
  boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
  boost::shared_ptr<planning_scene::PlanningScene> planning_scene_diff_;
  boost::shared_ptr<grasp_place_evaluation::GraspEvaluatorFast> grasp_evaluator_fast_;
};

TEST_F(MoveitManipulationTester, GraspCollideAttachedObject) {

  ROS_INFO_STREAM("Test");

  moveit_msgs::CollisionObject obj;
  obj.header.frame_id = planning_scene_diff_->getPlanningFrame();
  obj.operation = moveit_msgs::CollisionObject::ADD;
  obj.id = "box";
  obj.shapes.resize(1);
  obj.shapes[0].type = shape_msgs::Shape::BOX;
  obj.shapes[0].dimensions.resize(3);
  obj.shapes[0].dimensions[0] = .1;
  obj.shapes[0].dimensions[1] = .1;
  obj.shapes[0].dimensions[2] = .1;
  obj.poses.resize(1);
  obj.poses[0].position.x = .65;
  obj.poses[0].position.y = -.15;
  obj.poses[0].position.z = 1.1;
  obj.poses[0].orientation.w = 1.0;
  
  planning_scene_diff_->processCollisionObjectMsg(obj);

  moveit_manipulation_msgs::PickupGoal goal;
  goal.arm_name = "right_arm";
  goal.collision_object_name = "obj";
  
  goal.target.collision_name = "obj";
  goal.target.reference_frame_id = planning_scene_diff_->getPlanningFrame();

  goal.lift.direction.vector.z=1;
  goal.lift.desired_distance = .1;

  std::vector<moveit_manipulation_msgs::Grasp> grasps;
  grasps.resize(1);
  grasps[0].grasp_pose.position.x = .50;
  grasps[0].grasp_pose.position.y = -.15;
  grasps[0].grasp_pose.position.z = 1.0;
  grasps[0].grasp_pose.orientation.w = 1.0;
  grasps[0].desired_approach_distance = .1;
  grasps[0].min_approach_distance = .1;
 
  grasp_place_evaluation::GraspExecutionInfoVector execution_info;

  grasp_evaluator_fast_->testGrasps(planning_scene_diff_,
                                    &planning_scene_diff_->getCurrentState(),
                                    goal,
                                    grasps,
                                    execution_info,
                                    true);
  ASSERT_EQ(execution_info.size(),1);
  //low enough that box should be in collision with the gripper
  EXPECT_EQ(execution_info[0].result_.result_code, moveit_manipulation_msgs::GraspResult::LIFT_IN_COLLISION);

  collision_detection::CollisionWorld::ObjectConstPtr obj_ptr = planning_scene_diff_->getWorld()->getObject("box");

  //out of the way altogether
  obj.poses[0].position.z = 2.0;
  Eigen::Affine3d np;
  planning_models::poseFromMsg(obj.poses[0], np);
  planning_scene_diff_->getWorldNonConst()->moveShapeInObject("box", obj_ptr->shapes_[0], np);

  grasp_evaluator_fast_->testGrasps(planning_scene_diff_,
                                    &planning_scene_diff_->getCurrentState(),
                                    goal,
                                    grasps,
                                    execution_info,
                                    true);
  ASSERT_EQ(execution_info.size(),1);
  //low enough that box should be in collision with the gripper
  EXPECT_EQ(execution_info[0].result_.result_code, moveit_manipulation_msgs::GraspResult::SUCCESS);

  //only in contact with attached object
  obj.poses[0].position.z = 1.15;
  planning_models::poseFromMsg(obj.poses[0], np);
  planning_scene_diff_->getWorldNonConst()->moveShapeInObject("box", obj_ptr->shapes_[0], np);

  grasp_evaluator_fast_->testGrasps(planning_scene_diff_,
                                    &planning_scene_diff_->getCurrentState(),
                                    goal,
                                    grasps,
                                    execution_info,
                                    true);
  ASSERT_EQ(execution_info.size(),1);
  EXPECT_EQ(execution_info[0].result_.result_code, moveit_manipulation_msgs::GraspResult::LIFT_IN_COLLISION);
}

TEST_F(MoveitManipulationTester, GraspInWorldFrameOK) {

  moveit_manipulation_msgs::PickupGoal goal;
  goal.arm_name = "right_arm";
  goal.collision_object_name = "obj";
  
  goal.target.collision_name = "obj";
  goal.target.reference_frame_id = planning_scene_diff_->getPlanningFrame();

  goal.lift.direction.vector.z=1;
  goal.lift.desired_distance = .1;

  std::vector<moveit_manipulation_msgs::Grasp> grasps;
  grasps.resize(1);
  grasps[0].grasp_pose.position.x = .50;
  grasps[0].grasp_pose.position.y = -.15;
  grasps[0].grasp_pose.position.z = 1.0;
  grasps[0].grasp_pose.orientation.w = 1.0;
  grasps[0].desired_approach_distance = .1;
  grasps[0].min_approach_distance = .1;
 
  grasp_place_evaluation::GraspExecutionInfoVector execution_info;

  grasp_evaluator_fast_->testGrasps(planning_scene_diff_,
                                    &planning_scene_diff_->getCurrentState(),
                                    goal,
                                    grasps,
                                    execution_info,
                                    true);
  ASSERT_EQ(execution_info.size(),1);
  EXPECT_EQ(execution_info[0].result_.result_code, moveit_manipulation_msgs::GraspResult::SUCCESS);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "pr2_moveit_manipulation_tests");
  return RUN_ALL_TESTS();
}

