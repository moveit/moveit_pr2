Moveit Tutorials for the PR2
============================
Beginner Tutorials
==================
- :doc:`The Move Group Interface tutorial</planning/src/doc/move_group_interface_tutorial>` shows how to use the C++ interface to move_group. This is the primary interface that most users of MoveIt! should use.

- :doc:`The Move Group Python Interface tutorial</planning/scripts/doc/move_group_python_interface_tutorial>` shows how to use the Python interface to move_group. This is the primary interface that most users of MoveIt! should use from Python.

Advanced Tutorials
==================
- :doc:`The Kinematics tutorial</kinematics/src/doc/kinematics_tutorial>` shows the basics of using the C++ API for kinematics. It includes forward and inverse kinematics, setting and getting joint values, dealing with joint limits and computing jacobians.

- :doc:`The Planning Scene tutorial</planning/src/doc/planning_scene_tutorial>` shows the basics of using the C++ API for the Planning Scene. It shows you how to use the API for collision checking (including self-collision checking), constraint checking (including specifying user defined constraints), getting contact information and also setting the Allowed Collision Matrix. 

- :doc:`The Planning Scene ROS API tutorial</planning/src/doc/planning_scene_ros_api_tutorial>` shows the basics of using the ROS API for the Planning Scene through the use of Planning Scene diffs. It shows you how to add new objects into the planning scene, attach the objects to the robot, detach the objects from the robot and then remove the object from the planning scene.

- :doc:`The Motion Planning tutorial</planning/src/doc/motion_planning_api_tutorial>` shows the basics of using the C++ API for loading a motion planner (using the ROS pluginlib library) and calling it. It includes the API for setting joint space goals, pose goals and for specifying kinematic constraints on the motion. 

- :doc:`The Motion Planning Pipeline tutorial</planning/src/doc/planning_pipeline_tutorial>` shows the basics of using the C++ API for loading and calling the motion planning pipeline. The tutorial also illustrates the use of planning request adapters that are used to pre-process and post-process motion plans. 

Configuration Tutorials
=======================
- :doc:`The Kinematics configuration tutorial</kinematics/src/doc/kinematics_configuration>` shows you the configuration details for setting up kinematics (including setting up position only inverse kinematics). 

- :doc:`The 3D Perception configuration tutorial</planning/src/doc/perception_configuration>` shows you the configuration details for setting up 3D sensors with MoveIt!.

- :doc:`Controller Configuration tutorial</planning/src/doc/controller_configuration>` shows you the configuration details for setting up your robot's controllers with MoveIt!.

Links
=====

 * Back to :moveit_website:`Moveit Tutorials <tutorials>`
