Moveit Tutorials for the PR2
============================

- :doc:`The Kinematics tutorial</kinematics/src/doc/kinematics_tutorial>` shows the basics of using the C++ API for kinematics. It includes forward and inverse kinematics, setting and getting joint values, dealing with joint limits and computing jacobians.
- :doc:`The Planning Scene tutorial</planning/src/doc/planning_scene_tutorial>` shows the basics of using the C++ API for the Planning Scene. It shows you how to use the API for collision checking (including self-collision checking), constraint checking (including specifying user defined constraints), getting contact information and also setting the Allowed Collision Matrix. 
- :doc:`The Motion Planning tutorial</planning/src/doc/motion_planning_api_tutorial>` shows the basics of using the C++ API for loading a motion planner (using the ROS pluginlib library) and calling it. It includes the API for setting joint space goals, pose goals and for specifying kinematic constraints on the motion. 
- :doc:`The Motion Planning Pipeline tutorial</planning/src/doc/planning_pipeline_tutorial>` shows the basics of using the C++ API for loading and calling the motion planning pipeline. The tutorial also illustrates the use of planning request adapters that are used to pre-process and post-process motion plans. 
