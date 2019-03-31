^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_moveit_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.1 (2019-03-31)
------------------

0.7.0 (2018-04-27)
------------------
* Migrate to format2
* Fix xacro warnings
* [moveit.rviz] Fix always showing init pose. (`#89 <https://github.com/ros-planning/moveit_pr2/issues/89>`_)
  Init pose would always be shown in addition to the current {start, goal} poses. This is useless and confusing.
  Although only thing I did was to remove `RobotState` panel, and `save as` to overwrite the same `.rviz` file, there are some changes that seem unrelated. I do not know what happened but if some of those changes should be reverted let me know.
* allow to open pr2_moveit_config in setup_assistant
  The temp-file does not exist and the moveit_config
  already run_depends on pr2_description anyway.
* add arms_and_torso group
* Contributors: Bence Magyar, Christian Dornhege, Isaac I.Y. Saito, tarukosu, v4hn

0.6.4 (2017-06-20)
------------------
* [fix] Fix always showing init pose. (`#89 <https://github.com/ros-planning/moveit_pr2/issues/89>`_)
* [capability] add arms_and_torso group
* [enhance] allow to open pr2_moveit_config in setup_assistant
* Contributors: Isaac I.Y. Saito, tarukosu, v4hn

0.6.3 (2016-06-24)
------------------
* [feat] pr2_moveit_config: add additional capabilities
* [feat] rviz config: default to left_arm group
  Otherwise the object inserted into the planning scene
  in planning_scene_ros_api_tutorial is covered by
  the interactive marker of the right_arm group.
* [feat] made jiggle_fraction parameter usable from outside
* [feat] add arms_and_torso group
* Contributors: Tobias Fromm, tarukosu, v4hn

0.6.2 (2016-02-05)
------------------

0.6.1 (2015-01-16)
------------------

0.6.0 (2015-01-14)
------------------
* deleted duplicate debug xml tag in launchfile
* Contributors: arjungm

0.5.7 (2014-03-23)
------------------

0.5.6 (2014-02-27)
------------------

0.5.5 (2014-01-03)
------------------
* adding move group interface tutorial and sphinx docs

0.5.4 (2013-12-03)
------------------
* fixing broken tests for changes in robot_state
* fix `#26 <https://github.com/ros-planning/moveit_pr2/issues/26>`_.

0.5.3 (2013-09-23)
------------------
* enable more information output in demo mode
* keeping in sync with the setup assistant generated pkg
* add source param for joint_state_publisher
* use fake controllers for demo.launch
* use .xml suffix for included launch files

0.5.2 (2013-08-13)
------------------
* changing default object recognition behavior to listen on topic
* add debug flag to demo.launch
* add condition for starting mongodb as done in template
* update deps
* modified tabletop object recognition file to not trigger automatically
* lowering octomap resolution
* added ork config file
* new object recognition launch file

0.5.1 (2013-07-15)
------------------
* remove hacks for groovy

0.5.0 (2013-07-15)
------------------

0.4.3 (2013-07-12)
------------------
* remove hardcoded path
* Tweaking pr2 settings
* Created launch file for Benchmark GUI
