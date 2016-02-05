^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_moveit_tutorials
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.2 (2016-02-05)
-----------
* Convert deprecated shape_tools dependency to geometric_shapes
  for pick and place tutorial
* Install move_group_python_interface_tutorial.py in tutorials
* pr2_moveit_tutorials/package.xml add run_depends
  on pr2 kinematics, moveit config, and moveit plugins in tutorials
* Contributors: Ian McMahon, Steven Peters

0.6.1 (2015-01-16)
------------------
* update maintainer while debugging build errors
* Contributors: Michael Ferguson

0.6.0 (2015-01-14)
------------------
* Fixed launch file name in api_tutorial doc.
* Added missing dependency on moveit_fake_controller_manager to tutorial.
* Contributors: Dave Hershberger

0.5.7 (2014-03-23)
------------------
* update build system for ROS indigo
* Contributors: Ioan Sucan

0.5.6 (2014-02-27)
------------------
* Update controller_configuration.rst
  Fix cut and paste errors.
* fix topics to check for while debugging
* fixing
* added example for dual arm pose goals
* move_group tutorial: update expected output
* cleanup tutorial text (comments)
* move_group tutorial: fix timing & error checking
* move_group interface tutorial: plan around obstacle
  add code to generate a plan while a world box is in the way.
* improve move_group_interface tutorial
  Clarify use of display_publisher.
  Fix cartesian path request so it can succeed.
  Add path planning with obstacle.
* python tutorial: cleanup imports
* fix copyright and link
* python tutorial: update expected output description
* python tutorial: add cartesian paths
* python tutorial: add joint state plan
* python tutorial: fix display_trajectory_publisher
* python tutorial: fix title
* python tutorial: fix typo
* python tutorial: add go command
* python tutorial: add MoveGroupCommander interface
* add python move_group interface tutorial
* python interface tutorial
* Contributors: Acorn Pooley, Sachin Chitta

0.5.5 (2014-01-03)
------------------
* Switched moveit_website url from numeric to moveit.ros.org.
* Added links to all tutorials pointing back to main tutorials page on website.
* fixing expected results
* adding controller configuration tutorial
* adding kinematics and perception configuration
* adding cartesian path into tutorial
* adding move group interface tutorial and sphinx docs
* adding a tutorial for the planning scene ros api using diffs
* moving .rst files so their names match tutorial names, renaming cpp files to better reflect tutorials
* adding sphinx information for planning pipeline and motion planning interface tutorials
* Changed title of planning tutorial from 'Environment Representation' to 'Planning Scene'

0.5.4 (2013-12-03)
------------------
* Ported PlanningScene tutorial to sphinx, and fixed some bugs in its code.
* removed accidentally-copied png file from rviz tutorial
* Added external links to kinematics tutorial
* Converted kinematics tutorial from wiki to sphinx in source.
* add an example for using move_group_interface with constraints

0.5.3 (2013-09-23)
------------------
* clear potential attached body before pick&place
* fix launch files
* use new pick&place messages

0.5.2 (2013-08-13)
------------------
* make headers and author definitions aligned the same way; white space fixes

0.5.1 (2013-07-15)
------------------

0.5.0 (2013-07-15)
------------------

0.4.3 (2013-07-12)
------------------
* white space fixes (tabs are now spaces)
* port to new interface for planners
