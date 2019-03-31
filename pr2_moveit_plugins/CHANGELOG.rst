^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_moveit_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.1 (2019-03-31)
------------------
* use urdf typedefs to stay compatible with boost vs std ptrs
* Contributors: v4hn

0.7.0 (2018-04-27)
------------------
* Maintainer update & order dependencies
* pluginlib headers migration
* Migrate to format2
* Add Bence as maintainer
* GetKinematicSolverInfo -> KinematicSolverInfo
* compile with c++11
* Explicitly convert shared_ptr to bool for c++11 compatibility.
* Contributors: Bence Magyar, Christian Dornhege, Dave Coleman, Isaac I.Y. Saito, Maarten de Vries, v4hn

0.6.4 (2017-06-20)
------------------
* [enhance] Explicitly convert shared_ptr to bool for c++11 compatibility.
* Contributors: Maarten de Vries

0.6.3 (2016-06-24)
------------------

0.6.2 (2016-02-05)
------------------
* Fix message dependencies.
* Contributors: Christian Dornhege

0.6.1 (2015-01-16)
------------------
* update maintainer while debugging build errors
* add build depend on cmake_modules
* Contributors: Michael Ferguson

0.6.0 (2015-01-14)
------------------
* build system fix
* Fix deprecated class loader call, renamed global variables to have _, cleaned up launch files
* Contributors: Dave Coleman, Ioan Sucan

0.5.7 (2014-03-23)
------------------

0.5.6 (2014-02-27)
------------------

0.5.5 (2014-01-03)
------------------

0.5.4 (2013-12-03)
------------------
* for some reason it seems the Eigen allocator is not called when constructing objects on stack (?); this fixes things for 32bit OS

0.5.3 (2013-09-23)
------------------

0.5.2 (2013-08-13)
------------------
* make headers and author definitions aligned the same way; white space fixes
* fixes to gripper controller

0.5.1 (2013-07-15)
------------------
* remove hacks for groovy

0.5.0 (2013-07-15)
------------------

0.4.3 (2013-07-12)
------------------
* white space fixes (tabs are now spaces)
* adding options struct to kinematics base
