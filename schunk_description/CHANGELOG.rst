^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package schunk_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.14 (2019-11-20)
-------------------
* Merge pull request `#217 <https://github.com/ipa320/schunk_modular_robotics/issues/217>`_ from crthilakraj/indigo_dev
  Changed the origin of the left and right finger joints to avoid collision
* changed the origin of the left and right finger joints to avoid collision
* Merge pull request `#216 <https://github.com/ipa320/schunk_modular_robotics/issues/216>`_ from tobigs/indigo_dev
  Gazebo segfault fix
* Prepended PositionJointInterface and VelocityJointInterface with hardware_interface/ to prevent gazebo segfaults
* Merge pull request `#215 <https://github.com/ipa320/schunk_modular_robotics/issues/215>`_ from ipa320/indigo_release_candidate
  Indigo release candidate
* Merge branch 'indigo_dev' into indigo_release_candidate
* Merge branch 'indigo_dev' of github.com:ipa320/schunk_modular_robotics into indigo_release_candidate
* Merge pull request `#199 <https://github.com/ipa320/schunk_modular_robotics/issues/199>`_ from ipa320/indigo_dev
  Indigo dev
* Contributors: Christian Rauch, Felix Messmer, Tobias Seidel, chikmagalore.thilak, fmessmer

0.6.13 (2019-08-20)
-------------------
* Merge pull request `#213 <https://github.com/ipa320/schunk_modular_robotics/issues/213>`_ from PilzDE/remove-gazebo-depend
  drop gazebo_ros dependency
* drop gazebo_ros dependency
  Fixes `#209 <https://github.com/ipa320/schunk_modular_robotics/issues/209>`_
* Merge pull request `#208 <https://github.com/ipa320/schunk_modular_robotics/issues/208>`_ from christian-rauch/rm_visual_tip
  remove visual representation of virtual SDH grasp and tip links
* remove visual representation of virtual SDH grasp and tip links
* Contributors: Christian Rauch, Joachim Schleicher

0.6.12 (2018-08-16)
-------------------
* Merge pull request `#200 <https://github.com/ipa320/schunk_modular_robotics/issues/200>`_ from PilzDE/fix-pg70-urdf-limits
  fix PG+70 limits and simplify collision model
* fix limits and simplify collision model
  The gripper fingers both can move 30mm according to the datasheet
  to give a distance of 60mm between the fingers.
  The _finger_palm_link can be easily simplified as a box geometry
  for faster collision checking.
* Contributors: Joachim Schleicher, Nadia Hammoudeh Garc??a

0.6.11 (2018-07-21)
-------------------

0.6.10 (2018-01-07)
-------------------
* Merge pull request `#198 <https://github.com/ipa320/schunk_modular_robotics/issues/198>`_ from ipa320/indigo_release_candidate
  Indigo release candidate
* Merge pull request `#197 <https://github.com/ipa320/schunk_modular_robotics/issues/197>`_ from christian-rauch/mesh_face_orientation
  fix face orientation "Re-Orient all faces coherently"
* fix face orientation "Re-Orient all faces coherently"
* Merge pull request `#196 <https://github.com/ipa320/schunk_modular_robotics/issues/196>`_ from ipa-fxm/update_maintainer
  update maintainer
* update maintainer
* Merge pull request `#191 <https://github.com/ipa320/schunk_modular_robotics/issues/191>`_ from ipa-fxm/APACHE_license
  use license apache 2.0
* use license apache 2.0
* Contributors: Christian Rauch, Felix Messmer, Florian Weisshardt, ipa-fxm

0.6.9 (2017-07-17)
------------------
* Revert "python True/False"
  This reverts commit 28ee3a0bca4514baef065d4e2744b73ca7bd35f9.
* python True/False
* move gazebo_ros_control plugin
* Merge pull request `#173 <https://github.com/ipa320/schunk_modular_robotics/issues/173>`_ from ubi-agni/non_penetrating_collision
  Non-penetrating and simplified collision objects
* Added reduced collision meshes for knuckles and palm, with no interpenetration
* Added better inertia estimation for sdh (meshlab + aluminum density + adjusted to total mass)
* manually fix changelog
* use default_transmission macro
* introduce default_transmission macro
* cleanup
* newer xacro macros and missing include
* new package schunk_pg70
* Arm of cob3-6 configuration for twist controller
* Added missing include of common.xacro to access default_inertial macro
* Fix xacro warnings
* Contributors: Bruno Brito, Felix Messmer, Guillaume Walck, ipa-fxm, ipa-nhg

0.6.8 (2016-10-10)
------------------
* Enabled collison
* pw70 working
* Contributors: Andreea Tulbure, Denis ??togl

0.6.7 (2016-04-01)
------------------
* Update lwa4p.urdf.xacro with joint speed limits.
* Updated lwa4p urdf with manufacturer's specified joint limits.
* added schunk_lwa4d_custom macro and changed setting to use_factory_settings to make it easier to add default/DH config
* added comment describing invert parameters
* added axis invert macro and set up parameters for each joint -- defaults to dh convention
* added 'invert' paramter to lwa4d xacro description, default value matches DH params, -1 matches default hardware configuration
* fix meshes for lwa4p
* fix lwa4d urdf - joint orientation according to convention
* Contributors: Allison Thackston, William McMahan, ipa-fxm, wmcmahan

0.6.6 (2015-09-01)
------------------

0.6.5 (2015-08-31)
------------------

0.6.4 (2015-08-25)
------------------
* fix pg70
* migration to package format 2
* remove trailing whitespaces
* remove obsolete autogenerated mainpage.dox files
* review dependencies
* Contributors: ipa-fxm

0.6.3 (2015-06-17)
------------------
* remove unsupported calibration_rising
* proper rounding of joint_limits for lwa4d
* proper rounding of joint_limits for lwa4p
* round down joint limits for lwa4p_extended
* Update lwa4p_extended.urdf.xacro
  Velocity and Positions limits updated according to firmware settings
* reduce joint limits for moveit
* default inertia
* fix transmission for mimic joint
* allow cob3 components to be used with PositionJointInterface
* empty lines
* Fixes import error
* Safety_offset
* safety_offset
* Merge branch 'patch-1' of github.com:thiagodefreitas/schunk_modular_robotics into patch-2
  Conflicts:
  schunk_description/urdf/lwa4p_extended/lwa4p_extended_without_base.urdf.xacro
* Fixes the limits for the lwa4d
* Merge branch 'indigo_dev' of https://github.com/ipa320/schunk_modular_robotics into patch-2
  Conflicts:
  schunk_description/urdf/lwa4p_extended/lwa4p_extended.urdf.xacro
* new urdf structure
* Merge branch 'patch-1' of github.com:thiagodefreitas/schunk_modular_robotics into indigo_dev
  Conflicts:
  schunk_description/urdf/lwa4d/lwa4d.urdf.xacro
* new without_base descriptions
* updated schunk_lwa4d description
* Update pg70.urdf.xacro
* lwad description without base
* Update lwa4p_extended.urdf.xacro
  Proposing this as according to the tests on the Schunk LWA4D, the limits should be reduced from the HW limit in a range of 0.01 radians
* Update lwa4d.urdf.xacro
  According to HW limits
* Contributors: Nadia Hammoudeh Garc??a, Thiago de Freitas Oliveira Araujo, ipa-cob3-9, ipa-cob4-2, ipa-fxm, ipa-nhg, thiagodefreitas

0.6.2 (2014-12-15)
------------------
* added velocity and position controllers
* unify all schunk urdfs
* type error
* defined real limits
* defined real limits
* extend limits for axis 1
* multiple hw-interfaces for lwa4p
* prepare lwa4p for VelocityInterface
* multiple hardwareinterface tags in transmission
* multiple hardwareinterface tags in transmission
* inertia go to hell
* new collision meshes
* improve model - velocity error <0.01
* add new collision meshes
* remove wrong mesh
* Contributors: Florian Weisshardt, ipa-cob4-2, ipa-fmw, ipa-fxm, ipa-nhg

0.6.1 (2014-09-22)
------------------
* 1=true
* fix bumper plugins
* merge
* fixed center of mass and inertias
* Contributors: ipa-fxm, ipa-fxm-fm

0.6.0 (2014-09-18)
------------------
* beautification
* fix bad merge
* cleaning up
* merge
* Merge branch 'indigo_dev' into velocity_interface_controller_indigo
* Merge branch 'velocity_interface_controller' of github.com:ipa-fxm-fm/schunk_modular_robotics into velocity_interface_controller
* use velocity interface, fix link name
* added common xacro files
* back to CAD inertia
* changed inertia of link 5
* fix center of mass
* get rid off safety_controller and gazebo tags for more intuitive testing
* switch to velocity interface
* added inertias and limits for lwa4d
* Merge branch 'hydro_dev' into velocity_interface_controller
* back to CAD inertia
* changed inertia of link 5
* merge with new_model
* merge with 320
* fix center of mass
* get rid off safety_controller and gazebo tags for more intuitive testing
* switch to velocity interface
* Contributors: Felix Messmer, ipa-fxm, ipa-fxm-fm

0.5.6 (2014-08-27)
------------------

0.5.5 (2014-08-26)
------------------
* merge with hydro_dev
* spaces
* obsolete dependency
* Gazebo only detect links with inertia
* fix pg70 property name
* consistency changes due to new transmission format
* consistency changes due to latest gazebo tag format
* remove unused meshes
* fix arm_1_link mesh + use collision stl
* merge with latest 320 updates
* pg70 description and fixed origins for lwa4d
* pg70 setup
* mesh file
* Coloured mesh files
* use meshes with reduced vertices and reduce joint_limits for moveit_config
* use meshes with reduced vertices and reduce joint_limits for moveit_config
* cleaning up after testing
* lwa4d: fixed offsets error
* inertias for arm_1_link
* new meshes
* temporary modifications for easier controller tuning
* use inertias and physic properties from controller_tuning tests
* modifications in tranmissions, removing of bumpers and small cleanups
* Renamed links and added shoulder model
* mesh for shoulder added
* origin for collision model is in the center of the box
* pg70 collada model
* wrong lenght
* materials should not be loaded in the components urdf
* beautify mesh files
* Merge pull request `#81 <https://github.com/ipa320/schunk_modular_robotics/issues/81>`_ from ipa320/hydro_release_candidate
  bring back changes from Hydro release candidate
* New maintainer
* Redefined color LightGrey
* Contributors: Alexander Bubeck, Felix Messmer, Nadia Hammoudeh Garc??a, Tim Fr??hlich, ipa-cob3-8, ipa-fxm, ipa-nhg

0.5.4 (2014-03-28)
------------------

0.5.3 (2014-03-27)
------------------
* Merge branch 'hydro_dev' into hydro_release_candidate
* install tags
* Merge branch 'hydro_dev' of github.com:ipa320/schunk_modular_robotics into hydro_dev
* some catkin_lint
* Contributors: Florian Weisshardt, ipa-fxm

0.5.2 (2014-03-27)
------------------

0.5.1 (2014-03-20)
------------------
* update xacro file format
* merge with groovy
* meshes files for lwa4p_extended
* added meshes files for lwa4p_extended
* tested on real arm
* 27.02. current status
* new meshes
* Tested on real arm
* Fixed arm_7_joint position
* bring in groovy updates
* Fix mesh files for lwa4d
* Added calibration arm_1_calibrationg_rising
* description for the a new lwa4p version
* Adjust lwa limits
* adapt limits for lwa and lwa_extended
* update lwa4d description
* fix arm_6_joint
* update transmission for schunk components
* update xmlns + beautifying
* transmission for new simulation controllers
* 2DOF Hack for finger
* fix fingers
* update pg70
* add pg70 gripper
* Corrected xacro files for hydro.
* Removed instalation of gazebo folder which doesn't exist.
* Updated lwa4d description
* Created lwa4d urdf model
* remove install command for gazebo subdirectory
* merge
* More changes from powerball to lwa4d
* Changed from powerball to lwa4p
* remove mesh file generation
* installation stuff
* remove generation of mesh files
* Initial catkinization. Still a linking error in sdh lib.
* some more fixes and cleaning up for gazebo simulation
* fix sdh description according to new gazebo format
* fix blue color
* Groovy migration
* adjust color settings
* change to light grey
* Reorganized list of colors
* Redefined colors
* Merge branch 'master' of github.com:ipa320/schunk_modular_robotics
* update limits for lwa
* Renamed the colors
* Redefined Schunk component colors for gazebo and rviz
* merge
* Fixed arm_0_link origin
* modified mesh files
* powerball stl changes
* Revised powerball  urdf and mesh files
* New meshes files for powerball
* fixes for powerball arm urdf
* New colors for powerball in simulation
* changed stl files not using solid
* changed stlb links to stl
* New model schunk powerball
* fix lwa
* renamed to schunk names
* renamed arm to lwa
* rename from arm to lwa
* renamed arm to lwa
* moved schunk desc
* Contributors: Alexander Bubeck, Denis ??togl, Frederik Hegger, IPR-SR2, Thiago de Freitas, abubeck, fmw, ipa-cob3-5, ipa-cob3-6, ipa-fmw, ipa-fxm, ipa-nhg, ipa-tys, rmb-om
