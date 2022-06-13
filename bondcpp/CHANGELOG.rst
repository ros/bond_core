^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bondcpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.2 (2022-04-01)
------------------

3.0.1 (2021-01-26)
------------------
* Fix cpplint/uncrustify errors.
* Add build dependencies on pkg-config.
* Contributors: Chris Lalancette

3.0.0 (2021-01-26)
------------------
* [Fixing CI] using chrono literals for durations from rclcpp API update (`#69 <https://github.com/ros/bond_core/issues/69>`_)
* Contributors: Steve Macenski

2.0.0 (2020-11-05)
------------------
* Lifecycle support 2 (`#67 <https://github.com/ros/bond_core/issues/67>`_)
* find uuid correctly on ubuntu and osx (`#55 <https://github.com/ros/bond_core/issues/55>`_)
* Ros2 devel (`#54 <https://github.com/ros/bond_core/issues/54>`_)
* Make Michael Carroll the maintainer (`#40 <https://github.com/ros/bond_core/issues/40>`_)
* Contributors: Karsten Knese, Mikael Arguedas, Steve Macenski

1.8.3 (2018-08-17)
------------------
* Argument to Boost Milliseconds must be integral in Boost >= 1.67 (`#37 <https://github.com/ros/bond_core/issues/37>`_)
  * Argument to Boost milliseconds  must be integral
  * Fix style
  * More consistent type
* Contributors: Paul-Edouard Sarlin

1.8.2 (2018-04-27)
------------------
* uuid dependency fixup (`#36 <https://github.com/ros/bond_core/issues/36>`_)
  * dont export uuid dependency as this isnt anywhere in the public api
  * fixx uuid dependency in test_bond as well
* Contributors: Mikael Arguedas

1.8.1 (2017-10-27)
------------------
* fix package.xml to comply with schema (`#30 <https://github.com/ros/bond_core/issues/30>`_)
* Contributors: Mikael Arguedas

1.8.0 (2017-07-27)
------------------
* Use SteadyTime and SteadyTimer for bond timeouts (`#18 <https://github.com/ros/bond_core/issues/18>`_)
* C++ style (`#28 <https://github.com/ros/bond_core/issues/28>`_)
* switch to package format 2 (`#27 <https://github.com/ros/bond_core/issues/27>`_)
* remove trailing whitespaces (`#26 <https://github.com/ros/bond_core/issues/26>`_)
* Contributors: Felix Ruess, Mikael Arguedas

1.7.19 (2017-03-27)
-------------------
* fix unused var warning
* Contributors: Mikael Arguedas

1.7.18 (2016-10-24)
-------------------
* fix -isystem /usr/include build breakage in gcc6
* Contributors: Mikael Arguedas

1.7.17 (2016-03-15)
-------------------
* update maintainer
* Contributors: Mikael Arguedas

1.7.16 (2014-10-30)
-------------------
* Fix depedency version
* Contributors: Esteve Fernandez

1.7.15 (2014-10-28)
-------------------
* Added version dependency.
* Removed redundant include_directories
* Added cmake_modules in alphabetical order
* Use FindUUID.cmake from cmake-modules to find the UUID libraries `#8 <https://github.com/ros/bond_core/pull/8>`_
* Contributors: Esteve Fernandez

1.7.14 (2014-05-08)
-------------------
* Update maintainer field
* Contributors: Esteve Fernandez, Vincent Rabaud

1.7.13 (2013-08-21)
-------------------
* Use c++ style reinterpret_cast rather than c style cast
* use rpc for uuid on windows
* add missing archive/library/runtime destinations for library
* Contributors: David Hodo, Dirk Thomas, William Woodall

1.7.12 (2013-06-06)
-------------------
* fix dependency on exported targets if the variable is empty
* use EXPORTED_TARGETS variable instead of explicit target names
* Contributors: Dirk Thomas

1.7.11 (2013-03-13)
-------------------

1.7.10 (2013-01-13)
-------------------
* add missing link library uuid `#6 <https://github.com/ros/bond_core/issues/6>`_
* Contributors: Dirk Thomas

1.7.9 (2012-12-27)
------------------
* modified dep type of catkin
* Contributors: Dirk Thomas

1.7.8 (2012-12-13)
------------------

1.7.7 (2012-12-06)
------------------
* Added missing link against catkin_LIBRARIES
* Updated url tags in package.xml's `#1 <https://github.com/ros/bond_core/pull/1>`_
* updated catkin_package(DEPENDS)
* Contributors: Dirk Thomas, William Woodall

1.7.6 (2012-10-30)
------------------
* fix catkin function order
* Contributors: Dirk Thomas

1.7.5 (2012-10-27)
------------------
* clean up package.xml files
* add missing target dependency to gencpp
* Contributors: Dirk Thomas

1.7.4 (2012-10-06)
------------------

1.7.3 (2012-10-02 00:19)
------------------------
* fix package building issues
* Contributors: Vincent Rabaud

1.7.2 (2012-10-02 00:06)
------------------------
* add the missing catkin dependency
* Contributors: Vincent Rabaud

1.7.1 (2012-10-01 19:00)
------------------------
* add missing dependencies
* Contributors: Vincent Rabaud

1.7.0 (2012-10-01 16:51)
------------------------
* catkinize bond
* catkinize the package and bump to 1.7.0 even though it is not tagged yet
* add link flag for OSX
* removed spurious reference to libroslib
* bondcpp now explicitly links against the ros library.  `#5334 <https://github.com/ros/bond_core/issues/5334>`_
* Changed ros::Time/Duration to ros::WallTime/WallDuration so Bond still works when time stops.  Fixes `#5035 <https://github.com/ros/bond_core/issues/5035>`_
* Fixed destruction bug: doesn't destroy things if the bond was never started.
* Can now set a bond's callback queue
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4037081
* Modified bond's state machine to handle "alive" messages from the sibling when already dead.
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4036189
* Added global "bond_disable_heartbeat_timeout" parameter
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4036106
* typo
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4035731
* rosdep and packages are not the same
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4035730
* patch from stevenbellens for fedora uuid support `#4756 <https://github.com/ros/bond_core/issues/4756>`_
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4035729
* Re-ordering locking in bondcpp's destructor
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4035719
* In bond, wait_until_formed and wait_until_broken terminate when ROS shuts down.
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4035632
* Bond no longer warns on destructor when the other side disappeared.
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4035573
* removed wiki syntax from description
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4035392
* Creating package descriptions for bondpy, bondcpp, and test_bond.
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4035354
* The bond state machine more gracefully handles excessive requests to die.
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4032653
* Moving bond into common
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4032634
* Contributors: Brian Gerkey, Stuart Glaser, Vincent Rabaud, kwc, sglaser, tfoote
