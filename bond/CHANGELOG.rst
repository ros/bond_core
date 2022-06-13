^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bond
^^^^^^^^^^^^^^^^^^^^^^^^^^

3.0.2 (2022-04-01)
------------------

3.0.1 (2021-01-26)
------------------

3.0.0 (2021-01-26)
------------------

2.0.0 (2020-11-05)
------------------
* Ros2 devel (`#54 <https://github.com/ros/bond_core/issues/54>`_)
* Make Michael Carroll the maintainer (`#40 <https://github.com/ros/bond_core/issues/40>`_)
* Contributors: Karsten Knese, Mikael Arguedas

1.8.3 (2018-08-17)
------------------

1.8.2 (2018-04-27)
------------------

1.8.1 (2017-10-27)
------------------
* fix package.xml to comply with schema (`#30 <https://github.com/ros/bond_core/issues/30>`_)
* Contributors: Mikael Arguedas

1.8.0 (2017-07-27)
------------------
* switch to package format 2 (`#27 <https://github.com/ros/bond_core/issues/27>`_)
* Contributors: Mikael Arguedas

1.7.19 (2017-03-27)
-------------------

1.7.18 (2016-10-24)
-------------------

1.7.17 (2016-03-15)
-------------------
* update maintainer
* Contributors: Mikael Arguedas

1.7.16 (2014-10-30)
-------------------

1.7.15 (2014-10-28)
-------------------

1.7.14 (2014-05-08)
-------------------
* Export architecture_independent flag in package.xml `#4 <https://github.com/ros/bond_core/pull/4>`_
* Update maintainer field
* Contributors: Esteve Fernandez, Scott K Logan, Vincent Rabaud

1.7.13 (2013-08-21)
-------------------

1.7.12 (2013-06-06)
-------------------

1.7.11 (2013-03-13)
-------------------

1.7.10 (2013-01-13)
-------------------

1.7.9 (2012-12-27)
------------------
* modified dep type of catkin
* Contributors: Dirk Thomas

1.7.8 (2012-12-13)
------------------
* add missing downstream depend
* switched from langs to message\_* packages
* Contributors: Dirk Thomas

1.7.7 (2012-12-06)
------------------
* Updated url tags in package.xml's  `#1 <https://github.com/ros/bond_core/pull/1>`_
* Contributors: William Woodall

1.7.6 (2012-10-30)
------------------
* fix catkin function order
* Contributors: Dirk Thomas

1.7.5 (2012-10-27)
------------------
* clean up package.xml files
* Contributors: Dirk Thomas

1.7.4 (2012-10-06)
------------------

1.7.3 (2012-10-02 00:19)
------------------------

1.7.2 (2012-10-02 00:06)
------------------------
* add the missing catkin dependency
* Contributors: Vincent Rabaud

1.7.1 (2012-10-01 19:00)
------------------------

1.7.0 (2012-10-01 16:51)
------------------------
* catkinize the package and bump to 1.7.0 even though it is not tagged yet
* Modified bond's state machine to handle "alive" messages from the sibling when already dead.
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4036189
* Added global "bond_disable_heartbeat_timeout" parameter
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4036106
* The bond state machine more gracefully handles excessive requests to die.
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4032653
* Moving bond into common
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4032634
* Contributors: Vincent Rabaud, sglaser
