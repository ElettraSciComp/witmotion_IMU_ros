^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package witmotion_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* add MultiThreadedExecutor in witmotion_ros_node

1.3.1 (2023-08-08)
--------------------
* Fixed GPS decoder function in the underlying library (added x10 multiplier in Degrees section)
* Version bump to 1.3.1
* Community-driven test of the library on WTGAHRS1 combined enclosed IMU/GPS sensor module `<https://www.wit-motion.com/inertial-navigation/witmotion-wtgahrs1-10-axis-gps.html>` by Quing Joe Wong `joewong00`
* Contributors: Andrey Vukolov, Quing Joe Wong

1.3.0 (2023-06-14)
--------------------
* Added support for serial port timeout behaviour (`timeout_ms`)
* Updated README.md
* Merged pull request `#27 <https://github.com/ElettraSciComp/witmotion_IMU_ros/pull/27>` from `gsokoll`
* Version bump to 1.3.0
* Matched versions for ROS node and underlying library after merging pull request `#7 <https://github.com/ElettraSciComp/witmotion_IMU_QT/pull/7>`

1.2.28 (2023-03-21)
--------------------
* Updated online documentation
* Updated CMake build scenario to suppress useless Doxygen warning
* Version bump to 1.2.28
* Matched versions for ROS node and underlying library

1.2.27 (2023-02-27)
--------------------
* Merged pull request `#12 <https://github.com/ElettraSciComp/witmotion_IMU_ros/issues/12>`_ from fllay/main
  Migration to ROS2 made by @fllay approved. The information about the existence of the ROS2 branch will be added to README.md
* Added ROS2 branch information to README
* ros2 code
  Contributors: Andrei Vukolov, Andrey Vukolov, fllay
* Update .gitmodules
  Updated URLs to proper HTTPS
* Rename the project to witmotion_ros - cancelled
* fixed segmentation fault when Ctrl-C
* fixed polling interval and threading
* Fix link error
* package.xml version bump

0.11.18 (2022-08-26)
--------------------
* README fix
* Added RTC pre-synchronization
  - Fixed RTC callback crash
  - Implemented ISO8601 date conversion for pre-synchronization
  - Added presync parameter
* Version bump to 0.11.18
  - Fixed RTC unitialized STL pointer
  - Fixed Qt libraries linking
  - Fixed RTC publishing
  - TODO: Implementation of RTC precalibration
* Contributors: Andrey Vukolov

0.8.22 (2022-08-22)
-------------------
* Added dynamic versioning
  - Added automatic package.xml version generator
  - Added dependency on underlying library's 'version' file
  - Added automatic compatible ROS version string parser
  (GNU -> ROS)
* Library version bumped to 0.8.22-alpha
* Added convenient defaults for WT901 sensor on 200 Hz
* Library version bump to 0.7.15-alpha
* Version bump to 0.6.14-alpha
* Defaults for WT31N
  - Added default configuration file for WT31N sensor
  - Added example launch file for WT31N sensor
  - Bumped library version
* Added native quaternion support
  - Added 'use_native_orientation' parameter to IMU message
  publisher for forced waiting for native quaternion if
  the sensor provides it
  - Added native quaternion parser to IMU processing routine
  - Library version bump: the controller application for
  WT901 series sensors is now available
  - Tested on WT901 and JY901B sensors
* IPFS Datasheet directory fix
* Convenient defaults
  - Convenient defaults for spatial covariances (obtained from
  tests on spatially fixed WT901 sensor)
  - Convenient defaults for magnetometer linear calibration
  (conversion from 10x uTesla to Tesla)
  - Convenient defaults for magnetometer covariance
* Qt namespace workaround
  - Library with fully implemented Qt namespace workaround
  - Closes `#9 <https://github.com/ElettraSciComp/witmotion_IMU_ros/issues/9>`_
  - Prevalidation of WT901 controller application
* Snapped to library version 0.3.1-alpha
* Fixed IPFS hash
* Fixed logical error in IMU publisher
* Update README.md
  Closes `#6 <https://github.com/ElettraSciComp/witmotion_IMU_ros/issues/6>`_
* Submodule update
* README update for RTC
* Added RTC publisher support
  - Refers to `#3 <https://github.com/ElettraSciComp/witmotion_IMU_ros/issues/3>`_ opened by @rlabs-oss
  - Implements ROS timestamp publishing based on realtime clock data
  from Witmotion sensor device
* Readme fix, closes `#2 <https://github.com/ElettraSciComp/witmotion_IMU_ros/issues/2>`_ after adding installation guile
* Added installation guidance, minor fixes
* Merge pull request `#5 <https://github.com/ElettraSciComp/witmotion_IMU_ros/issues/5>`_ from agtbaskara/fix-missing-std-srvs
  Merged closing `#4 <https://github.com/ElettraSciComp/witmotion_IMU_ros/issues/4>`_
* Add std_srvs dependency
* README to ROS Wiki link
* Merge pull request `#1 <https://github.com/ElettraSciComp/witmotion_IMU_ros/issues/1>`_ from zacharykratochvil/main
  Merge: fixed linking to tf_geometry_msgs package from @zacharykratochvil
* Merge branch 'main' into main
* Added untested GPS decoder code
  - Added GPS decoder code with covariance dependency
  - Added GPS accuracy message decoder
  - Added GPS ground speed decoder
* fixed linking to tf_geometry_msgs package
* Updated README for last topic addition
* Added Orientation processing and publishing
* Renaming
* Rebase
* LICENSE added
* Initial documentation before migration
* First alpha release
* Contributors: Andrei Vukolov, Andrey Vukolov, agtbaskara, zacharykratochvil
