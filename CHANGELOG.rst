^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.0 (2018-06-27)
------------------
* Update docs for bouncy leveraging remapping for demo (`#53 <https://github.com/ros2/sros2/issues/53>`_)
* Windows tutorial tweaks (`#58 <https://github.com/ros2/sros2/issues/58>`_)
* publish ans subscribe to all parameter service topics (`#52 <https://github.com/ros2/sros2/issues/52>`_)
* remove partitions (`#45 <https://github.com/ros2/sros2/issues/45>`_)
* as of Bouncy access control is available for both Fast-RTPS and Connext (`#50 <https://github.com/ros2/sros2/issues/50>`_)
* add pytest markers to linter tests
* Remove outdated docker resources now that SROS2 ships as part of the core (`#48 <https://github.com/ros2/sros2/issues/48>`_)
* add X509 extensionCA:false (`#47 <https://github.com/ros2/sros2/issues/47>`_)
* enable_liveliness_protection (`#44 <https://github.com/ros2/sros2/issues/44>`_)
* set zip_safe to avoid warning during installation (`#42 <https://github.com/ros2/sros2/issues/42>`_)
* Linter fixup
* add special service rule only if not wildcarding everything (`#40 <https://github.com/ros2/sros2/issues/40>`_)
* remove whant now appears to be obsolete DCPS whitelisting (`#34 <https://github.com/ros2/sros2/issues/34>`_)
* fix sample_policy download command (`#37 <https://github.com/ros2/sros2/issues/37>`_)
* Fix access control for ardent (`#33 <https://github.com/ros2/sros2/issues/33>`_)
* advise to ask questions on ROS answers
* print full help when no command is passed (`#35 <https://github.com/ros2/sros2/issues/35>`_)
* add return code to all verb apis (`#28 <https://github.com/ros2/sros2/issues/28>`_)
* Contributors: Dirk Thomas, Mikael Arguedas, Shane Loretz, dhood

0.4.0 (2017-12-08)
------------------
* update maintainer
* update instructions now that connext security is supported on all pla… (`#30 <https://github.com/ros2/sros2/issues/30>`_)
* explicitly call out setting the variables (`#29 <https://github.com/ros2/sros2/issues/29>`_)
* remove test_suite, add pytest as test_requires (`#27 <https://github.com/ros2/sros2/issues/27>`_)
* update xml to match spec + connext 53 (`#16 <https://github.com/ros2/sros2/issues/16>`_)
* 0.0.3
* install/setup.bat -> <path to ros2 install>/setup.bat (`#25 <https://github.com/ros2/sros2/issues/25>`_)
* Add internal topics (without partition) to default allow rule (`#24 <https://github.com/ros2/sros2/issues/24>`_)
  The topic wildcard + partition wildcard doesn't match
* Correct ordering of string formatting params (`#23 <https://github.com/ros2/sros2/issues/23>`_)
  Topic and partition were swapped
* make policy filenames match
* Update OpenSSL install instructions for Windows
* Fix connext for node with default partition (`#20 <https://github.com/ros2/sros2/issues/20>`_)
* update style to satisfy new flake8 plugins (`#19 <https://github.com/ros2/sros2/issues/19>`_)
* implicitly inherit from object (`#18 <https://github.com/ros2/sros2/issues/18>`_)
* remove flake8 dependency from Dockerfile (`#17 <https://github.com/ros2/sros2/issues/17>`_)
* add issue template
* update dockerfile to use beta2 binaries (`#14 <https://github.com/ros2/sros2/issues/14>`_)
* add libssl-dev as an exec dependency (`#13 <https://github.com/ros2/sros2/issues/13>`_)
* OS X: fix typo for env. variable (`#15 <https://github.com/ros2/sros2/issues/15>`_)
* update links to master
* Split docs and update content (`#12 <https://github.com/ros2/sros2/issues/12>`_)
* use ros2 run (`#11 <https://github.com/ros2/sros2/issues/11>`_)
* 0.0.2
* Find openssl executable on osx and enforce minimum required version for all platforms (`#10 <https://github.com/ros2/sros2/issues/10>`_)
* Updates to Windows running instructions (`#9 <https://github.com/ros2/sros2/issues/9>`_)
  - Fixed the OpenSSL install link
  - Added OpenSSL to path
  - Updated talker and listener calls
  - Deleted source section
* fix fallback api without argcomplete (`#8 <https://github.com/ros2/sros2/issues/8>`_)
* fix wrong imports (`#7 <https://github.com/ros2/sros2/issues/7>`_)
* Add tools for security files generation (`#3 <https://github.com/ros2/sros2/issues/3>`_)
* Initial commit
* Contributors: Adam Allevato, Dirk Thomas, Mikael Arguedas, Morgan Quigley, Shane Loretz, Tully Foote, Víctor Mayoral Vilches, dhood
