# CMake generated Testfile for 
# Source directory: /home/lab307/wk_scout/src/nmea_navsat_driver
# Build directory: /home/lab307/wk_scout/src/nmea_navsat_driver/cmake-build-debug
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_nmea_navsat_driver_roslint_package "/home/lab307/wk_scout/src/nmea_navsat_driver/cmake-build-debug/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/home/lab307/wk_scout/src/nmea_navsat_driver/cmake-build-debug/test_results/nmea_navsat_driver/roslint-nmea_navsat_driver.xml" "--working-dir" "/home/lab307/wk_scout/src/nmea_navsat_driver/cmake-build-debug" "--return-code" "/opt/ros/kinetic/share/roslint/cmake/../../../lib/roslint/test_wrapper /home/lab307/wk_scout/src/nmea_navsat_driver/cmake-build-debug/test_results/nmea_navsat_driver/roslint-nmea_navsat_driver.xml make roslint_nmea_navsat_driver")
set_tests_properties(_ctest_nmea_navsat_driver_roslint_package PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/kinetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/kinetic/share/roslint/cmake/roslint-extras.cmake;67;catkin_run_tests_target;/home/lab307/wk_scout/src/nmea_navsat_driver/CMakeLists.txt;40;roslint_add_test;/home/lab307/wk_scout/src/nmea_navsat_driver/CMakeLists.txt;0;")
subdirs("gtest")
