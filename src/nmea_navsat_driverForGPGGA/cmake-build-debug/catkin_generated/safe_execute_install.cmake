execute_process(COMMAND "/home/lab307/wk_scout/src/nmea_navsat_driver/cmake-build-debug/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/lab307/wk_scout/src/nmea_navsat_driver/cmake-build-debug/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
