cmake_minimum_required(VERSION 2.8.3)
project(nmea_navsat_driver)



find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  roscpp
  rospy
  std_msgs)

catkin_python_setup()

catkin_package()

# add_executable(nmea_navsat_driver 
#   src/getPosition/getposition.cpp
# )
# target_link_libraries(nmea_navsat_driver
#   ${catkin_LIBRARIES}
# )

install(PROGRAMS
   scripts/nmea_serial_driver
   scripts/nmea_socket_driver
   scripts/nmea_topic_driver
   scripts/nmea_topic_serial_reader
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch"
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslint)
  roslint_python()
  roslint_add_test()
endif()
