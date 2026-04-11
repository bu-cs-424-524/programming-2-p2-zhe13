cmake_minimum_required(VERSION 3.0.2)
project(p2_zhe13)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  tf
  cv_bridge
  image_transport
)

catkin_package()

catkin_install_python(PROGRAMS
  script/waypoint_nav.py
  script/ball_follower.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
