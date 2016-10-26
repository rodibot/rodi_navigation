cmake_minimum_required(VERSION 2.8.3)
project(rodi_navigation)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS scripts/rodi_pose.py scripts/rodi_simple_navigation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch  
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
