cmake_minimum_required(VERSION 3.0.2)
project(nturt_bag_recorder)

find_package(catkin REQUIRED
        rospy
)

catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        CATKIN_DEPENDS rospy
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
        scripts/nturt_bag_recorder.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
