cmake_minimum_required(VERSION 3.0.2)
project(nturt_bag_recorder)

find_package(catkin REQUIRED
        rospy
        std_msgs
)

catkin_package(

)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
        scripts/nturt_bag_recorder.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
