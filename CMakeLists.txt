cmake_minimum_required(VERSION 3.0.2)
project(line_planner)

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    rospy
)

generate_dynamic_reconfigure_options(
    config/LinePlanner.cfg
)

catkin_package(
    CATKIN_DEPENDS dynamic_reconfigure rospy
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
    src/line_planner_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)