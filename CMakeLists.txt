cmake_minimum_required(VERSION 3.0.2)
project(line_planner)

find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    rospy
    move_base_msgs
    std_msgs
    message_generation
)

generate_dynamic_reconfigure_options(
    config/LinePlanner.cfg
    config/AreaToPath.cfg
)

catkin_package(
    CATKIN_DEPENDS dynamic_reconfigure rospy geometry_msgs move_base_msgs std_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
    src/line_planner_node.py
    src/area_to_path_node.py
    src/sensor_to_obstacle_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)