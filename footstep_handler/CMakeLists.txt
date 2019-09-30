cmake_minimum_required(VERSION 2.8.3)
project(footstep_handler)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    actionlib_msgs
    geometry_msgs
    move_base_msgs
    actionlib
    rospy
    message_generation
)

add_message_files(
    FILES
    time_msg.msg
)

catkin_python_setup()

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS
    move_base_msgs
    actionlib_msgs
    actionlib
)

set( nodes 
    src/footstep_handler/main
    src/footstep_handler/send_pose
)

catkin_install_python(PROGRAMS
    ${nodes}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
