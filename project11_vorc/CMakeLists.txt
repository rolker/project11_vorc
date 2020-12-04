cmake_minimum_required(VERSION 2.8.3)
project(project11_vorc)

find_package(catkin REQUIRED COMPONENTS
    project11
    rospy
    vrx_gazebo
    geographic_msgs
    marine_msgs
    robot_localization
)

catkin_python_setup()


catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS nodes/vorc_task_manager.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
