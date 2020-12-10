cmake_minimum_required(VERSION 2.8.3)
project(project11_vorc)

find_package(catkin REQUIRED COMPONENTS
    project11
    rospy
    vrx_gazebo
    geographic_msgs
    marine_msgs
    robot_localization
    xacro
)

catkin_python_setup()


catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# Run xacro to generate world files
xacro_add_files(
  worlds/marina.world.xacro
  worlds/perception.world.xacro
  worlds/station_keeping.world.xacro
  INORDER INSTALL DESTINATION worlds
)

install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)


catkin_install_python(PROGRAMS nodes/vorc_task_manager.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
