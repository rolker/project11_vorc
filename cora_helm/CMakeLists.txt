cmake_minimum_required(VERSION 2.8.3)
project(cora_helm)

find_package(catkin REQUIRED COMPONENTS
  geographic_msgs
  geometry_msgs
  rospy
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
    cfg/cora_helm.cfg
)


catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS nodes/cora_helm_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
