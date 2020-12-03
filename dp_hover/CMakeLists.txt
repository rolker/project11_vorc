cmake_minimum_required(VERSION 2.8.3)
project(dp_hover)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geographic_msgs
  marine_msgs
  roscpp
  genmsg
  actionlib_msgs
  actionlib
  project11
  geographic_visualization_msgs
  dynamic_reconfigure
  marine_msgs
)

generate_dynamic_reconfigure_options(
    cfg/dp_hover.cfg
)


add_action_files(DIRECTORY action FILES dp_hover.action)
generate_messages(DEPENDENCIES actionlib_msgs geographic_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES asv_helm
  CATKIN_DEPENDS geographic_msgs geometry_msgs roscpp project11
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/dp_hover.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_node marine_msgs_gencpp)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

