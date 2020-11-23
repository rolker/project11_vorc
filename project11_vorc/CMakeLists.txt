cmake_minimum_required(VERSION 2.8.3)
project(project11_vorc)

find_package(catkin REQUIRED COMPONENTS
    project11
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

