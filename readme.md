Project 11 and VORC Quick Start

- install Project 11: https://github.com/CCOMJHC/project11
- create a local dir for extra packages: mkdir project11/catkin_ws/src/local
- Clone the following repos in project11/catkin_ws/src/local
  - https://github.com/rolker/project11_vorc
  - https://github.com/osrf/vrx
  - https://github.com/osrf/vorc
  
roslaunch vorc_gazebo marina.launch
roslaunch project11_vorc cora_backseat.launch
