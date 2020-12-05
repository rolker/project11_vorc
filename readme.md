Project 11 and VORC Quick Start

- install Project 11: https://github.com/CCOMJHC/project11
- create a local dir for extra packages: mkdir project11/catkin_ws/src/local
- Clone the following repos in project11/catkin_ws/src/local
  - https://github.com/rolker/project11_vorc
  - https://github.com/osrf/vrx
  - https://github.com/osrf/vorc
  - https://github.com/rolker/pointcloud2_spherical_filter
  - https://github.com/Tossy0423/yolov4-for-darknet_ros (recursively!)

roslaunch vorc_gazebo marina.launch

roslaunch project11_vorc cora_backseat.launch
