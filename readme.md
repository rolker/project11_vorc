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

If CUDA is not avaiable, add the following repo in project11/catkin_ws/src/local

https://github.com/rolker/opencv_dnn.git

It needs to be compiled with a recent version of OpenCV 4. That will conflict with melodic version of cv_bridge so dowload and compile vision_opencv against OpenCV4 as well. TO make it compile properly, we switched to the noetic branch, and set ANDROID to true in the CMakeLists file to skip all the python stuff.
