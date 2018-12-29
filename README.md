# MOVO Cartesian Server
Simple PD controller for movo arm to move in cartesian space

## installation
* git clone this ros package to your movo catkin workspace and build the workspace
* `movo_msgs` is a required dependency

## test cartesian server
the program will move the left arm and right arm by 5cm in positive x direction with respect to the `base_link`

`roslaunch cartesian_server cart_controller.launch`

`cd [your directory]/cartesian_server/test`

`python cartesian_motion_test.py`


