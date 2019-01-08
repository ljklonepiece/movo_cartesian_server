# MOVO Cartesian Server
Simple PD controller for movo arm to move in cartesian space

## installation
* git clone this ros package to your movo catkin workspace and build the workspace
* `movo_msgs` is a required dependency
* make sure the rotating frame of both arms are set to Fixed frame before using this package, follow this [discussion on movo github](https://github.com/Kinovarobotics/kinova-movo/issues/60)

## test cartesian server
the program will move the left arm and right arm by 5cm in x, y, z direction with respect to the `base_link`

`roslaunch cartesian_server cart_controller.launch`

`cd [your directory]/cartesian_server/test`

`python cartesian_motion_test.py`


