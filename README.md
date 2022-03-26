![LAMP-logo](https://gitlab.robotics.caltech.edu/rollocopter/localizer/localizer_lamp/raw/master/LAMP-logo.png)


## Build Instructions

Install [ROS](http://wiki.ros.org/ROS/Installation)

Build this package in a catkin workspace 
```bash
mkdir -p catkin_ws/src
cd catkin_ws
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release -DGTSAM_TANGENT_PREINTEGRATION=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DOPENGV_BUILD_WITH_MARCH_NATIVE=OFF -DBUILD_TEASER_FPFH=ON
cd src
wstool init
wstool merge localizer_lamp/install/lamp_ssh.rosinstall
wstool up
catkin build lamp
```
The rosinstall file should take care of most of the dependencies such as [GTSAM](https://github.com/borglab/gtsam) and Eigen.

## Run Instructions
***LAMP*** is written in C++ with some Python interface elements, wrapped by the Robot Operating System ([ROS](http://ros.org)). LAMP by itself is a backend that is flexible to a range of odometry inputs. For running with Lidar, the standard setup is outlined in the `src/lamp/scripts/launch_lamp.yaml` file, which is a [tmuxp]() load script. Run this with:

```bash
tmuxp load $(rospack find lamp)/scripts/launch_lamp.yaml
```

Note that this includes static transform publishers and bagfile playing (edit to point to the appropriate bagfile). 

This will open a tmux terminal with all processes running (optionally start a roscore beforehand).

Otherwise, run the following nodes in different terminals, replacing `husky` with the appropriate robot name (affects transforms and topic names).

```bash
roslaunch lo_frontend lo_frontend.launch robot_namespace:=$ROBOT_NAME
```
This launches the lidar odometry front-end

```bash
roslaunch lamp turn_on_lamp.launch robot_namespace:=$ROBOT_NAME
```
This starts the ***LAMP*** robot version

For visualization on rviz, launch:
```bash
roslaunch lamp turn_on_robot_viz.launch robot_namespace:=$ROBOT_NAME
```

For running the base station (needed for visualization), launch
```bash
roslaunch lamp turn_on_lamp_base.launch
```

### Multi-robot testing 
WIP: See script coming here:
```bash
tmuxp load $(rospack find lamp)/scripts/launch_lamp.yaml
```

### Data Inputs


## Unit tests
To compile and run unit tests:
```bash
roscore & catkin build run_tests
``` 

To view the results of a package:
```bash
catkin_test_results build/<package_name>
``` 
Results for unit tests of packages are stored in the build/<package_name>/test_results folder.

## Developer notes
The reason why we need EXPMAP is for the correct calculation of Jacobians. 
Enabling this and the `#define SLOW_BUT_CORRECT_BETWEENFACTOR` in LaserLoopCLosure.h are both important. Otherwise the default are some identity approximations for rotations, which works for some cases but fails for cases with manual loop closures, or artifacts. 