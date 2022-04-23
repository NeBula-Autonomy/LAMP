![LAMP-logo](https://gitlab.robotics.caltech.edu/rollocopter/localizer/localizer_lamp/raw/master/LAMP-logo.png)


## Build Instructions

Install [ROS](http://wiki.ros.org/ROS/Installation)

Install catkin tools
```
sudo apt-get install ros-kinetic-catkin python-catkin-tools python3-catkin-tools
```

Install PCL 
```
sudo apt-get install ros-melodic-pcl-ros # for the melodic distro - Ubuntu 18.04
sudo apt-get install ros-noetic-pcl-ros # for the noetc distro - Ubuntu 20.04
```


Build this package in a catkin workspace 
```bash
mkdir -p catkin_ws/src
cd catkin_ws
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release -DGTSAM_TANGENT_PREINTEGRATION=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DOPENGV_BUILD_WITH_MARCH_NATIVE=OFF -DBUILD_TEASER_FPFH=ON
cd src
wstool init
wstool merge LAMP/install/lamp_ssh.rosinstall
wstool up
catkin build lamp
```
The rosinstall file should take care of most of the dependencies such as [GTSAM](https://github.com/borglab/gtsam) and Eigen.


For the loop closure prioritization module, we also need to install some Python dependencies.
```
pip install -r LAMP/install/requirements.txt
```

The dependencies are:
```
torch>=1.4.0
torch-scatter==1.4.0
torch-sparse==0.4.4
torch-cluster==1.4.5
torch-spline-conv==1.1.1
torch-geometric==1.3.2
torchvision
```



## Run Instructions
***LAMP*** TODO.
### Single-robot testing

### Multi-robot testing 
To run a multi-robot example with our released subterranean multi-robot [dataset](TODO), first download the dataset, 
then start the LAMP base-station process: 
```
roslaunch lamp turn_on_lamp_base.launch robot_namespace:=base1
```
then play the rosbag:
```
rosbag play <path-to-data>/*.bag -r1 --clock clock:=/clock --wait-for-subscribers
```
and to visualize the map, launch rviz:
```
rviz -d $(rospack find lamp)/rviz/lamp_base.rviz
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
