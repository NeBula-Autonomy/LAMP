![LAMP-logo](https://github.com/NeBula-Autonomy/LAMP/blob/main/LAMP-logo.png)


## Build Instructions

Install [ROS](http://wiki.ros.org/ROS/Installation)

Build this package in a catkin workspace 
```bash
mkdir -p catkin_ws/src
cd catkin_ws
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release -DGTSAM_TANGENT_PREINTEGRATION=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DOPENGV_BUILD_WITH_MARCH_NATIVE=OFF -DBUILD_TEASER_FPFH=ON
cd src
git clone git@github.com:NeBula-Autonomy/LAMP.git localizer_lamp
wstool init
wstool merge localizer_lamp/install/lamp_ssh.rosinstall
wstool up
catkin build lamp
```
The rosinstall file should take care of most of the dependencies such as [GTSAM](https://github.com/borglab/gtsam) and Eigen.
For the loop closure prioritization module, we also need to install some Python dependencies.
```
torch>=1.4.0
torch-scatter==1.4.0
torch-sparse==0.4.4
torch-cluster==1.4.5
torch-spline-conv==1.1.1
torch-geometric==1.3.2
torchvision
```
Alternatively, run

```
pip install -r requirements.txt
```

Note that we are using the develop branch of GTSAM, which is constantly being updated. 
The last tested commit of GTSAM is `99c01c4dba6443d923a28b9617b12fee06394688` for your reference. 

## Run Instructions

### Multi-robot testing 
To run a multi-robot example with our released subterranean multi-robot [dataset](https://github.com/NeBula-Autonomy/nebula-multirobot-dataset), first download the dataset, 
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