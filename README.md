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
git clone git@github.com:NeBula-Autonomy/common_nebula_slam.git
wstool init
wstool merge localizer_lamp/install/lamp_ssh.rosinstall
wstool up
catkin build lamp
```
The rosinstall file should take care of most of the dependencies such as [GTSAM](https://github.com/borglab/gtsam) and Eigen.
For the loop closure prioritization module, we also need to install some Python dependencies. This package uses python2.
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
python2 -m pip install -r requirements.txt
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

## Publications to cite when using this code

Original LAMP paper - 2020
```
@inproceedings{ebadi2020lamp,
  title={LAMP: Large-scale autonomous mapping and positioning for exploration of perceptually-degraded subterranean environments},
  author={Ebadi, Kamak and Chang, Yun and Palieri, Matteo and Stephens, Alex and Hatteland, Alex and Heiden, Eric and Thakur, Abhishek and Funabiki, Nobuhiro and Morrell, Benjamin and Wood, Sally and others},
  booktitle={2020 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={80--86},
  year={2020},
  organization={IEEE}
}
```

LAMP 2.0 paper - 2022 - Overall LAMP system
```
@article{chang2022lamp,
  title={LAMP 2.0: A robust multi-robot SLAM system for operation in challenging large-scale underground environments},
  author={Chang, Yun and Ebadi, Kamak and Denniston, Christopher E and Ginting, Muhammad Fadhil and Rosinol, Antoni and Reinke, Andrzej and Palieri, Matteo and Shi, Jingnan and Chatterjee, Arghya and Morrell, Benjamin and others},
  journal={IEEE Robotics and Automation Letters},
  volume={7},
  number={4},
  pages={9175--9182},
  year={2022},
  publisher={IEEE}
}
```

Loop Closure Prioritization
```
@article{denniston2022loop,
  title={Loop closure prioritization for efficient and scalable multi-robot SLAM},
  author={Denniston, Christopher E and Chang, Yun and Reinke, Andrzej and Ebadi, Kamak and Sukhatme, Gaurav S and Carlone, Luca and Morrell, Benjamin and Agha-mohammadi, Ali-akbar},
  journal={IEEE Robotics and Automation Letters},
  volume={7},
  number={4},
  pages={9651--9658},
  year={2022},
  publisher={IEEE}
}
```
