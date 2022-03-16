![LAMP-logo](https://gitlab.robotics.caltech.edu/rollocopter/localizer/localizer_lamp/raw/master/LAMP-logo.png)


## Build Instructions
Build this package in a catkin workspace 
```bash
mkdir -p catkin_ws/src
cd catkin_ws
catkin init
catkin config -DCMAKE_BUILD_TYPE=Release -DGTSAM_TANGENT_PREINTEGRATION=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DOPENGV_BUILD_WITH_MARCH_NATIVE=OFF
```

```
wstool init
wstool merge localizer_lamp/install/lamp_ssh.rosinstall
wstool up
```

```
### Dependencies
This package requires the core_messages package to be built:
```bash
cd ~/catkin_ws/src
git clone https://gitlab.robotics.caltech.edu/rollocopter/core/core_messages.git
catkin build pose_graph_msgs
```

And the lo_frontend package:
```bash
cd ~/catkin_ws/src
git clone https://gitlab.robotics.caltech.edu/rollocopter/localizer/localizer_lo_frontend.git
catkin build lo_frontend
```

This package requires `minizip` to be available globally. It can be installed from the package manager via:
```bash
apt install libminizip-dev
```

***LAMP*** also relies on system installations of the following packages:

* [ROS](http://wiki.ros.org/ROS/Installation)
* [GTSAM](https://collab.cc.gatech.edu/borg/gtsam)

We recommend:
Clone GTSAM in your *home* folder and checkout the feature branch:   
```bash
cd
git clone https://bitbucket.org/gtborg/gtsam
```

Build
```bash
cd gtsam 
mkdir build
cd build
cmake .. -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON
$ optional: sudo make 
sudo make install
```

**Note:** 
The reason why we need EXPMAP is for the correct calculation of Jacobians. 
Enabling this and the `#define SLOW_BUT_CORRECT_BETWEENFACTOR` in LaserLoopCLosure.h are both important. Otherwise the default are some identity approximations for rotations, which works for some cases but fails for cases with manual loop closures, or artifacts. 


### Normal Building
When developing, the dependencies should be appropriately set up to simply build `lamp` to build all the required packages. 

```bash
cd ~/catkin_ws/src
catkin build lamp
```


**Note for PCL:**
The following significant changes were made to the build process:
* Projects using PCL are now including `${PCL_LIBRARIES}` in their respective `CMakeLists.txt`.
* All `CMakelists.text` are set to build in Release



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
In the standard case, lidar inout is expected provided to the `/$(ROBOT_NAME)/velodyne_points` topic using message type `sensor_msgs::PointCloud2`.



## (Optional) Running TBB and MKL:
Follow these steps for downloading MKL package:

Downloading the GnuPG key first and add it to the keyring:
```
cd /tmp
wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB
apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB
sh -c 'echo deb https://apt.repos.intel.com/mkl all main > /etc/apt/sources.list.d/intel-mkl.list'
```

After this step for avoiding any error update your repository once more.
```
apt-get update
```

And then:
```
apt-get install intel-mkl-64bit-2018.2-046
```
This is the 64-bit of the MKL.

**Note:**
MKL package at least requires 500MB packages. If you are running out of space, it is not required to risk it.



For the purpose of enabling the TBB package follow these commands:
```
sudo apt-get install libtbb-dev
```

and then

```
cd ~/ws/gtsam/cmake
```

Add these two commands to the CMakeLists.txt of gtsam and then rebuild your gtsam.
```
FindMKL.cmake
FindTBB.cmake 
```

**Note:** By applying both the packages, there are still crashes you will be seeing. It is provided by the developer that these two packages are still under the development.

**Note:** There are not consistancy in TBB package. %70 cases used the MKL and TBB and perfectly working with enhancement in lowering the computation. There are cases of software crashing.



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