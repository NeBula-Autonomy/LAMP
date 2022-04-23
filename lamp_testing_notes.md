# Testing on Ubuntu 20.04 Dell Precision 


## Install notes

### Install ROS
http://wiki.ros.org/noetic/Installation/Ubuntu
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop
sudo apt install ros-melodic-desktop
```

Source ros to bashrc
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

Ros build tools 
```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```


### Clone 

```
mkdir -p lamp_ws/src/
cd lamp_ws/src/
git clone git@github.com:NeBula-Autonomy/LAMP.git
```


## Install error 


### Issue with Python installs
$ pip install -r install/requirements.txt
Collecting torch>=1.4.0 (from -r install/requirements.txt (line 1))
  Using cached https://files.pythonhosted.org/packages/46/ca/306bb933a68b888ab1c20ede0342506b85857635f04fb55a56e53065579b/torch-1.4.0-cp27-cp27mu-manylinux1_x86_64.whl
Collecting torch-scatter==1.4.0 (from -r install/requirements.txt (line 2))
  Using cached https://files.pythonhosted.org/packages/b8/c3/8bad887ffa55c86f120ef5ae252dc0e357b3bd956d9fbf45242bacc46290/torch_scatter-1.4.0.tar.gz
    Complete output from command python setup.py egg_info:
    Traceback (most recent call last):
      File "<string>", line 1, in <module>
      File "/tmp/pip-build-2RYz4u/torch-scatter/setup.py", line 3, in <module>
        import torch
    ImportError: No module named torch
    
    ----------------------------------------
Command "python setup.py egg_info" failed with error code 1 in /tmp/pip-build-2RYz4u/torch-scatter/


#### Fix by installing PCL
```
_______________________________________________________________________________________________________________________________________________________________________________________________________________
Errors     << teaserpp:cmake /home/costar/lamp_ws/logs/teaserpp/build.cmake.000.log                                                                                                                            
CMake Error at /home/costar/lamp_ws/src/teaser_plusplus/CMakeLists.txt:59 (find_package):
  By not providing "FindPCL.cmake" in CMAKE_MODULE_PATH this project has
  asked CMake to find a package configuration file provided by "PCL", but
  CMake did not find one.

                                                                                                                                                                                                                                                                        Could not find a package configuration file provided by "PCL" (requested
                                                                                                                                                                                                                                                                        version 1.8) with any of the following names:

    PCLConfig.cmake
    pcl-config.cmake

  Add the installation prefix of "PCL" to CMAKE_PREFIX_PATH or set "PCL_DIR"
  to a directory containing one of the above files.  If "PCL" provides a
  separate development package or SDK, be sure it has been installed.


cd /home/costar/lamp_ws/build/teaserpp; catkin build --get-env teaserpp | catkin env -si  /usr/bin/cmake /home/costar/lamp_ws/src/teaser_plusplus --no-warn-unused-cli -DCMAKE_INSTALL_PREFIX=/home/costar/lamp_ws/devel -DCMAKE_BUILD_TYPE=Release -DGTSAM_TANGENT_PREINTEGRATION=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DOPENGV_BUILD_WITH_MARCH_NATIVE=OFF -DBUILD_TEASER_FPFH=ON; cd -


```