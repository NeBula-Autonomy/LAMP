#!/bin/sh

: '
DOCUMENTATION: 
    - Script to avoid duplicate messages on vn100/imu_wori_wcov topic 

USAGE: 
    - Add "source ~/Desktop/filter_imu_bags.sh" to .bashrc
    - Run "filter_imu_bags husky#" in rosbag folder

TODO:  
    - Add support for multiple state_tcp_no_delay bags in rosfolder
'

filter_imu_bags()
{ 
  if [ "$1" != "" ]
    then
        echo 'Filtering imu bags retaining only vn100/imu topic'
        rosbag filter $1_state_tcp_no_delay_*.bag $1_imu.bag "topic=='/$1/vn100/imu'"
        mkdir imu_unused 
        mv $1_state_tcp_no_delay_*.bag imu_unused/
  else
        echo "Please specify robot name"
  fi
}