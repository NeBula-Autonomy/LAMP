#!/bin/sh

: '
DOCUMENTATION: 
    - Script to avoid duplicate messages on vn100/imu_wori_wcov topic 

USAGE: 
    - Add "source ~/Desktop/filter_imu_bags.sh" to .bashrc
    - Run "filter_imu_bags husky#" in rosbag folder
'

filter_imu_bags()
{ 
  if [ "$1" != "" ]
    then
        echo "Merging state_tcp_no_delay bags with vn100/imu topic"
        python $MERGE_SCRIPT $1_imu.bag $1_state_tcp_no_delay_* -t "/$1/vn100/imu"
        mkdir imu_unused 
        mv $1_state_tcp_no_delay_*.bag imu_unused/
  else
        echo "Please specify robot name"
  fi
}