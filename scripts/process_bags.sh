#!/bin/sh

# usage: 
#   add "source ~/Desktop/process_bags.sh" to .bashrc (use actual script dir)
#   run "process_bags husky#" in rosbags directory

# download merge_bags.py from here and record the path
# https://www.clearpathrobotics.com/assets/downloads/support/merge_bag.py
MERGE_SCRIPT=~/Desktop/merge_bag.py

process_bags()
{
  if [ "$1" != "" ]
  then
    echo Reindexing bags...
    rosbag reindex *.bag.active
    mkdir reindexed
    mv *.bag.orig.active reindexed/

    echo Merging IMU bags...
    python $MERGE_SCRIPT $1__imu.bag $1_comm* -t "vn100/imu"
    echo Merging artifact bags...
    python $MERGE_SCRIPT $1__artifact.bag $1_art* -t "artifact_reconciliation/reconciled_artifact"
    echo Merging static tfs...
    python $MERGE_SCRIPT $1__tf_static.bag $1_comm* -t "/tf_static"
    
    mkdir unused
    mv $1_trav* unused/
    mv $1_vision* unused/
    mv $1_common* unused/
    mv $1_blam* unused/
    mv $1_artifact* unused/
    mv $1_cpu* unused/
else
    echo "Please specify robot name"
  fi
}
