/*
 * Copyright Notes
 *
 * Authors: Angel Santamaria Navarro    (angel.santamaria.navarro@jpl.nasa.gov)
 */

#include "pose_graph_tools/PoseGraphTools.h"

/* main function */
int main(int argc,char *argv[])
{
  ros::init(argc, argv, "pose_graph_tools");
  ros::NodeHandle nh(ros::this_node::getName());
  ros::Rate loop_rate(3);

  pose_graph_tools::PoseGraphToolsNode node(nh);

  while (ros::ok()) 
  {
    node.mainNodeThread();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
