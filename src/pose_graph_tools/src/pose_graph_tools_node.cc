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
  ros::Rate loop_rate(10);

  dynamic_reconfigure::Server<pose_graph_tools::Config> dsrv;    

  pose_graph_tools::PoseGraphToolsNode node(nh, dsrv);

  while (ros::ok()) 
  {
    node.mainNodeThread();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
