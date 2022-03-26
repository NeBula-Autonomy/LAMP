// load_and_save_as_g2o.cc
// Save some rosbag with pose_graph topic as g2o file
// Use case is for post analysis with kimera rpgo
// Author: Yun Chang

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/dataset.h>
#include <pose_graph_msgs/PoseGraph.h>
#include <pose_graph_msgs/PoseGraphEdge.h>
#include <pose_graph_msgs/PoseGraphNode.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "lamp_utils/CommonFunctions.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "load_and_save_as_g2o");

  // usage: rosrun load_and_save_as_g2o bag_path topic_name output_path

  if (argc != 4) {
    std::cout
        << "usage: rosrun load_and_save_as_g2o bag_path topic_name output_path"
        << std::endl;
  }

  rosbag::Bag bag;
  bag.open(argv[1], rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(argv[2]);
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  pose_graph_msgs::PoseGraph::Ptr last_posegraph(new pose_graph_msgs::PoseGraph());
  for (const rosbag::MessageInstance& msg : view) {
    const std::string& msg_topic = msg.getTopic();
    pose_graph_msgs::PoseGraph::Ptr pg =
        msg.instantiate<pose_graph_msgs::PoseGraph>();
    if (pg != NULL and msg_topic == argv[2]) *last_posegraph = *pg;
  }
  gtsam::NonlinearFactorGraph nfg;
  gtsam::Values values;
  lamp_utils::PoseGraphMsgToGtsam(last_posegraph, &nfg, &values);
  gtsam::writeG2o(nfg, values, argv[3]);
  std::cout << "wrote g2o to: " << argv[3] << std::endl;
}
