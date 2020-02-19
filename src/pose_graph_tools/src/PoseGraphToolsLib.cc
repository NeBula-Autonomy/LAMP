#include "pose_graph_tools/PoseGraphToolsLib.h"

#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

namespace pose_graph_tools {

PoseGraphToolsLib::PoseGraphToolsLib(void) {}

PoseGraphToolsLib::~PoseGraphToolsLib(void) {}

std::string PoseGraphToolsLib::bashColor(const color& c) {
  std::stringstream text;
  switch (c) {
    case red:
      text << "\033[1;31m";
      break;
    case green:
      text << "\033[1;32m";
      break;
    case yellow:
      text << "\033[1;33m";
      break;
    case blue:
      text << "\033[1;34m";
      break;
    case magenta:
      text << "\033[1;35m";
      break;
    case cyan:
      text << "\033[1;36m";
      break;
    case white:
      text << "\033[1;37m";
      break;
    default:
      text << "\033[1;37m";
  }
  return text.str();
}

std::string PoseGraphToolsLib::restartWhite(void) {
  std::stringstream text;
  text << "\033[1;37m\033[0m";
  return text.str();
}

std::string PoseGraphToolsLib::print(const std::string& _name,
                                     const std::string& _txt,
                                     const color& _color) {
  std::stringstream text;
  text << print(_name, _color) << " \e[1m" << _txt << "\e[21m"
       << restartWhite();
  ROS_INFO("%s", text.str().c_str());
  return text.str();
}

std::string PoseGraphToolsLib::print(const std::string& msg,
                                     const color& c = white) {
  std::stringstream text;
  text << bashColor(c) << msg << "\033[0m" << restartWhite();
  return text.str();
}
std::string PoseGraphToolsLib::print(const float& msg, const color& c = white) {
  std::stringstream text;
  text << msg << restartWhite();
  return text.str();
}

pose_graph_msgs::PoseGraph PoseGraphToolsLib::updateNodePosition(
    const pose_graph_msgs::PoseGraph& graph,
    uint64_t key,
    HTransf delta_pose) {
  // Reset the merger
  merger_ = Merger();
  pose_graph_msgs::PoseGraphPtr new_pose_graph =
      pose_graph_msgs::PoseGraphPtr(new pose_graph_msgs::PoseGraph());
  *new_pose_graph = graph;

  // Keep track of the constant part of the pose graph
  pose_graph_msgs::PoseGraphPtr const_pose_graph =
      pose_graph_msgs::PoseGraphPtr(new pose_graph_msgs::PoseGraph());

  // Change the edge: first calculate new edge transform between the node
  // we want to update and the previous node
  // Search for the edge
  for (size_t i = 0; i < graph.edges.size(); i++) {
    if (graph.edges[i].key_to == key) {
      // Get new transform between the two nodes
      HTransf original_tf;
      tf::poseMsgToEigen(graph.edges[i].pose, original_tf);
      geometry_msgs::Pose new_tf;
      tf::poseEigenToMsg(original_tf * delta_pose, new_tf);
      new_pose_graph->edges[i].pose = new_tf;
      break;
    } else {
      const_pose_graph->edges.push_back(graph.edges[i]);
    }
  }
  // Change the node: modify the node
  for (size_t i = 0; i < graph.nodes.size(); i++) {
    if (graph.nodes[i].key == key) {
      HTransf original_pose;
      tf::poseMsgToEigen(graph.nodes[i].pose, original_pose);
      geometry_msgs::Pose new_pose;
      tf::poseEigenToMsg(original_pose * delta_pose, new_pose);
      new_pose_graph->nodes[i].pose = new_pose;
      break;
    } else {
      const_pose_graph->nodes.push_back(graph.nodes[i]);
    }
  }
  merger_.OnSlowGraphMsg(const_pose_graph);
  merger_.OnFastGraphMsg(new_pose_graph);
  pose_graph_msgs::PoseGraph corrected_graph = merger_.GetCurrentGraph();
  merger_ = Merger(); // Again reset merger 
  corrected_graph.header = graph.header;
  return corrected_graph;
}

pose_graph_msgs::PoseGraph PoseGraphToolsLib::addNewIncomingGraph(
    const pose_graph_msgs::PoseGraphConstPtr& new_graph,
    const pose_graph_msgs::PoseGraphConstPtr& old_graph) {
  merger_.OnSlowGraphMsg(old_graph);
  merger_.OnFastGraphMsg(new_graph);
  pose_graph_msgs::PoseGraph processed_graph = merger_.GetCurrentGraph();
  processed_graph.header = new_graph->header;
  return processed_graph;
}

}  // namespace pose_graph_tools