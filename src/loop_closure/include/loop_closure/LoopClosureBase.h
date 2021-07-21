/*
LoopClosureBase.h
Author: Yun Chang
Loop closure base class
*/

#ifndef LOOP_CLOSURE_BASE_H_
#define LOOP_CLOSURE_BASE_H_

#include <map>
#include <unordered_map>

#include <pose_graph_msgs/PoseGraph.h>
#include <pose_graph_msgs/PoseGraphEdge.h>
#include <pose_graph_msgs/PoseGraphNode.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include <geometry_utils/Transform3.h>
#include <utils/PrefixHandling.h>

class LoopClosure {
public:
  LoopClosure(const ros::NodeHandle& n);
  ~LoopClosure();

  virtual bool Initialize(const ros::NodeHandle& n) = 0;

protected:
  std::unordered_map<gtsam::Key, ros::Time> keyed_stamps_;
  std::unordered_map<gtsam::Key, gtsam::Pose3> keyed_poses_;

  // define publishers and subscribers
  ros::Publisher loop_closure_pub_;

  ros::Subscriber keyed_stamps_sub_;

  virtual bool FindLoopClosures(
      gtsam::Key new_key,
      std::vector<pose_graph_msgs::PoseGraphEdge>* loop_closure_edges) = 0;

  void InputCallback(const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg);

  void PublishLoopClosures(
      const std::vector<pose_graph_msgs::PoseGraphEdge> edges) const;


  // Parameter namespace ("robot" or "base")
  std::string param_ns_;

  // Whether or not to look for loop closures (set independently in each base
  // class)
  bool b_check_for_loop_closures_;
  // last_closure_key_<a,b> stores the last key for robot a on which there was a
  // loop closure between robots a and b
  std::map<std::pair<char, char>, gtsam::Key> last_closure_key_;

  pose_graph_msgs::PoseGraphEdge CreateLoopClosureEdge(
      const gtsam::Symbol& key1,
      const gtsam::Symbol& key2,
      const geometry_utils::Transform3& delta,
      const gtsam::Matrix66& covariance);

  pose_graph_msgs::PoseGraphEdge CreatePriorEdge(
      const gtsam::Symbol& key,
      const geometry_utils::Transform3& delta,
      const gtsam::Matrix66& covariance);
};
#endif // LOOP_CLOSURE_BASE_H_