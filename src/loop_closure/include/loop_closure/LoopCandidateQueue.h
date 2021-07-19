/**
 * @file   LoopCandidateQueue.h
 * @brief  Consolidation queue for loop closure candidates before computing
 * transform
 * @author Yun Chang
 */
#pragma once

#include <deque>
#include <map>
#include <queue>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <tuple>
#include <string>
#include <sstream>

#include <pose_graph_msgs/LoopCandidate.h>
#include <pose_graph_msgs/LoopCandidateArray.h>
#include <pose_graph_msgs/LoopComputationStatus.h>
#include <ros/console.h>
#include <ros/ros.h>

namespace lamp_loop_closure {

class LoopCandidateQueue {
public:
  LoopCandidateQueue();
  ~LoopCandidateQueue();

  virtual bool Initialize(const ros::NodeHandle& n);

  virtual bool LoadParameters(const ros::NodeHandle& n);

  virtual bool CreatePublishers(const ros::NodeHandle& n);

  virtual bool RegisterCallbacks(const ros::NodeHandle& n);

  void PublishAllLoopClosures();
 protected:
  void InputCallback(
      const pose_graph_msgs::LoopCandidateArray::ConstPtr& input_candidates);

  void LoopComputationStatusCallback(const pose_graph_msgs::LoopComputationStatus::ConstPtr& status);

  virtual void OnNewLoopClosure();

  virtual void OnLoopComputationCompleted();



  void PublishLoopCandidate(
      const pose_graph_msgs::LoopCandidateArray& candidates, bool check_sent=true);

  std::string make_key(const pose_graph_msgs::LoopCandidate& loop_closure);
  virtual bool LoopClosureHasBeenSent(const pose_graph_msgs::LoopCandidate& loop_closure);

  virtual void AddLoopClosureToSent(const pose_graph_msgs::LoopCandidate& loop_closure);



  // Define publishers and subscribers
  ros::Publisher loop_candidate_pub_;
  ros::Subscriber loop_candidate_sub_;
  ros::Subscriber loop_closure_status_sub_;
  std::unordered_map<int, std::deque<pose_graph_msgs::LoopCandidate>> queues;

  //Keys are: key_from, key_to, type
  std::unordered_set<std::string> sent_loop_closures_;
  std::string param_ns_;
};

} // namespace lamp_loop_closure