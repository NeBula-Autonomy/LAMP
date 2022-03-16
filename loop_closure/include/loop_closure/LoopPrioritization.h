/**
 * @file   LoopPrioritization.h
 * @brief  Base class for classes to find "priority loop closures" from the
 * candidates
 * @author Yun Chang
 */
#pragma once

#include <deque>
#include <map>
#include <queue>
#include <vector>
#include <mutex>

#include <pose_graph_msgs/LoopCandidate.h>
#include <pose_graph_msgs/LoopCandidateArray.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>

namespace lamp_loop_closure {

class LoopPrioritization {
public:
  LoopPrioritization();
  ~LoopPrioritization();

  virtual bool Initialize(const ros::NodeHandle& n) = 0;

  virtual bool LoadParameters(const ros::NodeHandle& n);

  virtual bool CreatePublishers(const ros::NodeHandle& n);

  virtual bool RegisterCallbacks(const ros::NodeHandle& n);

  virtual std::vector<ros::AsyncSpinner> SetAsyncSpinners(const ros::NodeHandle& n);

protected:
  // Use different priority metrics to populate output (priority) queue
  virtual void PopulatePriorityQueue() = 0;

  virtual void PublishBestCandidates() = 0;

  virtual pose_graph_msgs::LoopCandidateArray GetBestCandidates() = 0;

  void InputCallback(
      const pose_graph_msgs::LoopCandidateArray::ConstPtr& input_candidates);



  void ProcessPopulateCallback(const ros::TimerEvent& ev);

  ros::CallbackQueue prioritize_queue_;
  ros::Timer populate_timer_;

  // Define publishers and subscribers
  ros::Publisher loop_candidate_pub_;
  ros::Subscriber loop_candidate_sub_;

  // Loop closure candidates priority queue (high to low)
  std::deque<pose_graph_msgs::LoopCandidate> priority_queue_;
  // Loop closure queue as received from candidate generation
  std::queue<pose_graph_msgs::LoopCandidate> candidate_queue_;

  std::string param_ns_;

  double keyed_scans_max_delay_;


  std::mutex priority_queue_mutex_;
};

} // namespace lamp_loop_closure