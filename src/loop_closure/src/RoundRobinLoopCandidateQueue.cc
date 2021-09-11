//
// Created by chris on 6/2/21.
//
#include "loop_closure/RoundRobinLoopCandidateQueue.h"
#include <parameter_utils/ParameterUtils.h>
namespace pu = parameter_utils;
namespace lamp_loop_closure {

RoundRobinLoopCandidateQueue::RoundRobinLoopCandidateQueue() : LoopCandidateQueue() {}
RoundRobinLoopCandidateQueue::~RoundRobinLoopCandidateQueue() {}

bool RoundRobinLoopCandidateQueue::Initialize(const ros::NodeHandle &n) {
  if(!LoopCandidateQueue::Initialize(n)) { return false;}

  key_ = -1;
  return true;
}
bool RoundRobinLoopCandidateQueue::LoadParameters(const ros::NodeHandle &n) {
  if(!LoopCandidateQueue::LoadParameters(n)) {return false;}

  if (!pu::Get(param_ns_ + "/queue/amount_per_round",
               amount_per_round_)) {return false;}
  return true;
}

void RoundRobinLoopCandidateQueue::FindNextSet() {
  //ROS_INFO("Finding next set");
  pose_graph_msgs::LoopCandidateArray out_array;
  int num_found = 0;
  int attempts = 0;
  while (attempts < amount_per_round_) {
    //Loop through queues until we find next one
    pose_graph_msgs::LoopCandidate next_candidate;
    bool found = false;
    //Find the first one
    if (key_ == -1) {
      for (auto& cur_queue : queues) {
        if (found) {
          key_ = cur_queue.first;
          break;
        }
        if (!cur_queue.second.empty() && !found ) {
          found = true;
          next_candidate = cur_queue.second.back();
          cur_queue.second.pop_back();
        }
      }
    }
    else {
      for (auto& cur_queue : queues) {
        if (found) {
          key_ = cur_queue.first;
          break;
        }
        if (key_ == cur_queue.first && !cur_queue.second.empty() && !found ) {
          found = true;
          next_candidate = cur_queue.second.back();
          cur_queue.second.pop_back();
        }
      }
      //Loop back around
      if (!found) {
        key_ = -1;
      }
    }
    if (found){
      out_array.candidates.push_back(next_candidate);
      num_found++;
    }
    attempts++;
  }
  static int info_count = 0;
  if (info_count % 10 == 0) {
    for (auto cur_queue : queues) {
      ROS_INFO_STREAM("Queue " << cur_queue.first << " has "
                               << cur_queue.second.size() << " elements.");
    }
  }
  info_count++;
  if (out_array.candidates.size() > 0)
    LoopCandidateQueue::PublishLoopCandidate(out_array);
}

void RoundRobinLoopCandidateQueue::OnNewLoopClosure() {
}

void RoundRobinLoopCandidateQueue::OnLoopComputationCompleted() {
  FindNextSet();
}

}
