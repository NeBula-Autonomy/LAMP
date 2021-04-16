/**
 * @file   ObservabilityLoopCandidatePrioritization.h
 * @brief  Base class for classes to find "priority loop closures" from the
 * candidates
 * @author Yun Chang
 */

#include "loop_closure/ObservabilityLoopCandidatePrioritization.h"

namespace lamp_loop_closure {

ObservabilityLoopCandidatePrioritization::
    ObservabilityLoopCandidatePrioritization();
ObservabilityLoopCandidatePrioritization::
    ~ObservabilityLoopCandidatePrioritization();

bool ObservabilityLoopCandidatePrioritization::Initialize(
    const ros::NodeHandle& n) {}

bool ObservabilityLoopCandidatePrioritization::LoadParameters(
    const ros::NodeHandle& n) {}

bool ObservabilityLoopCandidatePrioritization::CreatePublishers(
    const ros::NodeHandle& n) {}

bool ObservabilityLoopCandidatePrioritization::RegisterCallbacks(
    const ros::NodeHandle& n) {}

bool ObservabilityLoopCandidatePrioritization::PopulatePriorityQueue() {}

void ObservabilityLoopCandidatePrioritization::PublishBestCandidates() {}

void ObservabilityLoopCandidatePrioritization::KeyedScanCallback(
    const pose_graph_msgs::KeyedScan::ConstPtr& scan_msg) {}

}  // namespace lamp_loop_closure