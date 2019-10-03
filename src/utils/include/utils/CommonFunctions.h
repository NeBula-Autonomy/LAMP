/*
 * Copyright Notes
 *
 * Authors:
 * Yun Chang       (yunchang@mit.edu)
 */

#ifndef COMMON_FUNCTIONS_H
#define COMMON_FUNCTIONS_H

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <pose_graph_msgs/PoseGraph.h>

namespace utils {
// Pose graph msg to gtsam conversion
void PoseGraphMsgToGtsam(const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg,
                         gtsam::NonlinearFactorGraph* graph_nfg,
                         gtsam::Values* graph_vals);
}  // namespace utils
#endif  // COMMON_FUNCTIONS_H
