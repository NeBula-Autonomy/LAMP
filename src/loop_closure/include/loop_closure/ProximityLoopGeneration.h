/**
 * @file   ProximityLoopGeneration.h
 * @brief  Find potentital loop closures based on proximity
 * @author Yun Chang
 */
#pragma once

#include <gtsam/inference/Symbol.h>

#include "loop_closure/LoopGeneration.h"

namespace lamp_loop_closure {

class ProximityLoopGeneration : public LoopGeneration {
public:
  ProximityLoopGeneration();
  ~ProximityLoopGeneration();

  bool Initialize(const ros::NodeHandle& n) override;

  bool LoadParameters(const ros::NodeHandle& n) override;

  bool CreatePublishers(const ros::NodeHandle& n) override;

  bool RegisterCallbacks(const ros::NodeHandle& n) override;

protected:
  void GenerateLoops(const gtsam::Key& new_key);

  void KeyedPoseCallback(
      const pose_graph_msgs::PoseGraph::ConstPtr& graph_msg) override;

  double DistanceBetweenKeys(const gtsam::Symbol& key1,
                             const gtsam::Symbol& key2) const;

  double proximity_threshold_;
  size_t skip_recent_poses_;
};

} // namespace lamp_loop_closure