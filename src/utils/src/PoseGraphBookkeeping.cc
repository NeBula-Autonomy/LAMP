#include "utils/CommonFunctions.h"
#include "utils/CommonStructs.h"

#include <gtsam/slam/PriorFactor.h>

void PoseGraph::TrackFactor(const EdgeMessage& msg) {

  auto edge_identifier = std::make_tuple(msg.key_from, msg.key_to, msg.type);

  // Only add the edge if it is the only one of its type between its two keys
  if (!tracked_edges_.count(edge_identifier)) {
    edges_.push_back(msg);
    edges_new_.push_back(msg);

    tracked_edges_.insert(edge_identifier);
  }

}

void PoseGraph::TrackFactor(const Factor& factor) {
  auto msg = factor.ToMsg();
  TrackFactor(msg);
}



void PoseGraph::TrackFactor(gtsam::Symbol key_from,
                            gtsam::Symbol key_to,
                            int type,
                            const gtsam::Pose3& transform,
                            const gtsam::SharedNoiseModel& covariance) {
  auto msg =
      utils::GtsamToRosMsg(key_from, key_to, type, transform, covariance);
  TrackFactor(msg);
}

void PoseGraph::TrackNode(const Node& node) {
  auto msg = node.ToMsg();
  priors_.push_back(msg);
  priors_new_.push_back(msg);
}

void PoseGraph::TrackNode(const NodeMessage& msg) {
  priors_.push_back(msg);
  priors_new_.push_back(msg);
}

void PoseGraph::TrackNode(const ros::Time& stamp,
                          gtsam::Symbol key,
                          const gtsam::Pose3& pose,
                          const gtsam::SharedNoiseModel& covariance) {
  NodeMessage msg =
      utils::GtsamToRosMsg(stamp, fixed_frame_id, key, pose, covariance);
  priors_.push_back(msg);
  priors_new_.push_back(msg);
}

void PoseGraph::AddNewValues(const gtsam::Values& new_values) {
  // Main values variable
  // for (auto v : new_values) {
  //   if (values.exists(v.key)) {
  //     ROS_WARN("Value already exists in values");
  //   }
  //   else {
  //     values.insert(v);
  //   }
  // }

  for (auto v : new_values) {
    if (!values.tryInsert(v.key, v.value).second) {
      values.update(v.key, v.value);
    }
    if (!values_new_.tryInsert(v.key, v.value).second) {
      values_new_.update(v.key, v.value);
    }    
  }
  
}

void PoseGraph::Initialize(gtsam::Symbol initial_key,
                           const gtsam::Pose3& pose,
                           const Diagonal::shared_ptr& covariance) {
  nfg = gtsam::NonlinearFactorGraph();
  values = gtsam::Values();
  nfg.add(gtsam::PriorFactor<gtsam::Pose3>(initial_key, pose, covariance));
  values.insert(initial_key, pose);
  values_new_ = values; // init this to track new values

  ros::Time stamp = ros::Time::now();
  keyed_stamps[initial_key] = stamp;

  TrackNode(stamp, initial_key, pose, covariance);
}

void PoseGraph::InsertKeyedScan(gtsam::Symbol key,
                                const PointCloud::ConstPtr& scan) {
  keyed_scans.insert(std::pair<gtsam::Symbol, PointCloud::ConstPtr>(key, scan));
}

void PoseGraph::InsertKeyedStamp(gtsam::Symbol key, const ros::Time& stamp) {
  keyed_stamps.insert(std::pair<gtsam::Symbol, ros::Time>(key, stamp));
}

void PoseGraph::InsertStampedOdomKey(double seconds, gtsam::Symbol key) {
  stamp_to_odom_key.insert(std::pair<double, gtsam::Symbol>(seconds, key));
}
