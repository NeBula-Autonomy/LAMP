#include <iostream>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <string>
#include <unordered_map>

#include <parameter_utils/ParameterUtils.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pose_graph_msgs/KeyedScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <lamp_utils/CommonFunctions.h>
#include <lamp_utils/PointCloudTypes.h>
#include <lamp_utils/PrefixHandling.h>
#include <visualization_msgs/Marker.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/dataset.h>

#include "KimeraRPGO/RobustSolver.h"

namespace pu = parameter_utils;
using KimeraRPGO::RobustSolver;
using KimeraRPGO::RobustSolverParams;
using pose_graph_msgs::KeyedScan;
using pose_graph_msgs::PoseGraph;

void inputKeyedScansBag(
    const std::string& filename,
    std::unordered_map<gtsam::Key, PointCloud>* keyed_scans) {
  ROS_INFO("Reading keyed scans from %s", filename.c_str());
  rosbag::Bag bag;
  bag.open(filename);
  for (rosbag::MessageInstance const m : rosbag::View(bag)) {
    KeyedScan::ConstPtr ks = m.instantiate<KeyedScan>();
    if (ks != nullptr) {
      PointCloud scan;
      pcl::fromROSMsg(ks->scan, scan);
      keyed_scans->insert({ks->key, scan});
    }
  }
  bag.close();
  ROS_INFO("Processed %d keyed scans from bag. ", keyed_scans->size());
}

void inputPoseGraphBag(const std::string& filename,
                       gtsam::NonlinearFactorGraph* nfg,
                       gtsam::Values* values) {
  ROS_INFO("Reading pose graph msgs from %s", filename.c_str());
  rosbag::Bag bag;
  bag.open(filename);
  for (rosbag::MessageInstance const m : rosbag::View(bag)) {
    PoseGraph::ConstPtr pg = m.instantiate<PoseGraph>();
    if (pg != nullptr) {
      for (const auto& node : pg->nodes) {
        gtsam::Key node_key = gtsam::Key(node.key);
        if (values->exists(node_key)) {
          continue;
        }
        gtsam::Pose3 node_pose = lamp_utils::MessageToPose(node);
        values->insert(node_key, node_pose);
      }
      for (const auto& edge : pg->edges) {
        if (edge.type == pose_graph_msgs::PoseGraphEdge::ODOM ||
            edge.type == pose_graph_msgs::PoseGraphEdge::LOOPCLOSE) {
          nfg->add(gtsam::BetweenFactor<gtsam::Pose3>(
              gtsam::Symbol(edge.key_from),
              gtsam::Symbol(edge.key_to),
              lamp_utils::MessageToPose(edge),
              lamp_utils::MessageToCovariance(edge)));
        }
        if (edge.type == pose_graph_msgs::PoseGraphEdge::PRIOR) {
          nfg->add(
              gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol(edge.key_from),
                                               lamp_utils::MessageToPose(edge),
                                               lamp_utils::MessageToCovariance(edge)));
        }
      }
    }
  }
  bag.close();
  ROS_INFO(
      "Processed %d factors and %d values from bag. ", nfg->size(), values->size());
}

void inputG2oFile(const std::string& filename,
                  gtsam::NonlinearFactorGraph* nfg,
                  gtsam::Values* values) {
  ROS_INFO("Reading factors and values from %s", filename.c_str());
  gtsam::GraphAndValues graph_and_values = gtsam::load3D(filename);
  *nfg = *graph_and_values.first;
  *values = *graph_and_values.second;
  ROS_INFO("Read in %d factors and %d values from g2o.",
           nfg->size(),
           values->size());
}

PointCloud transformCloudToWorld(const gtsam::Key& key,
                                 const gtsam::Values& values,
                                 const PointCloud& cloud) {
  PointCloud output_cloud;
  gtsam::Pose3 pose = values.at<gtsam::Pose3>(key);
  Eigen::Matrix4d b2w = pose.matrix();
  pcl::transformPointCloud(cloud, output_cloud, b2w);
  return output_cloud;
}

PointCloud
buildCloudMap(const gtsam::Values& values,
              const std::unordered_map<gtsam::Key, PointCloud>& keyed_scans,
              const double& grid_size) {
  PointCloud::Ptr map_cloud(new PointCloud);
  for (const auto& ks : keyed_scans) {
    if (values.exists(ks.first)) {
      PointCloud transformed_scan =
          transformCloudToWorld(ks.first, values, ks.second);
      *map_cloud += transformed_scan;
    }
  }
  pcl::VoxelGrid<Point> grid;
  grid.setLeafSize(grid_size, grid_size, grid_size);
  grid.setInputCloud(map_cloud);
  grid.filter(*map_cloud);

  return *map_cloud;
}

PointCloud buildCloudMapByRobot(
    const std::string robot_name,
    const gtsam::Values& values,
    const std::unordered_map<gtsam::Key, PointCloud>& keyed_scans,
    const double& grid_size) {
  const char prefix = lamp_utils::ROBOT_PREFIXES.at(robot_name);

  PointCloud::Ptr map_cloud(new PointCloud);
  for (const auto& ks : keyed_scans) {
    if (gtsam::Symbol(ks.first).chr() != prefix) {
      continue;
    }

    if (values.exists(ks.first)) {
      PointCloud transformed_scan =
          transformCloudToWorld(ks.first, values, ks.second);
      *map_cloud += transformed_scan;
    }
  }
  pcl::VoxelGrid<Point> grid;
  grid.setLeafSize(grid_size, grid_size, grid_size);
  grid.setInputCloud(map_cloud);
  grid.filter(*map_cloud);

  return *map_cloud;
}

bool isRobotKey(gtsam::Key key) {
  return lamp_utils::IsRobotPrefix(gtsam::Symbol(key).chr());
}

void publishOdomEdges(const gtsam::Values& values,
                      const gtsam::NonlinearFactorGraph& nfg,
                      ros::Publisher* publisher,
                      double marker_width) {
  // Extract odom edges and convert to marker msg
  visualization_msgs::Marker odom_edges;
  odom_edges.header.frame_id = "world";
  odom_edges.color.r = 1.0;
  odom_edges.color.b = 1.0;
  odom_edges.color.a = 1.0;
  odom_edges.scale.x = marker_width;
  odom_edges.pose.orientation.w = 1.0;
  odom_edges.type = visualization_msgs::Marker::LINE_LIST;
  for (const auto& f : nfg) {
    if (f->back() == f->front() + 1 && isRobotKey(f->front()) &&
        isRobotKey(f->back())) {
      if (values.exists(f->back()) && values.exists(f->front())) {
        odom_edges.points.push_back(lamp_utils::GtsamToRosMsg(
            values.at<gtsam::Pose3>(f->front()).translation()));
        odom_edges.points.push_back(lamp_utils::GtsamToRosMsg(
            values.at<gtsam::Pose3>(f->back()).translation()));
      }
    }
  }
  publisher->publish(odom_edges);
}

void publishInlierEdges(const gtsam::Values& values,
                        const gtsam::NonlinearFactorGraph& nfg,
                        const gtsam::Vector& gnc_weights,
                        ros::Publisher* publisher,
                        double marker_width) {
  // Extract inlier lc edges and convert to marker msg
  visualization_msgs::Marker inlier_edges;
  inlier_edges.header.frame_id = "world";
  inlier_edges.color.b = 1.0;
  inlier_edges.color.a = 1.0;
  inlier_edges.scale.x = marker_width;
  inlier_edges.pose.orientation.w = 1.0;
  inlier_edges.type = visualization_msgs::Marker::LINE_LIST;
  for (size_t i = 0; i < nfg.size(); i++) {
    const auto& f = nfg[i];
    if (f->back() != f->front() + 1 && isRobotKey(f->front()) &&
        isRobotKey(f->back())) {
      if (gnc_weights.size() == 0 || gnc_weights[i] > 0.5) {
        inlier_edges.points.push_back(lamp_utils::GtsamToRosMsg(
            values.at<gtsam::Pose3>(f->front()).translation()));
        inlier_edges.points.push_back(lamp_utils::GtsamToRosMsg(
            values.at<gtsam::Pose3>(f->back()).translation()));
      }
    }
  }
  publisher->publish(inlier_edges);
}

void publishOutlierEdges(const gtsam::Values& values,
                         const gtsam::NonlinearFactorGraph& nfg,
                         const gtsam::Vector& gnc_weights,
                         ros::Publisher* publisher,
                         double marker_width) {
  if (gnc_weights.size() == 0) {
    return;
  }

  // Extract outlier lc edges and convert to marker msg
  visualization_msgs::Marker outlier_edges;
  outlier_edges.header.frame_id = "world";
  outlier_edges.color.r = 0.5;
  outlier_edges.color.g = 0.5;
  outlier_edges.color.b = 0.5;
  outlier_edges.color.a = 0.5;
  outlier_edges.scale.x = marker_width * 0.5;
  outlier_edges.pose.orientation.w = 1.0;
  outlier_edges.type = visualization_msgs::Marker::LINE_LIST;
  for (size_t i = 0; i < nfg.size(); i++) {
    const auto& f = nfg[i];
    if (f->back() != f->front() + 1 && isRobotKey(f->front()) &&
        isRobotKey(f->back())) {
      if (gnc_weights[i] < 0.5) {
        outlier_edges.points.push_back(lamp_utils::GtsamToRosMsg(
            values.at<gtsam::Pose3>(f->front()).translation()));
        outlier_edges.points.push_back(lamp_utils::GtsamToRosMsg(
            values.at<gtsam::Pose3>(f->back()).translation()));
      }
    }
  }
  publisher->publish(outlier_edges);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lamp_from_g2o");

  std::vector<std::string> robots{"husky1",
                                  "husky2",
                                  "husky3",
                                  "husky4",
                                  "spot1",
                                  "spot2",
                                  "spot3",
                                  "spot4"};
  ros::NodeHandle n("~");

  //// Load parameters
  std::string param_ns = lamp_utils::GetParamNamespace(n.getNamespace());
  RobustSolverParams rpgo_params;
  bool b_use_outlier_rejection;
  if (!pu::Get(param_ns + "/b_use_outlier_rejection", b_use_outlier_rejection))
    return 1;
  if (b_use_outlier_rejection) {
    // outlier rejection on: set up PCM params
    double trans_threshold, rot_threshold, gnc_alpha;
    if (!pu::Get(param_ns + "/translation_check_threshold", trans_threshold))
      return 1;
    if (!pu::Get(param_ns + "/rotation_check_threshold", rot_threshold))
      return 1;
    if (!pu::Get(param_ns + "/gnc_alpha", gnc_alpha))
      return 1;
    bool b_gnc_bias_odom;
    if (!pu::Get(param_ns + "/b_gnc_bias_odom", b_gnc_bias_odom))
      return 1;
    rpgo_params.setPcmSimple3DParams(
        trans_threshold, rot_threshold, -1, -1, KimeraRPGO::Verbosity::UPDATE);
    if (gnc_alpha > 0 && gnc_alpha < 1) {
      rpgo_params.setGncInlierCostThresholdsAtProbability(gnc_alpha);
      if (b_gnc_bias_odom) rpgo_params.gncBiasOdom();
    }
  } else {
    rpgo_params.setNoRejection(
        KimeraRPGO::Verbosity::VERBOSE); // set no outlier rejection
  }

  // Artifact or UWB keys (l, m, n, ... + u)
  rpgo_params.specialSymbols = lamp_utils::GetAllSpecialSymbols();

  // set solver
  int solver_num;
  if (!pu::Get(param_ns + "/solver", solver_num))
    return 1;

  if (solver_num == 1) {
    // Levenberg-Marquardt
    rpgo_params.solver = KimeraRPGO::Solver::LM;
  } else if (solver_num == 2) {
    // Gauss Newton
    rpgo_params.solver = KimeraRPGO::Solver::GN;
  } else {
    ROS_ERROR("Unsupported solver parameter. Use 1 for LM and 2 for GN");
  }
  // Use incremental max clique
  rpgo_params.setIncremental();

  std::string output_dir;
  bool save_output = true;
  if (!pu::Get("output_dir", output_dir)) {
    save_output = false;
  } else {
    rpgo_params.logOutput(output_dir);
    ROS_INFO("Enabled logging in Kimera-RPGO");
  }

  bool load_from_g2o;
  if (!pu::Get("b_load_g2o", load_from_g2o)) {
    return 1;
  }

  std::string ks_file;
  if (!pu::Get("keyed_scans_bag", ks_file)) {
    ROS_ERROR("Keyed scans bag path not specified. ");
    return 1;
  }

  double map_grid_size;
  if (!pu::Get("map_resolution", map_grid_size)) {
    ROS_ERROR("Missing map resolution parameter. ");
    return 1;
  }

  double marker_size;
  if (!pu::Get("marker_size", marker_size)) {
    ROS_ERROR("Missing marker size parameter. ");
    return 1;
  }

  int batch_size;
  if (!pu::Get("batch_size", batch_size)) {
    ROS_ERROR("Missing batch size parameter. ");
    return 1;
  }

  bool visualize;
  pu::Get("visualize", visualize);

  //// Load g2o file
  gtsam::NonlinearFactorGraph nfg, filtered_nfg;
  gtsam::Values values;
  if (load_from_g2o) {
    std::string g2o_file;
    if (!pu::Get("g2o_file", g2o_file)) {
      ROS_ERROR("G2O file path not specified. ");
      return 1;
    }
    inputG2oFile(g2o_file, &nfg, &values);
  } else {
    std::string pose_graph_bag;
    if (!pu::Get("pose_graph_bag", pose_graph_bag)) {
      ROS_ERROR("Pose graph bag path not specified. ");
      return 1;
    }
    inputPoseGraphBag(pose_graph_bag, &nfg, &values);
  }

  double max_lc_error;
  if (!pu::Get(param_ns + "/max_lc_error", max_lc_error))
    return 1;

  // sort factors and filter out bad loop closures
  gtsam::NonlinearFactorGraph odom_factors, prior_factors, lc_factors;
  for (const auto& factor : nfg) {
    if (factor->front() + 1 == factor->back()) {
      odom_factors.add(factor);
      continue;
    }
    if (factor->front() == factor->back()) {
      prior_factors.add(factor);
      continue;
    }
    // if (factor->error(values) > max_lc_error) {
    //   continue;
    // }
    lc_factors.add(factor);
  }
  ROS_INFO("%d odom factors, %d prior factors, %d loop closure factors. ",
           odom_factors.size(),
           prior_factors.size(),
           lc_factors.size());

  //// Load keyed scans
  std::unordered_map<gtsam::Key, PointCloud> keyed_scans;
  inputKeyedScansBag(ks_file, &keyed_scans);

  //// Reinitialize with new values
  KimeraRPGO::RobustSolver pgo(rpgo_params);
  if (batch_size == -1) {
    // Process all at once
    //// Update with odom
    pgo.forceUpdate(odom_factors, values);
    values = pgo.calculateEstimate();
    //// Update with priors
    pgo.forceUpdate(prior_factors, gtsam::Values());
    values = pgo.calculateEstimate();
    // Update with loop closures
    pgo.forceUpdate(lc_factors, gtsam::Values());
  } else {
    pgo.forceUpdate(gtsam::NonlinearFactorGraph(), values);
    gtsam::NonlinearFactorGraph batch_nfg;
    for (const auto& factor : nfg) {
      if (batch_nfg.size() >= batch_size) {
        pgo.forceUpdate(batch_nfg, gtsam::Values());
        batch_nfg = gtsam::NonlinearFactorGraph();
      }
      batch_nfg.add(factor);
    }
  }
  

  nfg = pgo.getFactorsUnsafe();
  values = pgo.calculateEstimate();
  gtsam::Vector gnc_weights = pgo.getGncWeights();

  ROS_INFO("Building point cloud map from optimized values. ");
  PointCloud map_cloud = buildCloudMap(values, keyed_scans, map_grid_size);

  std::unordered_map<std::string, PointCloud> robot_maps;
  for (const auto& robot : robots) {
    robot_maps.insert(
        {robot,
         buildCloudMapByRobot(robot, values, keyed_scans, map_grid_size)});
  }

  if (save_output) {
    std::string pcd_file_name = output_dir + "/lamp_map.pcd";
    pcl::io::savePCDFileASCII(pcd_file_name, map_cloud);
    ROS_INFO("Saved map cloud to %s", pcd_file_name.c_str());
    for (const auto& robot : robots) {
      if (robot_maps[robot].size() == 0) {
        continue;
      }
      pcd_file_name = output_dir + "/" + robot + "_map.pcd";
      pcl::io::savePCDFileASCII(pcd_file_name, robot_maps[robot]);
      ROS_INFO("Saved map cloud to %s", pcd_file_name.c_str());
    }
  }

  if (visualize) {
    // Create publishers
    ros::Publisher map_pub = n.advertise<PointCloud>("map_cloud", 1, true);
    ros::Publisher odom_edge_pub =
        n.advertise<visualization_msgs::Marker>("odom_edges", 1, true);
    ros::Publisher inlier_edge_pub =
        n.advertise<visualization_msgs::Marker>("inlier_edges", 1, true);
    ros::Publisher outlier_edge_pub =
        n.advertise<visualization_msgs::Marker>("outlier_edges", 1, true);

    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(map_cloud, map_msg);
    map_msg.header.frame_id = "world";
    map_pub.publish(map_msg);

    std::vector<ros::Publisher> map_publishers;
    for (const auto& robot : robots) {
      if (robot_maps[robot].size() == 0) {
        continue;
      }
      std::string topic_name = robot + "_map";
      ros::Publisher robot_map_pub =
          n.advertise<PointCloud>(topic_name, 1, true);
      sensor_msgs::PointCloud2 robot_map_msg;
      pcl::toROSMsg(robot_maps[robot], robot_map_msg);
      robot_map_msg.header.frame_id = "world";
      robot_map_pub.publish(robot_map_msg);
      map_publishers.push_back(robot_map_pub);
    }

    publishOdomEdges(values, nfg, &odom_edge_pub, marker_size);
    publishInlierEdges(values, nfg, gnc_weights, &inlier_edge_pub, marker_size);
    publishOutlierEdges(
        values, nfg, gnc_weights, &outlier_edge_pub, marker_size);
    ros::spin();
  }
}