/*
 * Copyright (c) 2016, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Author: Erik Nelson            ( eanelson@eecs.berkeley.edu )
 */

#include <laser_loop_closure/LaserLoopClosure.h>

#include <geometry_utils/GeometryUtilsROS.h>
#include <parameter_utils/ParameterUtils.h>
#include <pose_graph_msgs/KeyedScan.h>
#include <pose_graph_msgs/PoseGraph.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>

#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <gtsam/slam/dataset.h>

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <fstream>

#include <minizip/zip.h>
#include <minizip/unzip.h>

#include <time.h>

namespace gu = geometry_utils;
namespace gr = gu::ros;
namespace pu = parameter_utils;

using gtsam::BetweenFactor;
using gtsam::RangeFactor;
using gtsam::ISAM2; // TODO - remove these
using gtsam::ISAM2Params;
using gtsam::NonlinearFactorGraph;
using gtsam::Pose3;
using gtsam::PriorFactor;
using gtsam::Rot3;
using gtsam::Values;
using gtsam::GraphAndValues;
using gtsam::Vector3;
using gtsam::Vector6;
using gtsam::ISAM2GaussNewtonParams;

LaserLoopClosure::LaserLoopClosure()
    : key_(), last_closure_key_(std::numeric_limits<int>::min()), tf_listener_(tf_buffer_),
    b_first_key_set_(false) {
  initial_noise_.setZero();
}

LaserLoopClosure::~LaserLoopClosure() {}

bool LaserLoopClosure::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "LaserLoopClosure");

  if (!filter_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize point cloud filter.", name_.c_str());
    return false;
  }

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  return true;
}

bool LaserLoopClosure::LoadParameters(const ros::NodeHandle& n) {

  // Load frame ids.
  if (!pu::Get("frame_id/fixed", fixed_frame_id_)) return false;
  if (!pu::Get("frame_id/base", base_frame_id_)) return false;

  // Should we turn loop closure checking on or off?
  if (!pu::Get("check_for_loop_closures", check_for_loop_closures_)) return false;

  // Should we save a backup posegraph?
  if (!pu::Get("save_posegraph_backup", save_posegraph_backup_)) return false;

  // Should we save a backup posegraph?
  if (!pu::Get("keys_between_each_posegraph_backup", keys_between_each_posegraph_backup_)) return false;

  // check if lamp is run as basestation
  b_is_basestation_ = false;
  if (!pu::Get("b_is_basestation", b_is_basestation_)) return false;

  // Load Optimization parameters.
  relinearize_skip_ = 1;
  relinearize_threshold_ = 0.01;
  if (!pu::Get("relinearize_skip", relinearize_skip_)) return false;
  if (!pu::Get("relinearize_threshold", relinearize_threshold_)) return false;
  if (!pu::Get("n_iterations_manual_loop_close", n_iterations_manual_loop_close_)) return false;

  // Load loop closing parameters.
  if (!pu::Get("translation_threshold_kf", translation_threshold_kf_))
    return false;
  if (!pu::Get("translation_threshold_nodes", translation_threshold_nodes_))
    return false;
  if (!pu::Get("rotation_threshold_kf", rotation_threshold_kf_))
    return false;
  if (!pu::Get("rotation_threshold_nodes", rotation_threshold_nodes_))
    return false;
  if (!pu::Get("proximity_threshold", proximity_threshold_)) return false;
  if (!pu::Get("max_tolerable_fitness", max_tolerable_fitness_)) return false;
  if (!pu::Get("distance_to_skip_recent_poses", distance_to_skip_recent_poses_)) return false;
  if (!pu::Get("distance_before_reclosing", distance_before_reclosing_)) return false;
  if (!pu::Get("distance_before_batch_reclosing",
               distance_before_batch_reclosing_))
    return false;

  // Compute Skip recent poses
  skip_recent_poses_ = (int)(distance_to_skip_recent_poses_/translation_threshold_nodes_);
  poses_before_reclosing_ = (int)(distance_before_reclosing_/translation_threshold_nodes_);

  if (!pu::Get("manual_lc_rot_precision", manual_lc_rot_precision_)) return false;
  if (!pu::Get("manual_lc_trans_precision", manual_lc_trans_precision_)) return false;
  if (!pu::Get("laser_lc_rot_sigma", laser_lc_rot_sigma_)) return false;
  if (!pu::Get("laser_lc_trans_sigma", laser_lc_trans_sigma_)) return false;
  if (!pu::Get("artifact_rot_precision", artifact_rot_precision_)) return false; 
  if (!pu::Get("artifact_trans_precision", artifact_trans_precision_)) return false; 

  if(!pu::Get("fiducial_trans_precision", fiducial_trans_precision_)) return false;
  if(!pu::Get("fiducial_rot_precision", fiducial_rot_precision_)) return false;

  // Load ICP parameters.
  if (!pu::Get("icp/tf_epsilon", icp_tf_epsilon_)) return false;
  if (!pu::Get("icp/corr_dist", icp_corr_dist_)) return false;
  if (!pu::Get("icp/iterations", icp_iterations_)) return false;

  // Load initial position and orientation.
  double init_x = 0.0, init_y = 0.0, init_z = 0.0;
  double init_qx = 0.0, init_qy = 0.0, init_qz = 0.0, init_qw = 1.0;
  bool b_have_fiducial = true;
  if (!pu::Get("fiducial_calibration/position/x", init_x))
    b_have_fiducial = false;
  if (!pu::Get("fiducial_calibration/position/y", init_y))
    b_have_fiducial = false;
  if (!pu::Get("fiducial_calibration/position/z", init_z))
    b_have_fiducial = false;
  if (!pu::Get("fiducial_calibration/orientation/x", init_qx))
    b_have_fiducial = false;
  if (!pu::Get("fiducial_calibration/orientation/y", init_qy))
    b_have_fiducial = false;
  if (!pu::Get("fiducial_calibration/orientation/z", init_qz))
    b_have_fiducial = false;
  if (!pu::Get("fiducial_calibration/orientation/w", init_qw))
    b_have_fiducial = false;

  if (!b_have_fiducial) {
    ROS_WARN("Can't find fiducials, using origin");
  }

  // Load initial position and orientation noise.
  double sigma_x = 0.0, sigma_y = 0.0, sigma_z = 0.0;
  double sigma_roll = 0.0, sigma_pitch = 0.0, sigma_yaw = 0.0;
  if (!pu::Get("init/position_sigma/x", sigma_x)) return false;
  if (!pu::Get("init/position_sigma/y", sigma_y)) return false;
  if (!pu::Get("init/position_sigma/z", sigma_z)) return false;
  if (!pu::Get("init/orientation_sigma/roll", sigma_roll)) return false;
  if (!pu::Get("init/orientation_sigma/pitch", sigma_pitch)) return false;
  if (!pu::Get("init/orientation_sigma/yaw", sigma_yaw)) return false;

  // convert initial quaternion to Roll/Pitch/Yaw
  double init_roll = 0.0, init_pitch = 0.0, init_yaw = 0.0;
  gu::Quat q(gu::Quat(init_qw, init_qx, init_qy, init_qz));
  gu::Rot3 m1;
  m1 = gu::QuatToR(q);
  init_roll = m1.Roll();
  init_pitch = m1.Pitch();
  init_yaw = m1.Yaw();

  // Sanity check parameters
  if (!pu::Get("b_check_deltas", b_check_deltas_)) return false;
  if (!pu::Get("translational_sanity_check_lc", translational_sanity_check_lc_)) return false;
  if (!pu::Get("translational_sanity_check_odom", translational_sanity_check_odom_)) return false;
  // UWB
  if (!pu::Get("uwb_range_measurement_error", uwb_range_measurement_error_)) return false;
  if (!pu::Get("uwb_adding_range_option", uwb_adding_range_option_)) return false;
  if (!pu::Get("uwb_factor_optimizer", uwb_factor_optimizer_)) return false;
  if (!pu::Get("uwb_number_added_rangefactor_first", uwb_number_added_rangefactor_first_)) return false;
  if (!pu::Get("uwb_number_added_rangefactor_not_first", uwb_number_added_rangefactor_not_first_)) return false;
  if (!pu::Get("uwb_minimum_range_threshold", uwb_minimum_range_threshold_)) return false;
  if (!pu::Get("uwb_maximum_range_threshold", uwb_maximum_range_threshold_)) return false;
  XmlRpc::XmlRpcValue uwb_list;
  if (!pu::Get("uwb_list", uwb_list)) return false;
  for (int i=0; i<uwb_list.size(); i++) {
    auto uwb_id = static_cast<std::string>(uwb_list[i]["id"]);
    uwb_id2keynumber_hash_[uwb_id] = static_cast<int>(uwb_list[i]["gtsam_key"]);
  }
  if (!pu::Get("b_use_display_uwb_data", b_use_display_uwb_data_)) return false;
  if (!pu::Get("b_use_uwb_outlier_rejection", b_use_uwb_outlier_rejection_)) return false;
  if (!pu::Get("b_uwb_add_betweenfactor", b_uwb_add_betweenfactor_)) return false;
  double uwb_dropped_x, uwb_dropped_y, uwb_dropped_z;
  if (!pu::Get("uwb_dropped_position/x", uwb_dropped_x)) return false;
  if (!pu::Get("uwb_dropped_position/y", uwb_dropped_y)) return false;
  if (!pu::Get("uwb_dropped_position/z", uwb_dropped_z)) return false;
  uwb_dropped_position_ << uwb_dropped_x, uwb_dropped_y, uwb_dropped_z;

    // Optimizer backend
  bool b_use_outlier_rejection;
  if (!pu::Get("b_use_outlier_rejection", b_use_outlier_rejection))
    return false;
  if (b_use_outlier_rejection) {
    // outlier rejection on: set up PCM params
    double trans_threshold, rot_threshold;
    if (!pu::Get("translation_check_threshold", trans_threshold))
      return false;
    if (!pu::Get("rotation_check_threshold", rot_threshold))
      return false;
    rpgo_params_.setPcmSimple3DParams(trans_threshold, rot_threshold, RobustPGO::Verbosity::VERBOSE);
  } else {
    rpgo_params_.setNoRejection(RobustPGO::Verbosity::VERBOSE); // set no outlier rejection
  }
  std::vector<char> special_symbs{'l', 'm', 'n', 'o', 'p', 'q', 'u'}; // for artifacts
  rpgo_params_.specialSymbols = special_symbs;
  // set solver 
  int solver_num;
  if (!pu::Get("solver", solver_num))
    return false; 
  if (solver_num == 1) {
    rpgo_params_.solver = RobustPGO::Solver::LM;
  } else if (solver_num == 2) {
    rpgo_params_.solver = RobustPGO::Solver::GN;
  } else {
    ROS_ERROR("Unsupported solver parameter. Use 1 for LM and 2 for GN");
  }

  // Initialize solver 
  pgo_solver_.reset(new RobustPGO::RobustSolver(rpgo_params_));
  pgo_solver_->print();

  // Set the initial position.
  Vector3 translation(init_x, init_y, init_z);
  Rot3 rotation(Rot3::RzRyRx(init_roll, init_pitch, init_yaw));
  Pose3 pose(rotation, translation);

  // Set the covariance on initial position.
  initial_noise_ << sigma_roll, sigma_pitch, sigma_yaw, sigma_x, sigma_y,
      sigma_z;

  LaserLoopClosure::Diagonal::shared_ptr covariance(
      LaserLoopClosure::Diagonal::Sigmas(initial_noise_));
  ROS_INFO_STREAM("covariance is");
  ROS_INFO_STREAM(initial_noise_);

  // Set the initial odometry.
  odometry_ = Pose3::identity();

  initial_key_ = 0;

  key_ = 0;

  // Initialize
  // Skip pgo_solver and prefix_robot init if base station
  if (b_is_basestation_) {
    ROS_INFO("LAMP run as base_station");
    return true;
  }

  //Get the robot prefix from launchfile to set initial key
  bool b_initialized_prefix_from_launchfile = true;
  std::string prefix;
  unsigned char prefix_converter[1];
  if (!pu::Get("robot_prefix", prefix)){
     b_initialized_prefix_from_launchfile = false;
     ROS_ERROR("Could not find node ID assosiated with robot_namespace");
  }
  
  if (b_initialized_prefix_from_launchfile){
    std::copy( prefix.begin(), prefix.end(), prefix_converter);
    initial_key_ = gtsam::Symbol(prefix_converter[0],0);
  }

  // Initialize key_
  key_ = initial_key_;

  NonlinearFactorGraph new_factor;
  Values new_value;
  new_factor.add(MakePriorFactor(pose, covariance));
  new_value.insert(key_, pose);

  pgo_solver_->update(new_factor, new_value);
  values_ = pgo_solver_->calculateEstimate();
  nfg_ = pgo_solver_->getFactorsUnsafe();
  key_ = key_ + 1;

  // Set the initial odometry.
  odometry_ = Pose3::identity();

   // Timestamp to keys initialization 
  ros::Time stamp = ros::Time::now();
  keyed_stamps_.insert(std::pair<gtsam::Symbol, ros::Time>(initial_key_, stamp));
  stamps_keyed_.insert(std::pair<double, gtsam::Symbol>(stamp.toSec(), initial_key_));


  return true;
}

bool LaserLoopClosure::RegisterCallbacks(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  laser_lc_toggle_sub_ = nl.subscribe("laser_lc_toggle", 1, &LaserLoopClosure::LaserLCToggle, this);


  scan1_pub_ = nl.advertise<PointCloud>("loop_closure_scan1", 10, false);
  scan2_pub_ = nl.advertise<PointCloud>("loop_closure_scan2", 10, false);

  pose_graph_pub_ =
      nl.advertise<pose_graph_msgs::PoseGraph>("pose_graph", 10, false);
  keyed_scan_pub_ =
      nl.advertise<pose_graph_msgs::KeyedScan>("keyed_scans", 10, false);
  erase_posegraph_pub_ =
      nl.advertise<std_msgs::Bool>("erase_posegraph", 10, false);
  remove_factor_viz_pub_ =
      nl.advertise<std_msgs::Bool>("remove_factor_viz", 10, false);

  loop_closure_notifier_pub_ = nl.advertise<pose_graph_msgs::PoseGraphEdge>(
      "loop_closure_edge", 10, false);

  artifact_pub_ = nl.advertise<core_msgs::Artifact>("artifact", 10);
      
  return true;
}

void LaserLoopClosure::LaserLCToggle(const std_msgs::Bool& msg){
  
  // Boolean to turn laser loop closures on and off
  if (msg.data){
    check_for_loop_closures_ = true;
    ROS_INFO("Laser loop closure on");
  }else{
    check_for_loop_closures_ = false;
    ROS_INFO("Laser loop closure off");
  }

}

bool LaserLoopClosure::AddFactorAtRestart(const gu::Transform3& delta, const LaserLoopClosure::Mat66& covariance){
  // Append the new odometry.
  Pose3 new_odometry = ToGtsam(delta);

  //add a new factor
  Pose3 last_pose = values_.at<Pose3>(key_-1);

  NonlinearFactorGraph new_factor;
  Values new_value;
  new_factor.add(MakeBetweenFactor(new_odometry, ToGtsam(covariance)));

  new_value.insert(key_, last_pose.compose(new_odometry));
  // TODO Compose covariances at the same time as odometry
  // Update
  try{
    pgo_solver_->update(new_factor, new_value);
  } catch (...){
    // redirect cout to file
    std::ofstream nfgFile;
    std::string home_folder(getenv("HOME"));
    nfgFile.open(home_folder + "/Desktop/factor_graph.txt");
    std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
    std::cout.rdbuf(nfgFile.rdbuf());

    // save entire factor graph to file and debug if loop closure is correct
    gtsam::NonlinearFactorGraph nfg = pgo_solver_->getFactorsUnsafe();
    nfg.print();
    nfgFile.close();

    std::cout.rdbuf(coutbuf); //reset to standard output again

    ROS_ERROR("Update ERROR in AddBetweenFactors");
    throw;
  }
  
  // Update class variables
  values_ = pgo_solver_->calculateEstimate();

  nfg_ = pgo_solver_->getFactorsUnsafe();

  // Notify PGV that the posegraph has changed
  has_changed_ = true;

  // Get ready with next key
  key_ = key_ + 1;

  return true;
}

bool LaserLoopClosure::AddFactorAtLoad(const gu::Transform3& delta, const LaserLoopClosure::Mat66& covariance){
  // Append the new odometry.
  Pose3 new_odometry = ToGtsam(delta);

  //add a new factor
  Pose3 first_pose = values_.at<Pose3>(first_loaded_key_);
  NonlinearFactorGraph new_factor;
  Values new_value;
  new_factor.add(MakeBetweenFactorAtLoad(new_odometry, ToGtsam(covariance)));
  new_value.insert(key_, first_pose.compose(new_odometry));
  // TODO Compose covariances at the same time as odometry
  // Update
  try{
    pgo_solver_->update(new_factor, new_value);
    has_changed_ = true;
  } catch (...){
    // redirect cout to file
    std::ofstream nfgFile;
    std::string home_folder(getenv("HOME"));
    nfgFile.open(home_folder + "/Desktop/factor_graph.txt");
    std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
    std::cout.rdbuf(nfgFile.rdbuf());

    // save entire factor graph to file and debug if loop closure is correct
    gtsam::NonlinearFactorGraph nfg = pgo_solver_->getFactorsUnsafe();
    nfg.print();
    nfgFile.close();

    std::cout.rdbuf(coutbuf); //reset to standard output again

    ROS_ERROR("Update ERROR in AddBetweenFactors");
    throw;
  }
  
  // Update class variables
  values_ = pgo_solver_->calculateEstimate();

  nfg_ = pgo_solver_->getFactorsUnsafe();

  // Notify PGV that the posegraph has changed
  has_changed_ = true;

  // Get ready with next key
  key_ = key_ + 1;

  return true;
}

bool LaserLoopClosure::AddBetweenFactor(
    const gu::Transform3& delta, const LaserLoopClosure::Mat66& covariance,
    const ros::Time& stamp, gtsam::Symbol* key) {
  if (key == NULL) {
    ROS_ERROR("%s: Output key is null.", name_.c_str());
    return false;
  }

  // Append the new odometry.
  Pose3 new_odometry = ToGtsam(delta);

  // Is the odometry translation large enough to add a new node to the graph
  odometry_ = odometry_.compose(new_odometry);
  odometry_kf_ = odometry_kf_.compose(new_odometry);

  if (std::isnan(acos(odometry_.rotation().toQuaternion().w()))){
    ROS_WARN("NAN rotational angle in odometry, reseting to unit quaternion");
    odometry_ = Pose3(Rot3::RzRyRx(0, 0, 0),odometry_.translation());
  }

  if (odometry_.translation().norm() < translation_threshold_nodes_ &&  2*acos(odometry_.rotation().toQuaternion().w()) < rotation_threshold_nodes_) {
    // No new pose - translation is not enough, nor is rotation to add a new node
    return false;
  }

  ROS_INFO_STREAM("New node\nTranslation is: " << odometry_.translation().norm() << " (lim: " << translation_threshold_nodes_ << ")\nRotation is: "
    << 2*acos(odometry_.rotation().toQuaternion().w())*180.0/3.1415 << " (lim: " << rotation_threshold_nodes_*180.0/3.1415 << ").");

  // Else - add a new factor

  NonlinearFactorGraph new_factor;
  Values new_value;
  new_factor.add(MakeBetweenFactor(odometry_, ToGtsam(covariance)));
  // TODO Compose covariances at the same time as odometry

  gtsam::Symbol previous_key = key_-1;
  ROS_INFO("Checking for key %c %d", previous_key.chr(), previous_key.key());
  Pose3 last_pose = values_.at<Pose3>(key_-1);
  new_value.insert(key_, last_pose.compose(odometry_));

  // Get edges for recreation on base-station
  Edge odometry_edge;
  odometry_edge = std::make_pair(key_-1, key_);
  edge_poses_[odometry_edge] = odometry_;

  // Get covariance for recreation on base-station
  covariance_betweenfactor_[odometry_edge] = covariance;

  // Compute cost before optimization
  NonlinearFactorGraph nfg_temp = pgo_solver_->getFactorsUnsafe();
  nfg_temp.add(new_factor);
  Values values_temp = pgo_solver_->getLinearizationPoint();
  values_temp.insert(key_, last_pose.compose(odometry_));
  double cost_old = nfg_temp.error(values_temp); // Assume values is up to date - no new values
  //ROS_INFO("Cost before optimization is: %f", cost_old);

  // Update
  try {
    pgo_solver_->update(new_factor, new_value);
    has_changed_ = true;
  } catch (...) {
    // redirect cout to file
    std::ofstream nfgFile;
    std::string home_folder(getenv("HOME"));
    nfgFile.open(home_folder + "/Desktop/factor_graph.txt");
    std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
    std::cout.rdbuf(nfgFile.rdbuf());

    // save entire factor graph to file and debug if loop closure is correct
    gtsam::NonlinearFactorGraph nfg = pgo_solver_->getFactorsUnsafe();
    nfg.print();
    nfgFile.close();

    std::cout.rdbuf(coutbuf); //reset to standard output again

    ROS_ERROR("Update ERROR in AddBetweenFactors");
    throw;
  }
  
  // Update class variables
  values_ = pgo_solver_->calculateEstimate();

  nfg_ = pgo_solver_->getFactorsUnsafe();

  // Get updated cost
  double cost = nfg_.error(values_);

  //ROS_INFO("Cost after optimization is: %f", cost);

  // Do sanity check on result
  bool b_accept_update;
  if (b_check_deltas_ && values_backup_.exists(key_-1)){ // Only check if the values_backup has been stored (a loop closure has occured)
    ROS_INFO("Sanity checking output");
    b_accept_update = SanityCheckForLoopClosure(translational_sanity_check_odom_, cost_old, cost);
    // TODO - remove vizualization keys if it is rejected 

    if (!b_accept_update){
      ROS_WARN("Returning false for add between factor - have reset, waiting for next pose update");
      // Erase the current posegraph to make space for the backup
      LaserLoopClosure::ErasePosegraph();  
      // Run the load function to retrieve the posegraph
      if (load_graph_from_zip_file_) {
        LaserLoopClosure::Load("posegraph_backup.zip");
      } else {
        // Use stored values in nfg_backup and values_backup (from prior to a loop closure)
          values_ = values_backup_;
          nfg_ = nfg_backup_;
      
        // Update
        try {
          pgo_solver_->update(nfg_, values_);
          has_changed_ = true;
        } catch (...) {
          ROS_ERROR("Update ERROR in AddBetweenFactors in sanity check rejection");
          throw;
        }
      }
      return false;
    }
    ROS_INFO("Sanity check passed");
  }

  // Adding poses to stored hash maps
  // Store this timestamp so that we can publish the pose graph later.
  keyed_stamps_.insert(std::pair<gtsam::Symbol, ros::Time>(key_, stamp));
  stamps_keyed_.insert(std::pair<double, gtsam::Symbol>(stamp.toSec(), key_));

  // Assign output and get ready to go again!
  *key = gtsam::Symbol (key_);
  key_ = key_ + 1;

  // Reset odometry to identity
  odometry_ = Pose3::identity();

  // Return true to store a key frame
  if (odometry_kf_.translation().norm() > translation_threshold_kf_ || 2*acos(odometry_kf_.rotation().toQuaternion().w()) > rotation_threshold_kf_) {
    // True for a new key frame
    // Reset odometry to identity
    odometry_kf_ = Pose3::identity();
    return true;
  }

  return false;
}

// Function to change key number for multiple robots
bool LaserLoopClosure::ChangeKeyNumber() {
  if (values_.exists(initial_key_)) {
    initial_key_ = initial_key_ + ((initial_key_.index()/10000)+1)*10000;
  }
  // Set key to initial key
  key_ = initial_key_;
  keyed_stamps_.insert(std::pair<unsigned int, ros::Time>(key_, ros::Time::now()));
  stamps_keyed_.insert(std::pair<double, unsigned int>(ros::Time::now().toSec(), key_));

  return true;
}

bool LaserLoopClosure::AddUwbFactor(const std::string uwb_id, UwbMeasurementInfo uwb_data) {

  // Check whether the input UWB ID exists in the pose graph or not
  gtsam::Key uwb_key;
  if (uwb_id2key_hash_.find(uwb_id) != uwb_id2key_hash_.end()) {
    uwb_key = uwb_id2key_hash_[uwb_id];
  }
  else {
    uwb_key = gtsam::Symbol('u', uwb_id2keynumber_hash_[uwb_id]);
    uwb_id2key_hash_[uwb_id] = uwb_key;
    uwb_key2id_hash_[uwb_key] = uwb_id;
  }

  // UWB data outlier rejection based on Hample outlier identifier
  if (b_use_uwb_outlier_rejection_) {
    UwbDataOutlierRejection(uwb_data);
  }

  // Sort the UWB-related data stored in the buffer 
  UwbRearrangedData sorted_data = RearrangeUwbData(uwb_data);

  // TODO: Range measurement error may depend on a distance between a transmitter and a receiver
  double sigmaR = uwb_range_measurement_error_;
  gtsam::noiseModel::Base::shared_ptr gaussian = gtsam::noiseModel::Isotropic::Sigma(1, sigmaR);
  gtsam::noiseModel::Base::shared_ptr tukey = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Tukey::Create(15), gaussian); //robust
  gtsam::noiseModel::Base::shared_ptr rangeNoise = tukey;

  NonlinearFactorGraph new_factor;
  gtsam::Values new_values;

  // Change the process according to whether the uwb anchor is observed for the first time or not
  if (!values_.exists(uwb_key)) {

    switch (uwb_adding_range_option_) {
      case 0 : 
      {      
        // Add RangeFactors between the nearest pose key and the UWB key
        int counter = 0;
        for (int itr = 0; itr < uwb_number_added_rangefactor_first_; itr++) {
          // Too short range data is ignored
          while(sorted_data.range_nearest_key[counter] < uwb_minimum_range_threshold_) {
            counter++;
            if (counter >= sorted_data.range_nearest_key.size()) {
              ROS_INFO("Not enough number of range measurement");
              // TODO
              return false;
            }
          }
          double range = sorted_data.range_nearest_key[counter];
          gtsam::Key pose_key = sorted_data.pose_key_list[counter];
          counter++;
          new_factor.add(gtsam::RangeFactor<Pose3, Pose3>(pose_key, uwb_key, range, rangeNoise));
          Edge edge_range = std::make_pair(pose_key, uwb_key);
          uwb_edges_range_.push_back(edge_range);
          edge_ranges_[edge_range] = range;
          error_rangefactor_[edge_range] = sigmaR;
          ROS_INFO_STREAM("LaserLoopClosure adds new UWB edge between... "
                          << gtsam::DefaultKeyFormatter(pose_key) << " and "
                          << gtsam::DefaultKeyFormatter(uwb_key));
        }

        // Add a UWB key
        // TODO
        Eigen::Vector3d robot_position = sorted_data.posekey2data[sorted_data.pose_key_list[0]].robot_position[0];
        gtsam::Pose3 pose_uwb = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(robot_position));
        new_values.insert(uwb_key, pose_uwb);

        // Add a PriorFactor for the UWB key
        gtsam::Vector6 prior_precisions;
        prior_precisions.head<3>().setConstant(10.0);
        prior_precisions.tail<3>().setConstant(0.0);
        static const gtsam::SharedNoiseModel& prior_noise = 
        gtsam::noiseModel::Diagonal::Precisions(prior_precisions);
        new_factor.add(gtsam::PriorFactor<gtsam::Pose3>(uwb_key, gtsam::Pose3(), prior_noise));
      }
        break;
      case 1 :
      {
        // Add RangeFactors between the nearest pose key and the UWB key
        int counter = 0;
        for (int itr = 0; itr < sorted_data.range_nearest_key.size(); itr++) {
          double range = sorted_data.range_nearest_key[counter];
          if (range > uwb_minimum_range_threshold_ && range < uwb_maximum_range_threshold_) {
            gtsam::Key pose_key = sorted_data.pose_key_list[counter];
            new_factor.add(gtsam::RangeFactor<Pose3, Pose3>(pose_key, uwb_key, range, rangeNoise));
            Edge edge_range = std::make_pair(pose_key, uwb_key);
            uwb_edges_range_.push_back(edge_range);
            edge_ranges_[edge_range] = range;
            error_rangefactor_[edge_range] = sigmaR;
            ROS_INFO_STREAM("LaserLoopClosure adds new UWB edge between... "
                            << gtsam::DefaultKeyFormatter(pose_key) << " and "
                            << gtsam::DefaultKeyFormatter(uwb_key));
          }
          counter++;
        }

        // Add a UWB key
        // TODO
        Eigen::Vector3d robot_position = sorted_data.posekey2data[sorted_data.pose_key_list[0]].robot_position[0];
        gtsam::Pose3 pose_uwb = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(robot_position));
        new_values.insert(uwb_key, pose_uwb);

        // Add a PriorFactor for the UWB key
        gtsam::Vector6 prior_precisions;
        prior_precisions.head<3>().setConstant(10.0);
        prior_precisions.tail<3>().setConstant(0.0);
        static const gtsam::SharedNoiseModel& prior_noise = 
        gtsam::noiseModel::Diagonal::Precisions(prior_precisions);
        new_factor.add(gtsam::PriorFactor<gtsam::Pose3>(uwb_key, gtsam::Pose3(), prior_noise));
      }
        break;
      default :
      {
        // Error
        ROS_INFO_STREAM("ERROR, wrong compensation selection");
        // TODO: handle the error
      }
    }
  }
  else {
    switch (uwb_adding_range_option_) {
      case 0 :
      {
        int counter = 0;
        for (int itr = 0; itr < uwb_number_added_rangefactor_not_first_; itr++) {
          while(sorted_data.range_nearest_key[counter] < uwb_minimum_range_threshold_) {
            counter++;
            if (counter >= sorted_data.range_nearest_key.size()) {
              ROS_INFO("Not enough number of range measurement");
              // TODO
              return false;
            }
          }
          double range = sorted_data.range_nearest_key[counter];
          gtsam::Key pose_key = sorted_data.pose_key_list[counter];
          counter++;
          new_factor.add(gtsam::RangeFactor<Pose3, Pose3>(pose_key, uwb_key, range, rangeNoise));
          Edge edge_range = std::make_pair(pose_key, uwb_key);
          uwb_edges_range_.push_back(edge_range);
          edge_ranges_[edge_range] = range;
          error_rangefactor_[edge_range] = sigmaR;
          ROS_INFO_STREAM("LaserLoopClosure adds new UWB edge between... "
                          << gtsam::DefaultKeyFormatter(pose_key) << " and "
                          << gtsam::DefaultKeyFormatter(uwb_key));
        }
      }
        break;
      case 1 :
      {
        int counter = 0;
        for (int itr = 0; itr < sorted_data.range_nearest_key.size(); itr++) {
          double range = sorted_data.range_nearest_key[counter];
          if (range > uwb_minimum_range_threshold_ && range < uwb_maximum_range_threshold_) {
            gtsam::Key pose_key = sorted_data.pose_key_list[counter];
            new_factor.add(gtsam::RangeFactor<Pose3, Pose3>(pose_key, uwb_key, range, rangeNoise));
            Edge edge_range = std::make_pair(pose_key, uwb_key);
            uwb_edges_range_.push_back(edge_range);
            edge_ranges_[edge_range] = range;
            error_rangefactor_[edge_range] = sigmaR;
            ROS_INFO_STREAM("LaserLoopClosure adds new UWB edge between... "
                            << gtsam::DefaultKeyFormatter(pose_key) << " and "
                            << gtsam::DefaultKeyFormatter(uwb_key));
          }
          counter++;
        }
      }
        break;
      default :
      {
        // Error
        ROS_INFO_STREAM("ERROR, wrong compensation selection");
        // TODO handle the error
      }
    }
  }

  return (UwbLoopClosureOptimization(new_factor, new_values));
}

void LaserLoopClosure::UwbDataOutlierRejection(UwbMeasurementInfo &uwb_data) {
  std::vector<bool> b_outlier_list;
  std::vector<bool> b_range_outlier_list;
  std::vector<bool> b_time_outlier_list;
  std::vector<double> time_stamp;
  for (auto itr : uwb_data.time_stamp) {
    time_stamp.push_back(itr.toSec());
  }
  std::vector<double> time_index;
  double counter = 0.0;
  for (auto itr : time_stamp) {
    time_index.push_back(counter);
    counter = counter + 1.0;
  }

  // Outlier rejection for range measurement
  hampelOutlierRejection<double, double>(uwb_data.range, time_stamp, b_range_outlier_list);
  // Outlier rejection for time stamps
  hampelOutlierRejection<double, double>(time_stamp, time_index, b_time_outlier_list);
  // Integrate the result of outlier rejection for range measurement and time stamps
  for (int itr = 0; itr < b_range_outlier_list.size(); itr++) {
    if (b_range_outlier_list[itr] == true || b_time_outlier_list[itr] == true) {
      b_outlier_list.push_back(true);
    }
    else {
      b_outlier_list.push_back(false);
    }
  }

  for (int itr = 0; itr < b_outlier_list.size(); itr++) {
    int itr_back = b_outlier_list.size()-itr-1;
    if (b_outlier_list[itr_back] == true) {
      // Show the outliers
      if (b_use_display_uwb_data_ == true) {
        std::cout << "outlier data No. " << itr_back << "\t";
        std::cout << "range: " << uwb_data.range[itr_back]  << "\t";
        std::cout << "time_stamp: " << uwb_data.time_stamp[itr_back] << "\n";
      }
      // Remove the outlier data from the buffer
      uwb_data.time_stamp.erase(uwb_data.time_stamp.begin()+itr_back);
      uwb_data.range.erase(uwb_data.range.begin()+itr_back);
      uwb_data.robot_position.erase(uwb_data.robot_position.begin()+itr_back);
      // uwb_data.dist_posekey.erase(uwb_data.dist_posekey.begin()+itr_back);
      uwb_data.nearest_pose_key.erase(uwb_data.nearest_pose_key.begin()+itr_back);
    }
  }
}

template <typename TYPE_DATA, typename TYPE_STAMP>
void LaserLoopClosure::hampelOutlierRejection(std::vector<TYPE_DATA> data_input, 
                            std::vector<TYPE_STAMP> data_stamp,
                            std::vector<bool>& b_outlier_list) {
    unsigned int data_size = data_input.size();

    std::vector<TYPE_STAMP> sorted_stamp = data_stamp;
    std::sort(sorted_stamp.begin(), sorted_stamp.end());

    TYPE_STAMP median_stamp;
    median_stamp = calculateMedian(sorted_stamp);

    std::vector<TYPE_STAMP> stamp_temp1 = sorted_stamp;
    std::vector<TYPE_STAMP> stamp_temp2 = sorted_stamp;
    std::vector<TYPE_STAMP> diff_stamp;
    stamp_temp1.erase(stamp_temp1.begin());
    stamp_temp2.pop_back();

    for (int iData = 0; iData < stamp_temp1.size(); iData++) {
        diff_stamp.push_back(stamp_temp1[iData] - stamp_temp2[iData]);
    }

    TYPE_STAMP half_window_length = 3.0*calculateMedian(diff_stamp);

    std::vector<TYPE_DATA> local_ref, local_var;
    for (int iData = 0; iData < data_size; iData++) {
        calculateLocalWindow(data_input, data_stamp, half_window_length, iData, local_ref, local_var);
    }

    for (int iData = 0; iData < data_size; iData++) {
        TYPE_DATA temp = std::fabs(data_input[iData] - local_ref[iData]) - 3.0*local_var[iData];
        if (temp > 0) {
          b_outlier_list.push_back(true);
        }
        else {
          b_outlier_list.push_back(false);
        }
    }
}

template <typename TYPE_DATA, typename TYPE_STAMP>
void LaserLoopClosure::calculateLocalWindow(std::vector<TYPE_DATA> data_input, std::vector<TYPE_STAMP> data_stamp,
                          TYPE_STAMP half_window_length, unsigned int data_index,
                          std::vector<TYPE_DATA>& local_ref, std::vector<TYPE_DATA>& local_var) {

    // Index related to local window
    std::vector<int> index_local;
    std::vector<TYPE_STAMP> data_in_window;
    int data_size = data_stamp.size();
    for (int iData = 0; iData < data_size; iData++) {
        if (data_stamp[data_index] - half_window_length <= data_stamp[iData]
            && data_stamp[data_index] + half_window_length >= data_stamp[iData]) {
            index_local.push_back(iData);
            data_in_window.push_back(data_input[iData]);
        }
    }

    // Calculate local nominal data reference value
    TYPE_DATA local_ref_temp = calculateMedian(data_in_window);
    local_ref.push_back(local_ref_temp);

    std::vector<TYPE_DATA> diff_ref;
    for (int iData = 0; iData < data_in_window.size(); iData++) {
        diff_ref.push_back(std::fabs(data_in_window[iData] - local_ref_temp));
    }

    // Calculate local scale of natural variation
    local_var.push_back(1.4826*calculateMedian(diff_ref));
}

template <typename TYPE>
TYPE LaserLoopClosure::calculateMedian(std::vector<TYPE> data_input) {
    std::sort(data_input.begin(), data_input.end());
    unsigned int data_size = data_input.size();
    TYPE median_value;
    // Calculate median
    if (data_size % 2) {
        median_value = data_input[(data_size+1)/2-1];
    }
    else {
        median_value = (data_input[data_size/2-1]+data_input[data_size/2])/2;
    }
    return median_value;
}


UwbRearrangedData LaserLoopClosure::RearrangeUwbData(UwbMeasurementInfo &uwb_data) {

  // Recalculate the nearest pose key at the range measurement timing
  // "uwb_data.nearest_pose_key" already stores the data, so it should be freed up in advance.
  uwb_data.nearest_pose_key.clear();
  for (int itr = 0; itr < uwb_data.time_stamp.size(); itr++) {
    uwb_data.nearest_pose_key.push_back(GetKeyAtTime(uwb_data.time_stamp[itr]));
  }

  // Calculate the distance between the pose key and the point where receiving UWB range measurement
  double dist_temp;
  Eigen::Vector3d pose_at_key;
  for (int itr = 0; itr < uwb_data.range.size(); itr++) {
    pose_at_key = GetPoseAtKey(uwb_data.nearest_pose_key[itr]).translation.Eigen();
    dist_temp = (pose_at_key - uwb_data.robot_position[itr]).norm();
    uwb_data.dist_posekey.push_back(dist_temp);
  }

  // Make a unique list of the pose keys
  auto pose_key_list = uwb_data.nearest_pose_key;
  pose_key_list.erase(std::unique(pose_key_list.begin(), pose_key_list.end()), pose_key_list.end());

  // Classify UWB data in accordance with the pose keys
  std::map<gtsam::Key, UwbDataLinkedWithKey> uwb_posekey2data;
  for (auto itr = pose_key_list.begin(); itr != pose_key_list.end(); itr++) {
    auto bounds     = std::equal_range(uwb_data.nearest_pose_key.begin(), uwb_data.nearest_pose_key.end(), *itr);
    auto itr2_begin = std::distance(uwb_data.nearest_pose_key.begin(), bounds.first);
    auto itr2_end   = std::distance(uwb_data.nearest_pose_key.begin(), bounds.second);
    for (auto itr2 = itr2_begin; itr2 != itr2_end; itr2++) {
      uwb_posekey2data[*itr].range.push_back(uwb_data.range[itr2]);
      uwb_posekey2data[*itr].dist_posekey.push_back(uwb_data.dist_posekey[itr2]);
      uwb_posekey2data[*itr].robot_position.push_back(uwb_data.robot_position[itr2]);
    }
    // TODO: the order of robot_position should be also changed.
    Sort2Vectors<double, double>(uwb_posekey2data[*itr].dist_posekey, uwb_posekey2data[*itr].range);
    uwb_posekey2data[*itr].data_number = uwb_posekey2data[*itr].range.size();
    double range_sum = std::accumulate(uwb_posekey2data[*itr].range.begin(), uwb_posekey2data[*itr].range.end(), 0.0);
    uwb_posekey2data[*itr].range_average = range_sum / uwb_posekey2data[*itr].data_number;
  }

  std::vector<double> range_nearest_key;
  for (auto itr = pose_key_list.begin(); itr != pose_key_list.end(); itr++) {
    if (uwb_posekey2data[*itr].range.size() == 0) {
      pose_key_list.erase(itr);
    }
    else {
      range_nearest_key.push_back(uwb_posekey2data[*itr].range[0]);
    }
  }

  UwbRearrangedData sorted_data;
  Sort2Vectors<double, gtsam::Key>(range_nearest_key, pose_key_list);
  sorted_data.pose_key_list = pose_key_list;
  sorted_data.range_nearest_key = range_nearest_key;
  sorted_data.posekey2data = uwb_posekey2data;

  if (b_use_display_uwb_data_) {
    ShowUwbRawData(uwb_data);
    ShowUwbRearrangedData(sorted_data);
  }

  return sorted_data;
}


void LaserLoopClosure::ShowUwbRawData(const UwbMeasurementInfo uwb_data) {

  std::cout << "----------UWB RAW DATA (start)----------" << "\n";
  std::cout << "UWB ID is " << uwb_data.id << "\n";
  std::cout << "Drop status is " << uwb_data.drop_status << "\n";
  std::cout << "Data size check" << "\n";
  std::cout << "range: " << uwb_data.range.size() << "\t";
  std::cout << "time_stamp: " << uwb_data.time_stamp.size() << "\t";
  std::cout << "robot_position: " << uwb_data.robot_position.size() << "\t";
  std::cout << "nearest_pose_key: " << uwb_data.nearest_pose_key.size() << "\n";
  for (int itr = 0; itr < uwb_data.range.size(); itr++) {
    std::cout << "range: " << uwb_data.range[itr] << "\t";
    std::cout << "time_stamp: " << uwb_data.time_stamp[itr] << "\t";
    // std::cout << ", robot_position: " << uwb_data.robot_position[itr];
    std::cout << "nearest_pose_key: " << uwb_data.nearest_pose_key[itr] << "\t";
    std::cout << "dist_posekey: " << uwb_data.dist_posekey[itr] << "\t";
    std::cout << "\n";
  }
  std::cout << "----------UWB RAW DATA (end)----------" << std::endl;
}


void LaserLoopClosure::ShowUwbRearrangedData(UwbRearrangedData uwb_data) {

  std::cout << "----------UWB SORTED DATA (start)----------" << "\n";
  // 
  for (int itr = 0; itr < uwb_data.pose_key_list.size(); itr++) {
    std::cout << "Pose key: " << uwb_data.pose_key_list[itr] << "\t";
    std::cout << "Range: " << uwb_data.range_nearest_key[itr] << "\n";
  }

  //
  for (auto itr = uwb_data.pose_key_list.begin(); itr != uwb_data.pose_key_list.end(); itr++) {
    std::cout << "Data linked with the pose key " << *itr << "\n";
    UwbDataLinkedWithKey data_temp = uwb_data.posekey2data[*itr];
    std::cout << "Number of range measurement data is " << data_temp.data_number << "\n";
    std::cout << "Average range measurement is " << data_temp.range_average << "\n";
    for (int itr2 = 0; itr2 < data_temp.range.size(); itr2++) {
      std::cout << "range: " << data_temp.range[itr2] << "\t";
      std::cout << "dist_posekey: " << data_temp.dist_posekey[itr2] << "\n";
    }
  }
  std::cout << "----------UWB SORTED DATA (end)----------" << std::endl;
}


template <class T1, class T2>
void LaserLoopClosure::Sort2Vectors(std::vector<T1> &vector1, std::vector<T2> &vector2) {
  int n = vector1.size();
  std::vector<T1> p(n), vec1_temp(n);
  std::vector<T2> vec2_temp(n);
  std::iota(p.begin(), p.end(), 0);
  std::sort(p.begin(), p.end(), [&](T1 i, T1 j) {return vector1[i] < vector1[j];});
  for (int itr = 0; itr < n; itr++) {
      vec1_temp[itr] = vector1[p[itr]];
      vec2_temp[itr] = vector2[p[itr]];
  }
  vector1 = vec1_temp;
  vector2 = vec2_temp;
}

bool LaserLoopClosure::DropUwbAnchor(const std::string uwb_id,
                                     const ros::Time& stamp,
                                     const Eigen::Matrix3d T_rot,
                                     const Eigen::Vector3d robot_position) {

  if (b_uwb_add_betweenfactor_) {
    gtsam::Key uwb_key;
    if (uwb_id2key_hash_.find(uwb_id) != uwb_id2key_hash_.end()) {
      uwb_key = uwb_id2key_hash_[uwb_id];
    }
    else {
      uwb_key = gtsam::Symbol('u', uwb_id2keynumber_hash_[uwb_id]);
      uwb_id2key_hash_[uwb_id] = uwb_key;
      uwb_key2id_hash_[uwb_key] = uwb_id;
    }

    NonlinearFactorGraph new_factor;
    gtsam::Values new_values;
    gtsam::Key pose_key = GetKeyAtTime(stamp);

    // Add a UWB key
    uwb_dropped_position_ = T_rot*uwb_dropped_position_;
    gtsam::Pose3 pose_uwb = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(robot_position+uwb_dropped_position_));
    new_values.insert(uwb_key, pose_uwb);

    // Add a BetweenFactor between the pose key and the UWB key
    // gtsam::Vector6 precisions;
    // precisions.head<3>().setConstant(0.0);
    // precisions.tail<3>().setConstant(4.0);
    // static const gtsam::SharedNoiseModel& noise = 
    // gtsam::noiseModel::Diagonal::Precisions(precisions);
    // // TODO
    // new_factor.add(gtsam::BetweenFactor<gtsam::Pose3>(pose_key, uwb_key, gtsam::Pose3(), noise));

    gu::MatrixNxNBase<double, 6> covariance;
    covariance.Zeros();
    for (int i = 0; i < 3; ++i)
      covariance(i, i) = 1000000; // rotation
    for (int i = 3; i < 6; ++i)
      covariance(i, i) = 1.0; // translation
    
    // Calculate the position of the dropped UWB anchor
    Eigen::Vector3d prev_key_position = GetLastPose().translation.Eigen();
    Eigen::Vector3d uwb_position;
    uwb_position = (robot_position - prev_key_position) + uwb_dropped_position_;

    const gtsam::Pose3 uwb_pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(uwb_position));

    new_factor.add(gtsam::BetweenFactor<gtsam::Pose3>(pose_key, uwb_key, uwb_pose, ToGtsam(covariance)));

    Edge edge_uwb = std::make_pair(pose_key, uwb_key);
    uwb_edges_between_.push_back(edge_uwb);
    edge_poses_[edge_uwb] = uwb_pose;
    covariance_betweenfactor_[edge_uwb] = covariance;

    return (UwbLoopClosureOptimization(new_factor, new_values));
  }
  return true;
}


bool LaserLoopClosure::UwbLoopClosureOptimization(gtsam::NonlinearFactorGraph new_factor,
                                                  gtsam::Values new_values) {

  gtsam::Values linPoint = pgo_solver_->getLinearizationPoint();
  linPoint.insert(new_values);
  nfg_ = pgo_solver_->getFactorsUnsafe();
  double cost; // for debugging
  
  try {
    ROS_INFO_STREAM("Optimizing uwb-based loop closure, iteration");
    gtsam::Values result;

    pgo_solver_->update(new_factor, new_values);
    result = pgo_solver_->calculateEstimate();
    nfg_ = NonlinearFactorGraph(pgo_solver_->getFactorsUnsafe());

    ROS_INFO_STREAM("initial cost = " << nfg_.error(linPoint));
    ROS_INFO_STREAM("final cost = " << nfg_.error(result));

    // Update values
    values_ = result;//

    // INFO stream new cost
    linPoint = pgo_solver_->getLinearizationPoint();
    cost = nfg_.error(linPoint);
    ROS_INFO_STREAM(
        "Cost at linearization point (after adding UWB RangeFactor): " << cost);

    PublishPoseGraph();

    return true;
  }
  catch (...) {
    ROS_ERROR("An error occurred while adding a factor");
    throw;
  }
}

bool LaserLoopClosure::AddKeyScanPair(gtsam::Symbol key,
                                      const PointCloud::ConstPtr& scan, bool initial_pose) {
  if (keyed_scans_.count(key)) {
    ROS_ERROR("%s: Key %u already has a laser scan.", name_.c_str(), key);
    return false;
  }

  // The first key should be treated differently; we need to use the laser
  // scan's timestamp for pose zero.
  if (initial_pose) {
    const ros::Time stamp = pcl_conversions::fromPCL(scan->header.stamp);
    keyed_stamps_.insert(std::pair<gtsam::Symbol, ros::Time>(key, stamp));
    stamps_keyed_.insert(std::pair<double, gtsam::Symbol>(stamp.toSec(), key));
  }

  // ROS_INFO_STREAM("AddKeyScanPair " << key);

  // Add the key and scan.
  keyed_scans_.insert(std::pair<gtsam::Symbol, PointCloud::ConstPtr>(key, scan));

  // Publish the inserted laser scan.
  if (keyed_scan_pub_.getNumSubscribers() > 0) {
    pose_graph_msgs::KeyedScan keyed_scan;
    keyed_scan.key = key;

    pcl::toROSMsg(*scan, keyed_scan.scan);
    keyed_scan_pub_.publish(keyed_scan);
  }

  return true;
}

bool LaserLoopClosure::FindLoopClosures(
    gtsam::Symbol key, std::vector<gtsam::Symbol>* closure_keys) {
  // Function to save the posegraph regularly
  if (key.index() % keys_between_each_posegraph_backup_ == 0 &&
      save_posegraph_backup_) {
    LaserLoopClosure::Save("posegraph_backup.zip");
  } 

  // If loop closure checking is off, don't do this step. This will save some
  // computation time.
  if (!check_for_loop_closures_)
    return false;

  // Don't check for loop closures against poses that are missing scans.
  if (!keyed_scans_.count(key)){
    ROS_WARN("Key %u does not have a scan", gtsam::DefaultKeyFormatter(key));
    return false;
  }

  // Check arguments.
  if (closure_keys == NULL) {
    ROS_ERROR("%s: Output pointer is null.", name_.c_str());
    return false;
  }
  closure_keys->clear();

  ROS_INFO("STARTING FindLoopCLosures...");

  // Update backups
  nfg_backup_ = pgo_solver_->getFactorsUnsafe();
  values_backup_ = pgo_solver_->getLinearizationPoint();

  // To track change in cost
  double cost;
  double cost_old;

  // Check that the key exists
  if (!values_.exists(key)) {
    ROS_WARN("Key %u does not exist in find loop closures",
             gtsam::DefaultKeyFormatter(key));
    return false;
  }

  // If a loop has already been closed recently, don't try to close a new one.
  if (std::fabs(key - last_closure_key_) * translation_threshold_nodes_ <
      distance_before_reclosing_)
    return false;

  // Get pose and scan for the provided key.
  const gu::Transform3 pose1 = ToGu(values_.at<Pose3>(key));
  const PointCloud::ConstPtr scan1 = keyed_scans_[key];

  // Filter the input point cloud once
  PointCloud::Ptr scan1_filtered(new PointCloud);
  filter_.Filter(scan1, scan1_filtered);

  // Transform the input point cloud once. For now its here, can be moved to
  // another function
  const Eigen::Matrix<double, 3, 3> R1 = pose1.rotation.Eigen();
  const Eigen::Matrix<double, 3, 1> t1 = pose1.translation.Eigen();
  Eigen::Matrix4d body1_to_world;
  body1_to_world.block(0, 0, 3, 3) = R1;
  body1_to_world.block(0, 3, 3, 1) = t1;
  PointCloud::Ptr transformedPointCloud(new PointCloud);
  pcl::transformPointCloud(
      *scan1_filtered, *transformedPointCloud, body1_to_world);
  std::string inputCoordinateFrame = "World";

  bool pose_graph_saved_ = false;

  // Iterate through past poses and find those that lie close to the most
  // recently added one.
  bool closed_loop = false;
  bool b_only_allow_one_loop = false; // TODO make this a parameter

  // Make new factor 
  NonlinearFactorGraph new_factor;

  for (const auto& keyed_pose : values_) {
    const gtsam::Symbol other_key = keyed_pose.key;

    if (closed_loop && b_only_allow_one_loop) {
      ROS_INFO("Found one loop with current scan, now exiting...");
      break;
    }

    // Don't self-check.
    if (other_key == key)
      continue;

    // Don't compare against poses that were recently collected.
    if (key > other_key){
      // Don't compare against poses that were recently collected.
      if (std::fabs(key - other_key) < skip_recent_poses_)
        continue;
    }

    if (other_key > key){
      // loopclosure can only occur from high to low valueclosure can only
      continue;
    }


    // Don't check for loop closures against poses that are not keyframes.
    if (!keyed_scans_.count(other_key))
      continue;

    // Check that the key exists
    if (!values_.exists(other_key)) {
      ROS_WARN("Key %u does not exist in loop closure search (other key)",
               gtsam::DefaultKeyFormatter(other_key));
      return false;
    }

    // Get pose for the other key.
    const gu::Transform3 pose2 = ToGu(values_.at<Pose3>(other_key));
    const gu::Transform3 difference = gu::PoseDelta(pose1, pose2);
    if (difference.translation.Norm() < proximity_threshold_) {
      // Found a potential loop closure! Perform ICP between the two scans to
      // determine if there really is a loop to close.
      const PointCloud::ConstPtr scan2 = keyed_scans_[other_key];

      gu::Transform3 delta; // (Using BetweenFactor)
      LaserLoopClosure::Mat66 covariance;
        

      if (PerformICP(transformedPointCloud,
                     scan2,
                     pose1,
                     pose2,
                     &delta,
                     &covariance,
                     true,
                     inputCoordinateFrame)) {

        ROS_INFO("Loop closure found");
        ROS_INFO_STREAM("Key between " << gtsam::DefaultKeyFormatter(key) << " and " << gtsam::DefaultKeyFormatter(other_key));
        // Save the backup pose graph
        if (save_posegraph_backup_ && !pose_graph_saved_) {
          LaserLoopClosure::Save("posegraph_backup.zip");
          pose_graph_saved_ = true;
        }

        //Tell posegraph to update
        has_changed_ = true;

        // We found a loop closure. Add it to the pose graph.
        new_factor.add(BetweenFactor<Pose3>(
            key, other_key, ToGtsam(delta), ToGtsam(covariance)));

        closed_loop = true;

        // Store for visualization and output.
        loop_edges_.push_back(std::make_pair(key, other_key));
        closure_keys->push_back(other_key);

        // Send an message notifying any subscribers that we found a loop
        // closure and having the keys of the loop edge.
        pose_graph_msgs::PoseGraphEdge edge;
        edge.key_from = key;
        edge.key_to = other_key;
        edge.pose = gr::ToRosPose(delta_icp_);
        loop_closure_notifier_pub_.publish(edge);

        // break if a successful loop closure
        // break;
      }
    }
  }// end of for loop

  if (closed_loop){

    ROS_INFO_STREAM("\n\n\t\tOptimiztion for " << new_factor.size() << " loop closure factors\n\n");

    // Compute cost before optimization
    NonlinearFactorGraph nfg_temp = pgo_solver_->getFactorsUnsafe();
    nfg_temp.add(new_factor);
    cost_old = nfg_temp.error(
        values_); // Assume values is up to date - no new values

    // Optimization
    pgo_solver_->update(new_factor);
    
    last_closure_key_ = key;

    // Get updated cost
    nfg_temp = pgo_solver_->getFactorsUnsafe();
    cost = nfg_temp.error(pgo_solver_->getLinearizationPoint());

    // Get values
    values_ = pgo_solver_->calculateEstimate();

    // Update factors
    nfg_ = pgo_solver_->getFactorsUnsafe();

    // Check the change in pose to see if it exceeds criteria
    if (b_check_deltas_ && closed_loop) {
      ROS_INFO("Sanity checking output");
      closed_loop = SanityCheckForLoopClosure(translational_sanity_check_lc_, cost_old, cost);
      // TODO - remove vizualization keys if it is rejected 

      if (!closed_loop){
        ROS_WARN("Returning false for bad loop closure - have reset, waiting for next pose update");
        if (load_graph_from_zip_file_) {
          // Erase the current posegraph to make space for the backup
          // Run the load function to retrieve the posegraph  
          ErasePosegraph();
          Load("posegraph_backup.zip");
        } else {
          // Take the values from the backup before loop closure
          nfg_ = nfg_backup_;
          values_ = values_backup_;
        }
        try {
          pgo_solver_->update(nfg_, values_);
          has_changed_ = true;
        } catch (...) {
          ROS_ERROR("Update ERROR in FindLoopClosures reject from sanity check ");
          throw;
        }
      }
    } // end of if statement 
  } // end of closed loop if statement

  return closed_loop;
}

bool LaserLoopClosure::SanityCheckForLoopClosure(double translational_sanity_check, double cost_old, double cost){
  // Checks loop closures to see if the translational threshold is within limits

  if (!values_backup_.exists(key_-1)){
    ROS_WARN("Key %u does not exist in backup in SanityCheckForLoopClosure");
  }

  // Init poses
  gtsam::Pose3 old_pose;
  gtsam::Pose3 new_pose;

  if (key_ > 1){
    ROS_INFO("Key is more than 1, checking pose change");
    // Previous pose
    old_pose = values_backup_.at<Pose3>(key_-1);
    // New pose
    new_pose = values_.at<Pose3>(key_-1);
  }
  else{
    ROS_INFO("Key is less than or equal to 1, not checking pose change");
    // Second pose - return 
    return true;
  }

  // Translational change 
  double delta = old_pose.compose(new_pose.inverse()).translation().norm();
  
  ROS_INFO("Translational change with update is %f",delta);

  // TODO vary the sanity check values based on what kind of update it is
  // e.g. have the odom update to have a smaller sanity check 
  if (delta > translational_sanity_check || cost > cost_old){ // TODO - add threshold for error in the graph - if it increases after loop closure then reject
    if (delta > translational_sanity_check)
      ROS_WARN("Update delta exceeds threshold, rejecting");
    
    if (cost > cost_old)
      ROS_WARN("Cost increases, rejecting");

    // Updating 
    values_ = values_backup_;
    nfg_ = nfg_backup_;

    // Save updated values
    values_ = pgo_solver_->calculateEstimate();
    nfg_ = pgo_solver_->getFactorsUnsafe();
    ROS_INFO("updated stored values");

    return false;
  }

  return true;

}

bool LaserLoopClosure::GetMaximumLikelihoodPoints(PointCloud* points) {
  if (points == NULL) {
    ROS_ERROR("%s: Output point cloud container is null.", name_.c_str());
    return false;
  }
  points->points.clear();

  // Iterate over poses in the graph, transforming their corresponding laser
  // scans into world frame and appending them to the output.
  for (const auto& keyed_pose : values_) {
    const gtsam::Symbol key = keyed_pose.key;

    // Check if this pose is a keyframe. If it's not, it won't have a scan
    // associated to it and we should continue.
    if (!keyed_scans_.count(key))
      continue;

    // Check that the key exists
    if (!values_.exists(key)) {
      ROS_WARN("Key %u does not exist in GetMaximumLikelihoodPoints",
               gtsam::DefaultKeyFormatter(key));
      return false;
    }
    const gu::Transform3 pose = ToGu(values_.at<Pose3>(key));
    Eigen::Matrix4d b2w;
    b2w.block(0, 0, 3, 3) = pose.rotation.Eigen();
    b2w.block(0, 3, 3, 1) = pose.translation.Eigen();

    // Transform the body-frame scan into world frame.
    PointCloud scan_world;
    pcl::transformPointCloud(*keyed_scans_[key], scan_world, b2w);

    // Append the world-frame point cloud to the output.
    *points += scan_world;
  }
}

bool LaserLoopClosure::GetLatestPointsFromKey(PointCloud* points, gtsam::Symbol key){

  ROS_INFO_STREAM("Target key in GetLatestPoints is " << gtsam::DefaultKeyFormatter(key));

  // Check that the key exists
  if (!values_.exists(key) || keyed_scans_.count(key) == 0) {
    ROS_INFO_STREAM("Key does not exist in GetLatestPointsFromKey. Key is: " << gtsam::DefaultKeyFormatter(key));
    return false;
  }

  const gu::Transform3 pose = ToGu(values_.at<Pose3>(key));
  Eigen::Matrix4d b2w;
  b2w.block(0, 0, 3, 3) = pose.rotation.Eigen();
  b2w.block(0, 3, 3, 1) = pose.translation.Eigen();

  // Transform the body-frame scan into world frame.
  PointCloud scan_world;
  pcl::transformPointCloud(*keyed_scans_[key], scan_world, b2w);

  // Append the world-frame point cloud to the output.
  *points += scan_world;
}

bool LaserLoopClosure::GetLatestPoints(PointCloud* points) {
  if (points == NULL) {
    ROS_ERROR("%s: Output point cloud container is null.", name_.c_str());
    return false;
  }
  ROS_INFO("In Get latest Points");
  points->points.clear();

  if (!values_.exists(key_-1) || keyed_scans_.count(key_-1) == 0){
    ROS_INFO_STREAM("Key does not exist in GetLatestPoints. Key is: " << gtsam::DefaultKeyFormatter(key_-1));
    return false;
  }

  const gu::Transform3 pose = ToGu(values_.at<Pose3>(key_-1));
  Eigen::Matrix4d b2w;
  b2w.block(0, 0, 3, 3) = pose.rotation.Eigen();
  b2w.block(0, 3, 3, 1) = pose.translation.Eigen();

  // Transform the body-frame scan into world frame.
  PointCloud scan_world;
  pcl::transformPointCloud(*keyed_scans_[key_-1], scan_world, b2w);

  // Append the world-frame point cloud to the output.
  *points += scan_world;

  return true;
}

gtsam::Symbol LaserLoopClosure::GetKey() const {
  return key_;
}

gtsam::Symbol LaserLoopClosure::GetInitialKey() const {
  return initial_key_;
}

gu::Transform3 LaserLoopClosure::GetLastPose() const {
  if (key_.index() > 1) {
    return ToGu(values_.at<Pose3>(key_-1));
  } else {
    ROS_WARN("%s: The graph only contains its initial pose.", name_.c_str());
    ROS_INFO_STREAM("Key is: " << gtsam::DefaultKeyFormatter(initial_key_));
    return ToGu(values_.at<Pose3>(initial_key_));
  }
}

gu::Transform3 LaserLoopClosure::GetInitialPose() const {
  if (key_.index() > 1) {
    return ToGu(values_.at<Pose3>(initial_key_));
  } else {
    ROS_WARN("%s: The graph only contains its initial pose.", name_.c_str());
    return ToGu(values_.at<Pose3>(initial_key_));
  }
}

gu::Transform3 LaserLoopClosure::GetInitialLoadedPose() const {
  if (key_.index() > 1) {
    return ToGu(values_.at<Pose3>(first_loaded_key_));
  } else {
    ROS_WARN("%s: The graph only contains its initial pose.", name_.c_str());
    return ToGu(values_.at<Pose3>(first_loaded_key_));
  }
}


gu::Transform3 LaserLoopClosure::ToGu(const Pose3& pose) const {
  gu::Transform3 out;
  out.translation(0) = pose.translation().x();
  out.translation(1) = pose.translation().y();
  out.translation(2) = pose.translation().z();

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j)
      out.rotation(i, j) = pose.rotation().matrix()(i, j);
  }

  return out;
}

Pose3 LaserLoopClosure::ToGtsam(const gu::Transform3& pose) const {
  Vector3 t;
  t(0) = pose.translation(0);
  t(1) = pose.translation(1);
  t(2) = pose.translation(2);

  Rot3 r(pose.rotation(0, 0), pose.rotation(0, 1), pose.rotation(0, 2),
         pose.rotation(1, 0), pose.rotation(1, 1), pose.rotation(1, 2),
         pose.rotation(2, 0), pose.rotation(2, 1), pose.rotation(2, 2));

  return Pose3(r, t);
}

LaserLoopClosure::Mat66 LaserLoopClosure::ToGu(
    const LaserLoopClosure::Gaussian::shared_ptr& covariance) const {
  gtsam::Matrix66 gtsam_covariance = covariance->covariance();

  LaserLoopClosure::Mat66 out;
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      out(i, j) = gtsam_covariance(i, j);

  return out;
}

LaserLoopClosure::Gaussian::shared_ptr LaserLoopClosure::ToGtsam(
    const LaserLoopClosure::Mat66& covariance) const {
  gtsam::Matrix66 gtsam_covariance;

  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      gtsam_covariance(i, j) = covariance(i, j);

  return Gaussian::Covariance(gtsam_covariance);
}

LaserLoopClosure::Gaussian::shared_ptr LaserLoopClosure::ToGtsam(
    const LaserLoopClosure::Mat1212& covariance) const {
  gtsam::Vector12 gtsam_covariance; 
  // TODO CHECK
  for (int i = 0; i < 12; ++i) 
    gtsam_covariance(i) = covariance(i,i);
  return gtsam::noiseModel::Diagonal::Covariance(gtsam_covariance);
}

PriorFactor<Pose3> LaserLoopClosure::MakePriorFactor(
    const Pose3& pose,
    const LaserLoopClosure::Diagonal::shared_ptr& covariance) {
  return PriorFactor<Pose3>(key_, pose, covariance);
}

BetweenFactor<Pose3> LaserLoopClosure::MakeBetweenFactor(
    const Pose3& delta,
    const LaserLoopClosure::Gaussian::shared_ptr& covariance) {
  odometry_edges_.push_back(std::make_pair(key_-1, key_));
  return BetweenFactor<Pose3>(key_-1, key_, delta, covariance);
}

BetweenFactor<Pose3> LaserLoopClosure::MakeBetweenFactorAtLoad(
    const Pose3& delta,
    const LaserLoopClosure::Gaussian::shared_ptr& covariance) {
  odometry_edges_.push_back(std::make_pair(first_loaded_key_, key_));
  return BetweenFactor<Pose3>(first_loaded_key_, key_, delta, covariance);
}

bool LaserLoopClosure::PerformICP(PointCloud::Ptr& scan1,
                                  const PointCloud::ConstPtr& scan2,
                                  const gu::Transform3& pose1,
                                  const gu::Transform3& pose2,
                                  gu::Transform3* delta,
                                  LaserLoopClosure::Mat66* covariance,
                                  const bool is_filtered,
                                  const std::string frame_id) {
  if (delta == NULL || covariance == NULL) {
    ROS_ERROR("%s: Output pointers are null.", name_.c_str());
    return false;
  }

  // Set up ICP.
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  // setVerbosityLevel(pcl::console::L_DEBUG);
  icp.setTransformationEpsilon(icp_tf_epsilon_);
  icp.setMaxCorrespondenceDistance(icp_corr_dist_);
  icp.setMaximumIterations(icp_iterations_);
  icp.setRANSACIterations(0);

  // Filter the two scans. They are stored in the pose graph as dense scans for
  // visualization. Filter the first scan only when it is not filtered already. Can be extended to the other scan if all
  // the key scan pairs store the filtered results. 
  PointCloud::Ptr scan1_filtered;
  if (!is_filtered) {
    scan1_filtered = boost::make_shared<PointCloud>();
    filter_.Filter(scan1, scan1_filtered);
  } else {
    scan1_filtered = scan1;
  }

  PointCloud::Ptr scan2_filtered(new PointCloud);
  filter_.Filter(scan2, scan2_filtered);

  // Set source point cloud. Transform it to pose 2 frame to get a delta.
  PointCloud::Ptr source;
  if (frame_id != "World") {
    const Eigen::Matrix<double, 3, 3> R1 = pose1.rotation.Eigen();
    const Eigen::Matrix<double, 3, 1> t1 = pose1.translation.Eigen();
    Eigen::Matrix4d body1_to_world;
    body1_to_world.block(0, 0, 3, 3) = R1;
    body1_to_world.block(0, 3, 3, 1) = t1;

    source = boost::make_shared<PointCloud>();
    pcl::transformPointCloud(*scan1_filtered, *source, body1_to_world);
  } else {
    source = scan1_filtered;
  }
  icp.setInputSource(source);

  // Transform the target point cloud 
  const Eigen::Matrix<double, 3, 3> R2 = pose2.rotation.Eigen();
  const Eigen::Matrix<double, 3, 1> t2 = pose2.translation.Eigen();
  Eigen::Matrix4d body2_to_world;
  body2_to_world.block(0, 0, 3, 3) = R2;
  body2_to_world.block(0, 3, 3, 1) = t2;

  // Set target point cloud in its own frame.
  PointCloud::Ptr target(new PointCloud);
  pcl::transformPointCloud(*scan2_filtered, *target, body2_to_world);
  icp.setInputTarget(target);

  // Perform ICP.
  PointCloud unused_result;
  icp.align(unused_result);

  // Get resulting transform.
  const Eigen::Matrix4f T = icp.getFinalTransformation();
  // gu::Transform3 delta_icp;
  delta_icp_.translation = gu::Vec3(T(0, 3), T(1, 3), T(2, 3));
  delta_icp_.rotation = gu::Rot3(T(0, 0), T(0, 1), T(0, 2),
                                T(1, 0), T(1, 1), T(1, 2),
                                T(2, 0), T(2, 1), T(2, 2));

  // Is the transform good?
  if (!icp.hasConverged()) {
    std::cout<<"No converged, score is: "<<icp.getFitnessScore() << std::endl;
    return false;
  }

  if (icp.getFitnessScore() > max_tolerable_fitness_) { 
       std::cout<<"Converged, score is: "<<icp.getFitnessScore() << std::endl;
    return false;
  }

  // Update the pose-to-pose odometry estimate using the output of ICP.
  const gu::Transform3 update =
      gu::PoseUpdate(gu::PoseInverse(pose1),
                     gu::PoseUpdate(gu::PoseInverse(delta_icp_), pose1));

  *delta = gu::PoseUpdate(update, gu::PoseDelta(pose1, pose2));

  // TODO: Use real ICP covariance.
  covariance->Zeros();
  for (int i = 0; i < 3; ++i)
    (*covariance)(i, i) = laser_lc_rot_sigma_*laser_lc_rot_sigma_; 
  for (int i = 3; i < 6; ++i)
    (*covariance)(i, i) = laser_lc_trans_sigma_*laser_lc_trans_sigma_; 

  // If the loop closure was a success, publish the two scans.
  source->header.frame_id = fixed_frame_id_;
  target->header.frame_id = fixed_frame_id_;
  scan1_pub_.publish(*source);
  scan2_pub_.publish(*target);

  return true;
}

bool LaserLoopClosure::PerformICP(PointCloud::Ptr& scan1,
                                  const PointCloud::ConstPtr& scan2,
                                  const gu::Transform3& pose1,
                                  const gu::Transform3& pose2,
                                  gu::Transform3* delta,
                                  LaserLoopClosure::Mat1212* covariance,
                                  const bool is_filtered,
                                  const std::string frame_id) {
  if (delta == NULL || covariance == NULL) {
    ROS_ERROR("%s: Output pointers are null.", name_.c_str());
    return false;
  }

  // Set up ICP.
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  // setVerbosityLevel(pcl::console::L_DEBUG);
  icp.setTransformationEpsilon(icp_tf_epsilon_);
  icp.setMaxCorrespondenceDistance(icp_corr_dist_);
  icp.setMaximumIterations(icp_iterations_);
  icp.setRANSACIterations(0);

  // Filter the two scans. They are stored in the pose graph as dense scans for
  // visualization. Filter the first scan only when it is not filtered already. Can be extended to the other scan if all
  // the key scan pairs store the filtered results. 
  PointCloud::Ptr scan1_filtered;
  if (!is_filtered) {
    scan1_filtered = boost::make_shared<PointCloud>();
    filter_.Filter(scan1, scan1_filtered);
  } else {
    scan1_filtered = scan1;
  }
  
  PointCloud::Ptr scan2_filtered(new PointCloud);
  filter_.Filter(scan2, scan2_filtered);

  // Set source point cloud. Transform it to pose 2 frame to get a delta.
  PointCloud::Ptr source;
  if (frame_id != "World") {
    const Eigen::Matrix<double, 3, 3> R1 = pose1.rotation.Eigen();
    const Eigen::Matrix<double, 3, 1> t1 = pose1.translation.Eigen();
    Eigen::Matrix4d body1_to_world;
    body1_to_world.block(0, 0, 3, 3) = R1;
    body1_to_world.block(0, 3, 3, 1) = t1;
    source = boost::make_shared<PointCloud>();
    pcl::transformPointCloud(*scan1_filtered, *source, body1_to_world);
  } else {
    source = scan1_filtered;
  }
  icp.setInputSource(source);

  // Transform target point cloud
  const Eigen::Matrix<double, 3, 3> R2 = pose2.rotation.Eigen();
  const Eigen::Matrix<double, 3, 1> t2 = pose2.translation.Eigen();
  Eigen::Matrix4d body2_to_world;
  body2_to_world.block(0, 0, 3, 3) = R2;
  body2_to_world.block(0, 3, 3, 1) = t2;

  // Set target point cloud in its own frame.
  PointCloud::Ptr target(new PointCloud);
  pcl::transformPointCloud(*scan2_filtered, *target, body2_to_world);
  icp.setInputTarget(target);

  // Perform ICP.
  PointCloud unused_result;
  icp.align(unused_result);

  // Get resulting transform.
  const Eigen::Matrix4f T = icp.getFinalTransformation();
  //gu::Transform3 delta_icp;
  delta_icp_.translation = gu::Vec3(T(0, 3), T(1, 3), T(2, 3));
  delta_icp_.rotation = gu::Rot3(T(0, 0), T(0, 1), T(0, 2),
                                T(1, 0), T(1, 1), T(1, 2),
                                T(2, 0), T(2, 1), T(2, 2));

  // Is the transform good?
  if (!icp.hasConverged()) {
    std::cout<<"No converged, score is: "<<icp.getFitnessScore() << std::endl;
    return false;
  }

  if (icp.getFitnessScore() > max_tolerable_fitness_) {
       std::cout<<"Converged, score is: "<<icp.getFitnessScore() << std::endl;
    return false;
  }

  // Update the pose-to-pose odometry estimate using the output of ICP.
  const gu::Transform3 update =
      gu::PoseUpdate(gu::PoseInverse(pose1),
                     gu::PoseUpdate(gu::PoseInverse(delta_icp_), pose1));

  *delta = gu::PoseUpdate(update, gu::PoseDelta(pose1, pose2));

  // TODO: Use real ICP covariance.
  covariance->Zeros();
  for (int i = 0; i < 9; ++i)
    (*covariance)(i, i) = laser_lc_rot_sigma_*laser_lc_rot_sigma_;
  for (int i = 9; i < 12; ++i)
    (*covariance)(i, i) = laser_lc_trans_sigma_*laser_lc_trans_sigma_;

  // If the loop closure was a success, publish the two scans.
  source->header.frame_id = fixed_frame_id_;
  target->header.frame_id = fixed_frame_id_;
  scan1_pub_.publish(*source);
  scan2_pub_.publish(*target);

  return true;
}

bool LaserLoopClosure::AddManualLoopClosure(gtsam::Key key1, gtsam::Key key2, 
                                            gtsam::Pose3 pose12){

  bool is_manual_loop_closure = true;
  return AddFactor(key1, key2, pose12, is_manual_loop_closure,
                   manual_lc_rot_precision_, manual_lc_trans_precision_); 
}

bool LaserLoopClosure::AddArtifact(gtsam::Key posekey, gtsam::Key artifact_key, gtsam::Pose3 pose12,
                   ArtifactInfo artifact, bool fiducial, gtsam::Point3 location) {

  // keep track of artifact info: add to hash if not added
  if (artifact_key2info_hash_.find(artifact_key) == artifact_key2info_hash_.end()) {
    // ROS_INFO_STREAM("New artifact detected with id" << artifact.id);
    artifact_key2info_hash_[artifact_key] = artifact;
  } else {
    ROS_INFO("Existing artifact detected");
    artifact_key2info_hash_[artifact_key] = artifact;
  }
  // add to pose graph 
  bool is_manual_loop_closure = false;
  
  bool success = AddFactor(posekey, artifact_key, pose12, is_manual_loop_closure,
                 artifact_rot_precision_, artifact_trans_precision_);
  if (fiducial) {
    gtsam::Vector6 precisions;                       // inverse of variances
    precisions.head<3>().setConstant(fiducial_rot_precision_); // TODO: define in params
    precisions.tail<3>().setConstant(fiducial_trans_precision_); // TODO: define in params
    static const gtsam::SharedNoiseModel& noise =
        gtsam::noiseModel::Diagonal::Precisions(precisions);
    gtsam::NonlinearFactorGraph prior_factor;
    prior_factor.add(gtsam::PriorFactor<gtsam::Pose3>(artifact_key, gtsam::Pose3(gtsam::Rot3(),  location), noise)); 
    pgo_solver_->update(prior_factor, gtsam::Values());
  }
  return success;
}

bool LaserLoopClosure::AddFactor(gtsam::Key key1, gtsam::Key key2, 
                                 gtsam::Pose3 pose12, 
                                 bool is_manual_loop_closure,
                                 double rot_precision, 
                                 double trans_precision) {
  // Thanks to Luca for providing the code
  ROS_INFO_STREAM("Adding factor between " << gtsam::DefaultKeyFormatter(key1) << " and " << gtsam::DefaultKeyFormatter(key2));

  gtsam::Values linPoint = pgo_solver_->getLinearizationPoint();
  nfg_ = pgo_solver_->getFactorsUnsafe();

  // Update backups
  nfg_backup_ = pgo_solver_->getFactorsUnsafe();
  values_backup_ = pgo_solver_->getLinearizationPoint();

  // Remove visualization of edge to be confirmed
  if (is_manual_loop_closure) {
    // check keys are already in factor graph 
    if (!linPoint.exists(key1) || !linPoint.exists(key2)) { 
      ROS_WARN("AddFactor: Trying to add manual loop closure involving at least one nonexisting key");
      return false;
    }
  }

  double cost_old;
  double cost; // for sanity checks

  NonlinearFactorGraph new_factor;
  gtsam::Values new_values;

  // Flag to track if the artifact addition is a look closure 
  bool b_artifact_loop_closure = !is_manual_loop_closure && linPoint.exists(key2);

  if (!b_artifact_loop_closure && !is_manual_loop_closure) {
    // Adding a new artifact
    if(!linPoint.exists(key1)) {
      ROS_WARN("AddFactor: Trying to add artifact factor, but key1 does not exist");
      return false;    
    }
    // We should add initial guess to values 
    new_values.insert(key2, linPoint.at<gtsam::Pose3>(key1).compose(pose12));
    ROS_INFO("New artifact added");

    ROS_INFO("Initial global position of artifact is: %f, %f, %f",
                  new_values.at<Pose3>(key2).translation().vector().x(),
                  new_values.at<Pose3>(key2).translation().vector().y(),
                  new_values.at<Pose3>(key2).translation().vector().z());

    // Add timestamps (don't want to reference this with time, so just one way)
    keyed_stamps_.insert(
        std::pair<gtsam::Symbol, ros::Time>(key_, artifact_key2info_hash_[key2].msg.header.stamp));
  }

  linPoint.insert(new_values); // insert new values (valid for all cases)

  // Use BetweenFactor
  // creating relative pose factor (also works for relative positions)

  // create Information of measured
  gtsam::Vector6 precisions;                       // inverse of variances
  precisions.head<3>().setConstant(rot_precision); // rotation precision
  precisions.tail<3>().setConstant(
      trans_precision); // std: 1/1000 ~ 30 m 1/100 - 10 m 1/25 - 5m
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Diagonal::Precisions(precisions);

  gtsam::BetweenFactor<gtsam::Pose3> factor(key1, key2, pose12, noise);

  if (is_manual_loop_closure) {
    factor.print("manual loop closure factor \n");
    cost = factor.error(linPoint);
    ROS_INFO_STREAM(
        "Cost of loop closure: "
        << cost); // 10^6 - 10^9 is ok (re-adjust covariances)  // cost = (
                  // error )’ Omega ( error ), where the Omega = diag([0 0 0
                  // 1/25 1/25 1/25]). Error = [3 3 3] get an estimate for cost.
    // TODO get the positions of each of the poses and compute the distance
    // between them - see what the error should be - maybe a bug there
  } else {
    if (b_artifact_loop_closure){
      factor.print("Artifact loop closure factor \n");
      cost = factor.error(linPoint);
      ROS_INFO_STREAM("Cost of artifact factor is: " << cost);
    } else {
      factor.print("New Artifact factor \n");
    }
  }

  // add factor to factor graph
  new_factor.add(factor);

  // Store cost before optimization
  cost_old = new_factor.error(linPoint);

  // optimize
  try {
    if (is_manual_loop_closure) {
      std::cout << "Optimizing manual loop closure, iteration" << std::endl;
    } else {
      std::cout << "Optimizing artifact factor addition" << std::endl;
    }
    gtsam::Values result;

    pgo_solver_->update(new_factor, new_values);
    result = pgo_solver_->calculateEstimate();
    nfg_ = NonlinearFactorGraph(pgo_solver_->getFactorsUnsafe());

    std::cout << "initial cost = " << nfg_.error(linPoint) << std::endl;
    std::cout << "final cost = " << nfg_.error(result) << std::endl;

    if (is_manual_loop_closure) {
      // Store for visualization and output.
      loop_edges_.push_back(std::make_pair(key1, key2));

      // Store manual loop keys to not interfere with batch loop closure.
      manual_loop_edges_.push_back(std::make_pair(key1, key2));

    }
    else{
      // Placeholder visualization for artifacts
      artifact_edges_.push_back(std::make_pair(key1, key2));
    }

    // Send an message notifying any subscribers that we found a loop
    // closure and having the keys of the loop edge.
    if (b_artifact_loop_closure || is_manual_loop_closure){
      pose_graph_msgs::PoseGraphEdge edge;
      edge.key_from = key1;
      edge.key_to = key2;
      edge.pose = gr::ToRosPose(ToGu(pose12));
      loop_closure_notifier_pub_.publish(edge);
    }

    // Update values
    values_ = result;//

    // INFO stream new cost
    linPoint = pgo_solver_->getLinearizationPoint();
    cost = nfg_.error(linPoint);
    ROS_INFO_STREAM(
        "Solver cost at linearization point (after loop closure): " << cost);

    // Check the change in pose to see if it exceeds criteria
    if (b_check_deltas_){
      ROS_INFO("Sanity checking output");
      bool check_result = SanityCheckForLoopClosure(translational_sanity_check_lc_, cost_old, cost);

       if (!check_result){
        ROS_WARN("Returning false for bad loop closure - have reset, waiting for next pose update");
        if (load_graph_from_zip_file_) {
          // Erase the current posegraph to make space for the backup
          ErasePosegraph();
          // Run the load function to retrieve the posegraph  
          Load("posegraph_backup.zip");
        } else {
          // Reload from backup
          nfg_ = nfg_backup_;
          values_ = values_backup_;
        }
        try {
          pgo_solver_->update(nfg_, values_);
          has_changed_ = true;
        } catch (...) {
          ROS_ERROR("Update ERROR in AddFactors rejection from sanity check");
          throw;
        }
        return false;
      }

    }

    // flag to note change in graph
    has_changed_ = true;

    return true;
  } catch (...) {
    ROS_ERROR(
        "An ERROR occurred while manually adding a factor to the PGO solver.");
    throw;
  }
}

bool LaserLoopClosure::RemoveFactor(gtsam::Symbol key1,
                                    gtsam::Symbol key2,
                                    bool is_batch_loop_closure) {
  ROS_INFO("Removing factor between %i and %i from the pose graph...", key1, key2);

  // Prevent removing odometry edges 
  if ((key1 == key2 - 1) || (key2 == key1 - 1)) {
    ROS_WARN("RemoveFactor: Removing edges from consecutive poses (odometry) is currently forbidden (disable if condition to allow)");
    return false; 
  }

  // 1. Get factor graph
  NonlinearFactorGraph nfg = pgo_solver_->getFactorsUnsafe();
  // 2. Search for the two keys
  gtsam::FactorIndices factorsToRemove;
  for (size_t slot = 0; slot < nfg.size(); ++slot) {
    const gtsam::NonlinearFactor::shared_ptr& f = nfg[slot];
    if (f) {
      boost::shared_ptr<gtsam::BetweenFactor<Pose3>> pose3Between =
          boost::dynamic_pointer_cast<gtsam::BetweenFactor<Pose3>>(nfg[slot]);

      if (pose3Between) {
        if ((pose3Between->key1() == key1 && pose3Between->key2() == key2) ||
            (pose3Between->key1() == key2 && pose3Between->key2() == key1)) {
          factorsToRemove.push_back(slot);
          nfg[slot]->print("Removing... ");
        }
      }
    }
  }

  if (factorsToRemove.size() == 0) {
    ROS_WARN("RemoveFactor: Factor not found between given keys");
    return false; 
  }

  ROS_INFO("removing manual loop edge visualization");
  // Remove the visual edge of the factor
  std::vector<int> remove_indices;
  for (int i = 0; i < loop_edges_.size(); i++) {
    if ((key1 == loop_edges_[i].first && key2 == loop_edges_[i].second) ||
        (key1 == loop_edges_[i].second && key2 == loop_edges_[i].first)) {
      // Remove the edge from LaserLoopClosure
      remove_indices.push_back(i);

      // Send a message to posegraph visualizer that the edges must be updated
      if (remove_factor_viz_pub_.getNumSubscribers() > 0) {
        std_msgs::Bool empty_edge;
        empty_edge.data = true;
        remove_factor_viz_pub_.publish(empty_edge);
      }
    }
  }

  ROS_INFO("About to remove loop_edges");
  // Remove from the edges
  for (int i = 0; i < remove_indices.size(); i++){
    loop_edges_.erase(loop_edges_.begin() + remove_indices[i]);
  }

  // 3. Remove factors and update
  ROS_INFO("Before remove factor in pgo_solver");
  // Only updating the factor graph to remove factors
  pgo_solver_->removeFactorsNoUpdate(factorsToRemove);

  // Update values
  values_ = pgo_solver_->calculateEstimate();

  // Publish
  has_changed_ = true;
  PublishPoseGraph();

  return true; 
}

std::string absPath(const std::string &relPath) {
  return boost::filesystem::canonical(boost::filesystem::path(relPath)).string();
}

bool writeFileToZip(zipFile &zip, const std::string &filename) {
  // this code is inspired by http://www.vilipetek.com/2013/11/22/zippingunzipping-files-in-c/
  static const unsigned int BUFSIZE = 2048;

  zip_fileinfo zi = {0};
  tm_zip& tmZip = zi.tmz_date;
  time_t rawtime;
  time(&rawtime);
  auto timeinfo = localtime(&rawtime);
  tmZip.tm_sec = timeinfo->tm_sec;
  tmZip.tm_min = timeinfo->tm_min;
  tmZip.tm_hour = timeinfo->tm_hour;
  tmZip.tm_mday = timeinfo->tm_mday;
  tmZip.tm_mon = timeinfo->tm_mon;
  tmZip.tm_year = timeinfo->tm_year;
  int err = zipOpenNewFileInZip(zip, filename.c_str(), &zi,
      NULL, 0, NULL, 0, NULL, Z_DEFLATED, Z_DEFAULT_COMPRESSION);

  if (err != ZIP_OK) {
    ROS_ERROR_STREAM("Failed to add entry \"" << filename << "\" to zip file.");
    return false;
  }
  char buf[BUFSIZE];
  unsigned long nRead = 0;

  std::ifstream is(filename);
  if (is.bad()) {
    ROS_ERROR_STREAM("Could not read file \"" << filename << "\" to be added to zip file.");
    return false;
  }
  while (err == ZIP_OK && is.good()) {
    is.read(buf, BUFSIZE);
    unsigned int nRead = (unsigned int)is.gcount();
    if (nRead)
      err = zipWriteInFileInZip(zip, buf, nRead);
    else
      break;
  }
  is.close();
  if (err != ZIP_OK) {
    ROS_ERROR_STREAM("Failed to write file \"" << filename << "\" to zip file.");
    return false;
  }
  return true;
}

bool LaserLoopClosure::ErasePosegraph(){
  keyed_scans_.clear();
  keyed_stamps_.clear();
  stamps_keyed_.clear();

  loop_edges_.clear();
  manual_loop_edges_.clear();
  odometry_ = Pose3::identity();
  odometry_kf_ = Pose3::identity();
  odometry_edges_.clear();

  // Send message to Pose graph visualizer that it needs to be erased
  if (erase_posegraph_pub_.getNumSubscribers() > 0) {
    std_msgs::Bool erase;
    erase.data = true;
    // Publish.
    erase_posegraph_pub_.publish(erase);
  }

  has_changed_ = true;
} 

bool LaserLoopClosure::Save(const std::string &zipFilename) const {
  const std::string path = "pose_graph";
  const boost::filesystem::path directory(path);
  boost::filesystem::create_directory(directory);

  pgo_solver_->getFactorsUnsafe().print("In save, pgo_solver_nfg_ is: ") ;
  nfg_.print("In save, from nfg_ is: ") ;

  writeG2o(pgo_solver_->getFactorsUnsafe(), pgo_solver_->calculateEstimate(), path + "/graph.g2o");
  ROS_INFO("Saved factor graph as a g2o file.");

  // keys.csv stores factor key, point cloud filename, and time stamp
  std::ofstream keys_file(path + "/keys.csv");
  if (keys_file.bad()) {
    ROS_ERROR("Failed to write keys file.");
    return false;
  }

  auto zipFile = zipOpen64(zipFilename.c_str(), 0);
  writeFileToZip(zipFile, path + "/graph.g2o");
  int i = 0;
  for (const auto &entry : keyed_scans_) {
    keys_file << entry.first << ",";
    // save point cloud as binary PCD file
    const std::string pcd_filename = path + "/pc_" + std::to_string(i) + ".pcd";
    pcl::io::savePCDFile(pcd_filename, *entry.second, true);
    writeFileToZip(zipFile, pcd_filename);

    ROS_INFO("Saved point cloud %d/%d.", i+1, (int) keyed_scans_.size());
    keys_file << pcd_filename << ",";
    if (!values_.exists(entry.first)) {
      ROS_WARN("Key,  %u, does not exist in Save", entry.first);
      return false;
    }
    keys_file << keyed_stamps_.at(entry.first).toNSec() << "\n";
    ++i;
  }
  keys_file.close();
  writeFileToZip(zipFile, path + "/keys.csv");

  // save odometry edges
  std::ofstream odometry_edges_file(path + "/odometry_edges.csv");
  if (odometry_edges_file.bad()) {
    ROS_ERROR("Failed to write odometry_edges file.");
    return false;
  }
  for (const auto &entry : odometry_edges_) {
    odometry_edges_file << entry.first << ',' << entry.second << '\n';
  }
  odometry_edges_file.close();
  writeFileToZip(zipFile, path + "/odometry_edges.csv");

  // save loop edges
  std::ofstream loop_edges_file(path + "/loop_edges.csv");
  if (loop_edges_file.bad()) {
    ROS_ERROR("Failed to write loop_edges file.");
    return false;
  }
  for (const auto &entry : loop_edges_) {
    loop_edges_file << entry.first << ',' << entry.second << '\n';
  }
  loop_edges_file.close();
  writeFileToZip(zipFile, path + "/loop_edges.csv");

  zipClose(zipFile, 0);
  boost::filesystem::remove_all(directory);
  ROS_INFO_STREAM("Successfully saved pose graph to " << absPath(zipFilename) << ".");
}

bool LaserLoopClosure::Load(const std::string &zipFilename) {
  const std::string absFilename = absPath(zipFilename);
  auto zipFile = unzOpen64(zipFilename.c_str());
  //Storing current key before loading graph to set key to this after
  stored_key_ = key_;
  if (!zipFile) {
    ROS_ERROR_STREAM("Failed to open zip file " << absFilename);
    return false;
  }

  unz_global_info64 oGlobalInfo;
  int err = unzGetGlobalInfo64(zipFile, &oGlobalInfo);
  std::vector<std::string> files;  // files to be extracted

  std::string graphFilename{""}, keysFilename{""},
              odometryEdgesFilename{""}, loopEdgesFilename{""};

  for (unsigned long i = 0; i < oGlobalInfo.number_entry && err == UNZ_OK; ++i) {
    char filename[256];
    unz_file_info64 oFileInfo;
    err = unzGetCurrentFileInfo64(zipFile, &oFileInfo, filename,
                                  sizeof(filename), NULL, 0, NULL, 0);
    if (err == UNZ_OK) {
      char nLast = filename[oFileInfo.size_filename-1];
      // this entry is a file, extract it later
      files.emplace_back(filename);
      if (files.back().find("graph.g2o") != std::string::npos) {
        graphFilename = files.back();
      } else if (files.back().find("keys.csv") != std::string::npos) {
        keysFilename = files.back();
      } else if (files.back().find("odometry_edges.csv") != std::string::npos) {
        odometryEdgesFilename = files.back();
      } else if (files.back().find("loop_edges.csv") != std::string::npos) {
        loopEdgesFilename = files.back();
      }
      err = unzGoToNextFile(zipFile);
    }
  }

  if (graphFilename.empty()) {
    ROS_ERROR_STREAM("Could not find pose graph g2o-file in " << absFilename);
    return false;
  }
  if (keysFilename.empty()) {
    ROS_ERROR_STREAM("Could not find keys.csv in " << absFilename);
    return false;
  }

  // extract files
  int i = 1;
  std::vector<boost::filesystem::path> folders;
  for (const auto &filename: files) {
    if (unzLocateFile(zipFile, filename.c_str(), 0) != UNZ_OK) {
			ROS_ERROR_STREAM("Could not locate file " << filename << " from " << absFilename);
      return false;
    }
    if (unzOpenCurrentFile(zipFile) != UNZ_OK) {
      ROS_ERROR_STREAM("Could not open file " << filename << " from " << absFilename);
      return false;
    }
    unz_file_info64 oFileInfo;
    if (unzGetCurrentFileInfo64(zipFile, &oFileInfo, 0, 0, 0, 0, 0, 0) != UNZ_OK)
    {
      ROS_ERROR_STREAM("Could not determine file size of entry " << filename << " in " << absFilename);
      return false;
    }

    boost::filesystem::path dir(filename);
    dir = dir.parent_path();
    if (boost::filesystem::create_directory(dir))
      folders.emplace_back(dir);

    auto size = (unsigned int) oFileInfo.uncompressed_size;
    char* buf = new char[size];
    size = unzReadCurrentFile(zipFile, buf, size);
    std::ofstream os(filename);
    if (os.bad()) {
      ROS_ERROR_STREAM("Could not create file " << filename << " for extraction.");
      return false;
    }
    if (size > 0) {
      os.write(buf, size);
      os.flush();
    } else {
      ROS_WARN_STREAM("Entry " << filename << " from " << absFilename << " is empty.");
    }
    os.close();
    delete [] buf;
    ROS_INFO_STREAM("Extracted file " << i << "/" << (int) files.size() << " -- " << filename);
    ++i;
  }
  unzClose(zipFile);

  // restore pose graph from g2o file
  const GraphAndValues gv = gtsam::load3D(graphFilename);
  nfg_ = *gv.first;
  nfg_.print("File output facrtors are: ");
  values_ = *gv.second;
  ROS_INFO_STREAM("1");

  pgo_solver_.reset(new RobustPGO::RobustSolver(rpgo_params_));
  pgo_solver_->print();
  ROS_INFO_STREAM("2");

  // TODO: Should store initial_noise to use in load
  const LaserLoopClosure::Diagonal::shared_ptr covariance(
      LaserLoopClosure::Diagonal::Sigmas(initial_noise_));
  const gtsam::Symbol key0 = gtsam::Symbol(*nfg_.keys().begin());
  first_loaded_key_ = key0;
  ROS_INFO_STREAM("3");
  if (!values_.exists(key0)){
    ROS_WARN("Key0, %s, does not exist in Load",
             gtsam::DefaultKeyFormatter(key0));
    return false;
  }
  // update with prior
  pgo_solver_->loadGraph<Pose3>(
      nfg_,
      values_,
      gtsam::PriorFactor<Pose3>(key0, values_.at<Pose3>(key0), covariance));

  ROS_INFO_STREAM("Updated graph from " << graphFilename);

  // info_file stores factor key, point cloud filename, and time stamp
  std::ifstream info_file(keysFilename);
  if (info_file.bad()) {
    ROS_ERROR_STREAM("Failed to open " << keysFilename);
    return false;
  }
  ROS_INFO_STREAM("5");
  std::string keyStr, pcd_filename, timeStr;
  while (info_file.good()) {
    std::getline(info_file, keyStr, ',');
    if (keyStr.empty())
      break;
    key_ = gtsam::Symbol (std::stoull(keyStr));
    std::getline(info_file, pcd_filename, ',');
    PointCloud::Ptr pc(new PointCloud);
    if (pcl::io::loadPCDFile(pcd_filename, *pc) == -1) {
      ROS_ERROR_STREAM("Failed to load point cloud " << pcd_filename << " from " << absFilename);
      return false;
    }
    ROS_INFO_STREAM("Loaded point cloud " << pcd_filename);
    keyed_scans_[key_] = pc;
    std::getline(info_file, timeStr);
    ros::Time t;
    t.fromNSec(std::stol(timeStr));
    keyed_stamps_[key_] = t;
    //stamps_keyed_[t] = key_ ;
  }
  ROS_INFO_STREAM("6");
  // Increment key to be ready for more scans
  key_ = key_+ 1;
  ROS_INFO("Restored all point clouds.");
  info_file.close();

  if (!odometryEdgesFilename.empty()) {
    std::ifstream edge_file(odometryEdgesFilename);
    if (edge_file.bad()) {
      ROS_ERROR_STREAM("Failed to open " << odometryEdgesFilename);
      return false;
    }
    std::string edgeStr;
    while (edge_file.good()) {
      Edge edge;
      std::getline(edge_file, edgeStr, ',');
      if (edgeStr.empty())
        break;
      edge.first = static_cast<gtsam::Key>(std::stoull(edgeStr));
      std::getline(edge_file, edgeStr);
      edge.second = static_cast<gtsam::Key>(std::stoull(edgeStr));
      odometry_edges_.emplace_back(edge);
    }
    edge_file.close();
    ROS_INFO("Restored odometry edges.");
  }
  ROS_INFO_STREAM("7");
  if (!loopEdgesFilename.empty()) {
    std::ifstream edge_file(loopEdgesFilename);
    if (edge_file.bad()) {
      ROS_ERROR_STREAM("Failed to open " << loopEdgesFilename);
      return false;
    }
    std::string edgeStr;
    while (edge_file.good()) {
      Edge edge;
      std::getline(edge_file, edgeStr, ',');
      if (edgeStr.empty())
        break;
      edge.first = static_cast<gtsam::Key>(std::stoull(edgeStr));
      std::getline(edge_file, edgeStr);
      edge.second = static_cast<gtsam::Key>(std::stoull(edgeStr));
      loop_edges_.emplace_back(edge);
    }
    edge_file.close();
    ROS_INFO("Restored loop closure edges.");
  }
  ROS_INFO_STREAM("8");
  // remove all extracted folders
  for (const auto &folder: folders)
    boost::filesystem::remove_all(folder);

  ROS_INFO_STREAM("Successfully loaded pose graph from " << absPath(zipFilename) << ".");
  PublishPoseGraph();
  return true;
}

bool LaserLoopClosure::BatchLoopClosure() {
  // Store parameter values as initalized in parameters.yaml in memory
  bool save_posegraph = save_posegraph_backup_;
  bool loop_closure_checks = check_for_loop_closures_;
  double store_distance_before_reclosing = distance_before_reclosing_;

  // Change the distance before reclosing a loop when doing batch loop closure
  distance_before_reclosing_ = distance_before_batch_reclosing_;

  //Disable save flag before doing optimization
  save_posegraph_backup_ = false;

  //Enable loop-closure before running search
  check_for_loop_closures_ = true;
  
  //Remove all manual factors to not make the system underdetermined
  for (int i = 0; i < manual_loop_edges_.size(); i++){
    bool is_batch_loop_closure = true;
    RemoveFactor(manual_loop_edges_[i].first, manual_loop_edges_[i].second, is_batch_loop_closure);
  }

  ROS_INFO("Removed manual loop closure factors");

  bool found_loop = false;
  //Loop through all keyed scans and look for loop closures
  for (const auto& keyed_pose : values_) {
    std::vector<gtsam::Symbol> closure_keys;
    if (FindLoopClosures(keyed_pose.key, &closure_keys)){
      found_loop = true;
    }
  }
  // Restore the flags as initalized in parameters.yaml from memory
  save_posegraph_backup_ = save_posegraph;
  check_for_loop_closures_ = loop_closure_checks;
  distance_before_reclosing_ = store_distance_before_reclosing;

  // Update the posegraph after looking for loop closures and performing optimization
  has_changed_ = true;
  // publish posegraph called in BlamSlam.cc

  if (found_loop == true)
    return true;
  else
    return false;
}

geometry_msgs::Quaternion LaserLoopClosure::CorrectMapRotation(Eigen::Vector3d v1,
                                          gtsam::Key gate_key,
                                          gtsam::Key distal_key,
                                          std::string robot_name) {
  std::cout << "Received the CorrectMapRotation request in LaserLoopClosure..."
            << std::endl;
  // geometry_utils::Transform3 gate_pose = GetPoseAtKey(gate_key);
  geometry_utils::Transform3 distal_pose = GetPoseAtKey(distal_key);

  // Retrive the location of the AprilTag4 from the map
  // Eigen::Matrix<double, 3, 1> T_gate = gate_pose.translation.Eigen();
  
  // Retrive the location of the AprilTag26 from the map
  Eigen::Matrix<double, 3, 1> T_distal = distal_pose.translation.Eigen();
  
  // Find the vector connecting the gate to distal AprilTag in the map
  // Eigen::Matrix<double, 3, 1> T_delta = T_distal - T_gate;

  // Convert the vector to a vector3d
  Eigen::Vector3d v2(T_distal(0, 0), T_distal(1, 0), T_distal(2, 0));
  // Compute the quaternion that represents the rotation going from v2 to v1
  std::cout << "Find the 3D rotation between the map and GT..." << std::endl;

  Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(v1, v2);
  geometry_msgs::Quaternion q_msg;
  tf::quaternionEigenToMsg(q, q_msg);
  // Normalize the quaternion and get the corrispondant rotation matrix
  Eigen::Matrix3d R = q.normalized().toRotationMatrix();
  // Print the computed rotation matrix between the map and ground truth
  Eigen::Vector3d Euler = R.eulerAngles(2, 1, 0);

  std::cout << "The computed 3D rotation between the map and GT is R= "
            << std::endl
            << R << std::endl;

  std::string path =
      homedir + "/.ros/global_map_rotation_" + robot_name + ".yaml";
  std::ofstream file(path);
  std::stringstream ss;
  ss << "position:" << std::endl
     << "  x: " << 0 << std::endl
     << "  y: " << 0 << std::endl
     << "  z: " << 0 << std::endl
     << "orientation:" << std::endl
     << "  x: " << q.normalized().x() << std::endl
     << "  y: " << q.normalized().y() << std::endl
     << "  z: " << q.normalized().z() << std::endl
     << "  w: " << q.normalized().w() << std::endl
     << "Euler:" << std::endl
     << "  Roll: " << Euler[2] << std::endl
     << "  Pitch: " << Euler[1] << std::endl
     << "  Yaw: " << Euler[0] << std::endl;
  file << ss.str();
  file.close();

  // Set rosparam
  ros::NodeHandle n("global_map_rotation");
  n.setParam("position/x", 0);
  n.setParam("position/y", 0);
  n.setParam("position/z", 0);
  n.setParam("orientation/x", q.normalized().x());
  n.setParam("orientation/y", q.normalized().y());
  n.setParam("orientation/z", q.normalized().z());
  n.setParam("orientation/w", q.normalized().w());
  return q_msg;
}

bool LaserLoopClosure::PublishPoseGraph(bool only_publish_if_changed) {

  //has_changed must be true to update the posegraph
  if (only_publish_if_changed && !has_changed_)
    return false;
  
  has_changed_ = false;

  // update inliers 
  updateInlierLoopEdges();

  // Construct and send the pose graph.
  if (pose_graph_pub_.getNumSubscribers() > 0) {
    pose_graph_msgs::PoseGraph g;
    g.header.frame_id = fixed_frame_id_;
    g.header.stamp = keyed_stamps_[key_ - 1]; // Get timestamp from latest keyed pose

    // Flag on whether it is incremental or not
    // TODO make incremental Pose Graph publishing
    g.incremental = false;

    for (const auto& keyed_pose : values_) {
      if (!values_.exists(keyed_pose.key)) {
        ROS_WARN("Key, %lu, does not exist in PublishPoseGraph pose graph pub", keyed_pose.key);
        return false;
      }
      gu::Transform3 t = ToGu(values_.at<Pose3>(keyed_pose.key));

      gtsam::Symbol sym_key = gtsam::Symbol(keyed_pose.key);

      // Populate the message with the pose's data.
      pose_graph_msgs::PoseGraphNode node;
      node.key = keyed_pose.key;
      node.header.frame_id = fixed_frame_id_;
      node.pose = gr::ToRosPose(t);
      if (keyed_scans_.count(keyed_pose.key)) {
        node.header.stamp = keyed_stamps_[keyed_pose.key];
        // Key frame, note in the ID
        node.ID = "key_frame";
      } else if (artifact_key2info_hash_.count(keyed_pose.key)){
        node.header.stamp = artifact_key2info_hash_[keyed_pose.key].msg.header.stamp;
      } else if (uwb_key2id_hash_.count(keyed_pose.key)){
        ROS_INFO("UWB node, leaving node timestamp empty in pose graph message");
      } else {
        ROS_INFO_STREAM("Node " << gtsam::DefaultKeyFormatter(keyed_pose.key) << " is not a key frame, leaving timestamp empty in pose graph message");
        // Normal odom node, note in ID
        node.ID = "odom_node";
      }

      // Add UUID if an artifact or uwb node
      if (sym_key.chr() == 'l' || sym_key.chr() == 'm' || sym_key.chr() == 'n' || sym_key.chr() == 'o' || sym_key.chr() == 'p' || sym_key.chr() == 'q'){
        // Artifact
        node.ID = artifact_key2info_hash_[keyed_pose.key].msg.parent_id;
      }
      if (sym_key.chr() == 'u'){
        // UWB
        node.ID = uwb_key2id_hash_[keyed_pose.key];
      }

      g.nodes.push_back(node);
    }

    pose_graph_msgs::PoseGraphEdge edge;
    for (size_t ii = 0; ii < odometry_edges_.size(); ++ii) {
      edge.key_from = odometry_edges_[ii].first;
      edge.key_to = odometry_edges_[ii].second;

      // Tocleanup! will find it from nfg_ not edge_poses_
      edge.pose = gr::ToRosPose(ToGu(edge_poses_[odometry_edges_[ii]]));
      //edge.covariance = covariance_betweenfactor_[odometry_edges_[ii]];
      edge.type = pose_graph_msgs::PoseGraphEdge::ODOM;

      // factors is protected, can maybe make a getter function inside it?
      // const auto& measured = nfg_.factors_[ii].measured();
      // Get covariance
      // TODO
      g.edges.push_back(edge);
    }

    for (size_t ii = 0; ii < inlier_loop_edges_.size(); ++ii) {
      edge.key_from = inlier_loop_edges_[ii].first;
      edge.key_to = inlier_loop_edges_[ii].second;
      edge.type = pose_graph_msgs::PoseGraphEdge::LOOPCLOSE;


      // Get edge transform and covariance
      // TODO
      g.edges.push_back(edge);
    }

    for (size_t ii = 0; ii < artifact_edges_.size(); ++ii) {
      edge.key_from = artifact_edges_[ii].first;
      edge.key_to = artifact_edges_[ii].second;
      edge.type = pose_graph_msgs::PoseGraphEdge::ARTIFACT;
      // Get edge transform and covariance
      // TODO
      g.edges.push_back(edge);
    }

    for (size_t ii = 0; ii < uwb_edges_range_.size(); ++ii) {
      edge.key_from = uwb_edges_range_[ii].first;
      edge.key_to = uwb_edges_range_[ii].second;
      edge.type = pose_graph_msgs::PoseGraphEdge::UWB_RANGE;
      edge.range = edge_ranges_[uwb_edges_range_[ii]];
      edge.range_error = error_rangefactor_[uwb_edges_range_[ii]];
      g.edges.push_back(edge);
    }

    for (size_t ii = 0; ii < uwb_edges_between_.size(); ++ii) {
      edge.key_from = uwb_edges_between_[ii].first;
      edge.key_to = uwb_edges_between_[ii].second;
      edge.type = pose_graph_msgs::PoseGraphEdge::UWB_BETWEEN;
      // Tocleanup! will find it from nfg_ not edge_poses_
      edge.pose = gr::ToRosPose(ToGu(edge_poses_[uwb_edges_between_[ii]]));
      g.edges.push_back(edge);
    }

    // Publish.
    pose_graph_pub_.publish(g);
  }
  
  return true;
}

void LaserLoopClosure::PublishArtifacts(gtsam::Key artifact_key) {
  // For now, loop through artifact_key2label_hash
  // then publish. (might want to change this to an array later?)

  Eigen::Vector3d artifact_position;
  std::string artifact_label;
  bool b_publish_all = false;

  // Default input key is 'z0' if this is the case, publish all artifacts
  if (gtsam::Symbol(artifact_key).chr() == 'z') {
    b_publish_all = true;
    ROS_INFO("\n\n\t\tPublishing all artifacts\n\n");
  }

  ROS_INFO_STREAM("size of artifact key to info hash is " << artifact_key2info_hash_.size());

  // loop through values 
  for (auto it = artifact_key2info_hash_.begin();
            it != artifact_key2info_hash_.end(); it++ ) {

    ROS_INFO_STREAM("Artifact hash key is " << gtsam::DefaultKeyFormatter(it->first));
    gtsam::Symbol art_key = gtsam::Symbol(it->first);
    if (!(art_key.chr() == 'l' || art_key.chr() == 'm' || art_key.chr() == 'n' || art_key.chr() == 'o' || art_key.chr() == 'p' || art_key.chr() == 'q')){
      ROS_WARN("ERROR - have a non-landmark ID");
      ROS_INFO_STREAM("Bad ID is " << gtsam::DefaultKeyFormatter(it->first));
      continue;
    }

    if (b_publish_all) { // The default value
      // Update all artifacts - loop through all - the default
      // Get position and label 
      ROS_INFO_STREAM("Artifact key to publish is " << gtsam::DefaultKeyFormatter(it->first));
      artifact_position = GetArtifactPosition(gtsam::Key(it->first));
      artifact_label = it->second.msg.label;
      // Get the artifact key
      artifact_key = it->first;

      // Increment update count
      it->second.num_updates++;

      std::cout << "Number of updates of artifact is: "
                << it->second.num_updates << std::endl;

    }
    else{
      // Updating a single artifact - will return at the end of this first loop
      // Using the artifact key to publish that artifact
      ROS_INFO("Publishing only the new artifact");
      ROS_INFO_STREAM("Artifact key to publish is " << gtsam::DefaultKeyFormatter(artifact_key));

      // Check that the key exists
      if (artifact_key2info_hash_.count(artifact_key) == 0) {
        ROS_WARN("Artifact key is not in hash, nothing to publish");
        return;
      }

      // Get position and label 
      artifact_position = GetArtifactPosition(artifact_key);
      artifact_label = artifact_key2info_hash_[artifact_key].msg.label;
      // Keep the input artifact key

      // Increment update count
      artifact_key2info_hash_[artifact_key].num_updates++;

      std::cout << "Number of updates of artifact is: "
                << artifact_key2info_hash_[artifact_key].num_updates
                << std::endl;
    }

    // Check that the key exists
    if (artifact_key2info_hash_.count(artifact_key) == 0) {
      ROS_WARN("Artifact key is not in hash, nothing to publish");
      return;
    }

    // Fill artifact message
    core_msgs::Artifact new_msg = artifact_key2info_hash_[artifact_key].msg;

    if (b_publish_all){
      // Update the message ID
      new_msg.id = new_msg.id + std::to_string(it->second.num_updates-1);
      
      // Update the time after loop closure
      new_msg.header.stamp = ros::Time::now();
    }

    // Fill the new message positions
    new_msg.point.point.x = artifact_position[0];
    new_msg.point.point.y = artifact_position[1];
    new_msg.point.point.z = artifact_position[2];
    new_msg.point.header.frame_id = fixed_frame_id_;
    // Transform to world frame from map frame
    new_msg.point = tf_buffer_.transform(
        new_msg.point, "world", new_msg.point.header.stamp, "world");

    // Print out
    // Transform at time of message
    std::cout << "Artifact position in world is: " << new_msg.point.point.x
              << ", " << new_msg.point.point.y << ", " << new_msg.point.point.z
              << std::endl;
    std::cout << "Frame ID is: " << new_msg.point.header.frame_id << std::endl;

    std::cout << "\t Parent id: " << new_msg.parent_id << std::endl;
    std::cout << "\t Confidence: " << new_msg.confidence << std::endl;
    std::cout << "\t Position:\n[" << new_msg.point.point.x << ", "
              << new_msg.point.point.y << ", " << new_msg.point.point.z << "]"
              << std::endl;
    std::cout << "\t Label: " << new_msg.label << std::endl;

    // Publish
    artifact_pub_.publish(new_msg);

    if (!b_publish_all) {
      ROS_INFO("Single artifact - exiting artifact pub loop");
      // Only a single artifact - exit the loop 
      return;
    }
  }
}

gtsam::Key LaserLoopClosure::GetKeyAtTime(const ros::Time& stamp) const {
  // ROS_INFO("Get pose key closest to input time %f ", stamp.toSec());

  auto iterTime = stamps_keyed_.lower_bound(stamp.toSec()); // First key that is not less than timestamp 

  // std::cout << "Got iterator at lower_bound. Input: " << stamp.toSec() << ", found " << iterTime->first << std::endl;

  // TODO - interpolate - currently just take one
  double t2 = iterTime->first;

  if (iterTime == stamps_keyed_.begin()){
    ROS_WARN("Only one value in the graph - using that");
    return iterTime->second;
  }
  double t1 = std::prev(iterTime,1)->first; 

  // std::cout << "Time 1 is: " << t1 << ", Time 2 is: " << t2 << std::endl;

  gtsam::Symbol key;

  if (t2-stamp.toSec() < stamp.toSec() - t1) {
    // t2 is closer - use that key
    // std::cout << "Selecting later time: " << t2 << std::endl;
    key = iterTime->second;
  } else {
    // t1 is closer - use that key
    // std::cout << "Selecting earlier time: " << t1 << std::endl;
    key = std::prev(iterTime,1)->second;
    iterTime--;
  }
  // std::cout << "Key is: " << key << std::endl;
  if (iterTime == std::prev(stamps_keyed_.begin())){
    ROS_WARN("Invalid time for graph (before start of graph range). Choosing next value");
    iterTime++;
    // iterTime = stamps_keyed_.begin();
    key = iterTime->second;
  } else if(iterTime == stamps_keyed_.end()) {
    ROS_WARN("Invalid time for graph (past end of graph range). take latest pose");
    key = key_ -1;
  }

  return key; 
}

gu::Transform3 LaserLoopClosure::GetPoseAtKey(const gtsam::Key& key) const {
  // Get the pose at that key
  if (!values_.exists(key)) {
    ROS_WARN("Key does not exist in GetPoseAtKey");
    ROS_INFO_STREAM("Bad key is: " << gtsam::DefaultKeyFormatter(key));
    return gu::Transform3();
  }
  return ToGu(values_.at<Pose3>(key));
}

Eigen::Vector3d LaserLoopClosure::GetArtifactPosition(const gtsam::Key& artifact_key) const {
  if (!values_.exists(artifact_key)){
    ROS_WARN("Key does not exist in GetArtifactPosition");
    ROS_INFO_STREAM("Bad key is: " << gtsam::DefaultKeyFormatter(artifact_key));
    return Eigen::Vector3d();
  }
  return values_.at<Pose3>(artifact_key).translation().vector();
}

// Update loop edges based on iniers 
void LaserLoopClosure::updateInlierLoopEdges() {
  // first get the edges corresponding to each key 
  std::unordered_map < gtsam::Key, std::vector<Edge> > edges; 
  for (size_t i = 0; i < nfg_.size(); i++) {
    if (nfg_[i] != NULL && nfg_[i]->keys().size() == 2) {
      edges[nfg_[i]->front()].push_back(std::make_pair(nfg_[i]->front(), nfg_[i]->back()));
    }
  }
  // now update loop edges 
  inlier_loop_edges_.clear(); 
  for (size_t i  = 0; i < loop_edges_.size(); i++) {
    gtsam::Key front_key = loop_edges_[i].first; 
    if (edges.find(front_key) != edges.end()) {
      if (std::find(edges[front_key].begin(), edges[front_key].end(), 
          loop_edges_[i]) != edges[front_key].end()) {
        inlier_loop_edges_.push_back(loop_edges_[i]);
      }
    }
  }
}

//------------------Basestation functions:---------------------------------

void LaserLoopClosure::KeyedScanBaseHandler(
    const pose_graph_msgs::KeyedScan::ConstPtr& msg) {
  gtsam::Symbol key = gtsam::Symbol(msg->key);
  if (keyed_scans_.find(key) != keyed_scans_.end()) {
    ROS_ERROR("%s: Key %u already has a laser scan.", name_.c_str(), key);
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(msg->scan, *scan);

  // The first key should be treated differently; we need to use the laser
  // scan's timestamp for pose zero.
  if (key == 0) {
    const ros::Time stamp = pcl_conversions::fromPCL(scan->header.stamp);
    keyed_stamps_.insert(std::pair<gtsam::Symbol, ros::Time>(key, stamp));
    stamps_keyed_.insert(std::pair<double, gtsam::Symbol>(stamp.toSec(), key));
  }

  // ROS_INFO_STREAM("AddKeyScanPair " << key);

  // Add the key and scan.
  keyed_scans_.insert(
      std::pair<gtsam::Symbol, PointCloud::ConstPtr>(key, scan));
}

void LaserLoopClosure::PoseGraphBaseHandler(
    const pose_graph_msgs::PoseGraph::ConstPtr& msg,
    bool* found_loop) {
  ROS_INFO_STREAM("Loop closure pose_graph_processing");

  // Update graph to load with other (non odom) factors, if not already initialized
  if (values_recieved_at_base_.exists(initial_key_)){
    ROS_INFO("updating graph in PoseGraphBaseHandler from new factors");
    nfg_to_load_graph_ = nfg_;
    values_to_load_graph_ = values_;
  }

  // // Clear pgo_solver_
  // pgo_solver_.reset(new RobustPGO(pcm, SOLVER, special_symbs));
  // pgo_solver_->print();

  NonlinearFactorGraph new_factor;
  Values new_values;
  std::map<gtsam::Symbol, gtsam::Pose3> new_edge_to_transform;
  std::map<gtsam::Symbol, gtsam::Symbol> new_edge_to_parent_node;
  // PriorFactor<Pose3> prior_factor;

  ROS_INFO("\n\tAdding edges from pose-graph\n");
  // Add edges to basestation posegraph
  for (const auto &msg_edge : msg->edges) {
    gtsam::Point3 delta_translation(msg_edge.pose.position.x,
                                    msg_edge.pose.position.y,
                                    msg_edge.pose.position.z);
    gtsam::Rot3 delta_orientation(
        Rot3::quaternion(msg_edge.pose.orientation.w,
                         msg_edge.pose.orientation.x,
                         msg_edge.pose.orientation.y,
                         msg_edge.pose.orientation.z));
    gtsam::Pose3 delta = gtsam::Pose3(delta_orientation, delta_translation);

    // TODO! How do we want to get the covariance??? FIX SOON!!
    gu::MatrixNxNBase<double, 6> covariance;
    covariance.Zeros();
    for (int i = 0; i < 3; ++i)
      covariance(i, i) = 0.316 * 0.316; // 0.4, 0.004; 0.2 m sd
    for (int i = 3; i < 6; ++i)
      covariance(i, i) = 0.141 * 0.141; // 0.1, 0.01; sqrt(0.01) rad sd
    //-------------------------------------------------------------------------

    if (msg_edge.type == pose_graph_msgs::PoseGraphEdge::ODOM) {
      // Check if odometry_edge already exsists on basestation
      Edge incomming_edge = std::make_pair(msg_edge.key_from, msg_edge.key_to);
      bool b_edge_exists = false;
      for (size_t ii = 0; ii < odometry_edges_.size(); ++ii) {
        if (odometry_edges_[ii].first == msg_edge.key_from && odometry_edges_[ii].second == msg_edge.key_to){
          b_edge_exists = true;
        }
      }
      if (b_edge_exists){
        // ROS_INFO_STREAM("Odom edge already exists for key " << gtsam::DefaultKeyFormatter(msg_edge.key_from) << " to key " << gtsam::DefaultKeyFormatter(msg_edge.key_to));
        continue;
      }

      // Add edge to new edges
      new_edge_to_transform[gtsam::Symbol(msg_edge.key_to)] = delta;
      new_edge_to_parent_node[gtsam::Symbol(msg_edge.key_to)] = gtsam::Symbol(msg_edge.key_from);

      // Add to basestation posegraph
      ROS_INFO_STREAM("Adding Odom edge for key " << gtsam::DefaultKeyFormatter(msg_edge.key_from) << " to key " << gtsam::DefaultKeyFormatter(msg_edge.key_to));
      odometry_edges_.emplace_back(incomming_edge);
      new_factor.add(BetweenFactor<Pose3>(gtsam::Symbol(msg_edge.key_from),
                                          gtsam::Symbol(msg_edge.key_to),
                                          delta,
                                          ToGtsam(covariance)));

    } else if (msg_edge.type == pose_graph_msgs::PoseGraphEdge::LOOPCLOSE) {
      // Check if loop_edge already exsists on basestation
      Edge incomming_edge = std::make_pair(msg_edge.key_from, msg_edge.key_to);
      bool b_edge_exists = false;
      for (size_t ii = 0; ii < loop_edges_.size(); ++ii) {
        if (loop_edges_[ii] == incomming_edge)
          b_edge_exists = true;
      }
      if (b_edge_exists)
        continue;

      // Add to basestation posegraph
      loop_edges_.emplace_back(
          std::make_pair(msg_edge.key_from, msg_edge.key_to));
      new_factor.add(BetweenFactor<Pose3>(gtsam::Symbol(msg_edge.key_from),
                                          gtsam::Symbol(msg_edge.key_to),
                                          delta,
                                          ToGtsam(covariance)));
    } else if (msg_edge.type == pose_graph_msgs::PoseGraphEdge::ARTIFACT) {
      // Check if artifact_edge already exsists on basestation
      Edge incomming_edge = std::make_pair(msg_edge.key_from, msg_edge.key_to);
      bool b_edge_exists = false;
      for (size_t ii = 0; ii < artifact_edges_.size(); ++ii) {
        if (artifact_edges_[ii] == incomming_edge)
          b_edge_exists = true;
      }
      if (b_edge_exists)
        continue;

      // Add edge to new edges
      new_edge_to_transform[gtsam::Symbol(msg_edge.key_to)] = delta;
      new_edge_to_parent_node[gtsam::Symbol(msg_edge.key_to)] = gtsam::Symbol(msg_edge.key_from);

      // Add to basestation posegraph
      artifact_edges_.emplace_back(
          std::make_pair(msg_edge.key_from, msg_edge.key_to));
      new_factor.add(BetweenFactor<Pose3>(gtsam::Symbol(msg_edge.key_from),
                                          gtsam::Symbol(msg_edge.key_to),
                                          delta,
                                          ToGtsam(covariance)));

      // TODO include artifacts in the pose-graph
    } else if (msg_edge.type == pose_graph_msgs::PoseGraphEdge::UWB_RANGE) {
      // TODO send only incremental UWB edges (if msg.incremental is true)
      // uwb_edges_.clear();
      bool found = false;
      for (const auto& edge : uwb_edges_range_) {
        // cast to long unsigned int to ensure comparisons are correct
        if (edge.first == static_cast<gtsam::Symbol>(msg_edge.key_from) &&
            edge.second == static_cast<gtsam::Symbol>(msg_edge.key_to)) {
          found = true;
          ROS_DEBUG("PGV: UWB edge from %u to %u already exists.",
                    msg_edge.key_from,
                    msg_edge.key_to);
          break;
        }
      }
      // avoid duplicate UWB edges
      if (!found) {
        uwb_edges_range_.emplace_back(
            std::make_pair(static_cast<gtsam::Symbol>(msg_edge.key_from),
                           static_cast<gtsam::Symbol>(msg_edge.key_to)));
        ROS_INFO("PGV: Adding new UWB edge from %u to %u.",
                 msg_edge.key_from,
                 msg_edge.key_to);
        double range = msg_edge.range;
        double sigmaR = msg_edge.range_error;
        gtsam::noiseModel::Base::shared_ptr rangeNoise = gtsam::noiseModel::Isotropic::Sigma(1, sigmaR);

        new_factor.add(RangeFactor<Pose3, Pose3>(gtsam::Symbol(msg_edge.key_from),
                                          gtsam::Symbol(msg_edge.key_to),
                                          range,
                                          rangeNoise));
      }
    } else if (msg_edge.type == pose_graph_msgs::PoseGraphEdge::UWB_BETWEEN) {
      // TODO send only incremental UWB edges (if msg.incremental is true)
      // uwb_edges_.clear();
      bool found = false;
      for (const auto& edge : uwb_edges_between_) {
        // cast to long unsigned int to ensure comparisons are correct
        if (edge.first == static_cast<gtsam::Symbol>(msg_edge.key_from) &&
            edge.second == static_cast<gtsam::Symbol>(msg_edge.key_to)) {
          found = true;
          ROS_DEBUG("PGV: UWB edge from %u to %u already exists.",
                    msg_edge.key_from,
                    msg_edge.key_to);
          break;
        }
      }
      
      // Add edge to new edges
      new_edge_to_transform[gtsam::Symbol(msg_edge.key_to)] = delta;
      new_edge_to_parent_node[gtsam::Symbol(msg_edge.key_to)] = gtsam::Symbol(msg_edge.key_from);

      // avoid duplicate UWB edges
      if (!found) {
        uwb_edges_between_.emplace_back(
            std::make_pair(static_cast<gtsam::Symbol>(msg_edge.key_from),
                           static_cast<gtsam::Symbol>(msg_edge.key_to)));
        ROS_INFO("PGV: Adding new UWB edge from %u to %u.",
                 msg_edge.key_from,
                 msg_edge.key_to);
        
        // TODO! How do we want to get the covariance??? FIX SOON!!
        gu::MatrixNxNBase<double, 6> covariance_uwb;
        covariance_uwb.Zeros();
        for (int i = 0; i < 3; ++i)
          covariance_uwb(i, i) = 1000000; // rotation
        for (int i = 3; i < 6; ++i)
          covariance_uwb(i, i) = 0.25; // translation
        
        new_factor.add(BetweenFactor<Pose3>(gtsam::Symbol(msg_edge.key_from),
                                          gtsam::Symbol(msg_edge.key_to),
                                          delta,
                                          ToGtsam(covariance_uwb)));
        
      }
    }
  }

  //-----------------------------------------------//
  ROS_INFO("\n\tAdding new nodes\n");
  // Add the new nodes to base station posegraph
  for (const pose_graph_msgs::PoseGraphNode &msg_node : msg->nodes) {
    // Get key 
    key_ = gtsam::Symbol(msg_node.key);

    if (values_recieved_at_base_.exists(key_))
      continue;

    // Have a new node
    ROS_INFO("Have new node ");
    gtsam::Pose3 full_pose;

    // Check if the previous node exists 
    if (!values_recieved_at_base_.exists(new_edge_to_parent_node[key_])){
      ROS_INFO_STREAM("Node with key " << gtsam::DefaultKeyFormatter(new_edge_to_parent_node[key_]) << " that connects to new node at key " << gtsam::DefaultKeyFormatter(key_)  << " has not yet been received");

      ROS_WARN("Using pose from node message");
      gtsam::Point3 pose_translation(msg_node.pose.position.x,
                                   msg_node.pose.position.y,
                                   msg_node.pose.position.z);
      gtsam::Rot3 pose_orientation(Rot3::quaternion(msg_node.pose.orientation.w,
                                                    msg_node.pose.orientation.x,
                                                    msg_node.pose.orientation.y,
                                                    msg_node.pose.orientation.z));
      full_pose = gtsam::Pose3(pose_orientation, pose_translation);
    } else {
      ROS_INFO("Using edge to get pose");

      // Get the old pose from the graph 
      gtsam::Pose3 from_pose;

      bool b_build_pose_from_edge = true;

      if (!values_.exists(new_edge_to_parent_node[key_])){
        ROS_WARN("Key doesn't exist in values_");
        ROS_INFO_STREAM("Node with key " << gtsam::DefaultKeyFormatter(new_edge_to_parent_node[key_]) << " that connects to new node at key " << gtsam::DefaultKeyFormatter(key_)  << " has not yet been received in values_");

        ROS_INFO("Checking new_values");
        
        if (!new_values.exists(new_edge_to_parent_node[key_])){
          ROS_WARN("Key doesn't exist in new_values, using pose from node message");
          gtsam::Point3 pose_translation(msg_node.pose.position.x,
                                   msg_node.pose.position.y,
                                   msg_node.pose.position.z);
          gtsam::Rot3 pose_orientation(Rot3::quaternion(msg_node.pose.orientation.w,
                                                        msg_node.pose.orientation.x,
                                                        msg_node.pose.orientation.y,
                                                        msg_node.pose.orientation.z));
          full_pose = gtsam::Pose3(pose_orientation, pose_translation);

          b_build_pose_from_edge = false;

        } else {
          ROS_INFO_STREAM("New values has a pose at key " << gtsam::DefaultKeyFormatter(new_edge_to_parent_node[key_]));
          from_pose = new_values.at<Pose3>(new_edge_to_parent_node[key_]);
        }
      } else {
        ROS_INFO("Getting from pose from values_");
        from_pose = values_.at<Pose3>(new_edge_to_parent_node[key_]);
      }

      if (b_build_pose_from_edge){
        // Get the delta
        gtsam::Pose3 delta = new_edge_to_transform[key_];

        // Compose the delta
        full_pose = from_pose.compose(delta);
      }
      // TODO - this only handles the odometry case - not artifacts or UWB etc. 
    }

    // Check input for NaNs

    // Add UUID if an artifact node
    if (key_.chr() == 'l' || key_.chr() == 'm' || key_.chr() == 'n' || key_.chr() == 'o' || key_.chr() == 'p' || key_.chr() == 'q') {
      // Artifact
      // artifact_key2info_hash_[key_] = msg_node.ID;
      std::cout << "\t Artifact added to basestaion(PGcallback): "
                << gtsam::DefaultKeyFormatter(key_);
      std::cout << "\t with parent id: " << msg_node.ID << std::endl;
      new_values.insert(key_, full_pose);
      values_recieved_at_base_.insert(key_, full_pose);
      // Add time and ID to the artifact info
      ArtifactInfo info;
      info.msg.header.stamp = msg_node.header.stamp;
      info.msg.parent_id = msg_node.ID;
      artifact_key2info_hash_.insert(std::pair<gtsam::Symbol, ArtifactInfo>(key_, info));
      continue;
    } else if (key_.chr() == 'u') {
      // Process UWB  
      // Artifact
      std::cout << "\t UWB added to PGcallback: "
                << gtsam::DefaultKeyFormatter(key_);
      std::cout << "\t with id: " << msg_node.ID << std::endl;
      new_values.insert(key_, full_pose);
      values_recieved_at_base_.insert(key_, full_pose);
      // Add ID to the uwb hash
      uwb_key2id_hash_[key_] = msg_node.ID;
      continue;
    } else {
      // A node or keyframe - just add to keyed stamps
      // Add time to each node
      keyed_stamps_.insert(
          std::pair<gtsam::Symbol, ros::Time>(key_, msg_node.header.stamp));
      stamps_keyed_.insert(
          std::pair<double, gtsam::Symbol>(msg_node.header.stamp.toSec(), key_));
    }

    // if previous key exists
    if (values_recieved_at_base_.exists(key_ - 1)) {
      ROS_INFO_STREAM("Previous key exists, no prior for key " << gtsam::DefaultKeyFormatter(key_) );
      new_values.insert(key_, full_pose);
      values_recieved_at_base_.insert(key_, full_pose);
    }
    // is initial key
    else {
      ROS_INFO_STREAM("Adding prior in pose graph callback for key " << gtsam::DefaultKeyFormatter(key_) );
      LaserLoopClosure::Diagonal::shared_ptr covariance(
          LaserLoopClosure::Diagonal::Sigmas(initial_noise_));

      new_values.insert(key_, full_pose);
      values_recieved_at_base_.insert(key_, full_pose);

      // Set the first key if not already set - use this for adding between factors at start @AlexH
      if (!b_first_key_set_){
        initial_key_ = key_;
        b_first_key_set_ = true;
        prior_factor_ = MakePriorFactor(full_pose, covariance);
      } else {
        // Add betweenfactor between the first poses of the two different graphs
        // Calculate distance between the poses
        // In the case of multiple maps merging together
        gu::Transform3 init_pose =
            ToGu(values_recieved_at_base_.at<Pose3>(initial_key_));
        gu::Transform3 current_pose =
            ToGu(values_recieved_at_base_.at<Pose3>(key_));
        const gu::Transform3 pose_delta =
            gu::PoseDelta(init_pose, current_pose);

        new_factor.add(MakePriorFactor(full_pose, covariance));

        // TODO! How do we want to get the covariance??? FIX SOON!!
        gu::MatrixNxNBase<double, 6> covariance;
        covariance.Zeros();
        for (int i = 0; i < 3; ++i)
          covariance(i, i) = 0.316 * 0.316; // 0.4, 0.004; 0.2 m sd
        for (int i = 3; i < 6; ++i)
          covariance(i, i) = 0.141 * 0.141; // 0.1, 0.01; sqrt(0.01) rad sd
        //-------------------------------------------------------------------------

        between_initial_factors_ = BetweenFactor<Pose3>(
            initial_key_, key_, ToGtsam(pose_delta), ToGtsam(covariance));
      }
    }

  }

  if (new_factor.size() == 1 && new_values.size() == 1 && key_.chr() == initial_key_.chr()){
    // Only an odom addition, just run update
    ROS_INFO("\n\nAdding just an odom factor - no opt\n\n");
    pgo_solver_->update(new_factor, new_values);
  } 
  else {
    // Full graph or new graph
    ROS_INFO("\n\nAdding full graph - opt\n\n");
    if (key_.chr() == initial_key_.chr()) {
      nfg_to_load_graph_.add(new_factor);
      values_to_load_graph_.insert(new_values);
    } else {
      ROS_INFO_STREAM("it came to the end");
      values_to_add_graph_.insert(new_values);
      nfg_to_add_graph_.add(new_factor);
    }

    // RobustPGO should include load multiplemaps functions, but for now we first
    // load then add the other graph
    pgo_solver_.reset(new RobustPGO::RobustSolver(rpgo_params_));
    ROS_INFO("Pre pgo_solver print");
    pgo_solver_->print();
    // nfg_to_load_graph_.print("NFGload");
    // nfg_to_add_graph_.print("NFGadd");

    // Update
    ROS_INFO("Pre loadGraph");
    try {
      pgo_solver_->loadGraph(
          nfg_to_load_graph_,
          values_to_load_graph_,
          prior_factor_); // Add full graph and the prior separately
      has_changed_ = true;
    } catch (...) {
      ROS_ERROR("PGO Solver update error in AddBetweenFactors");
      throw;
    }

    if (!values_to_add_graph_.empty()) {
      gtsam::Symbol other_intial_key(key_.chr(), 0);
      pgo_solver_->addGraph(
          nfg_to_add_graph_,
          values_to_add_graph_,
          between_initial_factors_); // Add full graph to an existing graph
    }

  }


  // Get all values and nfg to laser loop closure
  values_ = pgo_solver_->calculateEstimate();
  nfg_ = pgo_solver_->getFactorsUnsafe();
  
  // values_.print("from RobustPGO values");
  // nfg_.print("from RobustPGO nfg");
  // Update key for getting the latest pose
  key_ = key_ + 1;

  // Run loop closures
  *found_loop = false;
  std::vector<gtsam::Symbol> closure_keys;
  if (FindLoopClosures(key_ - 1, &closure_keys)){
    ROS_INFO("Found loop closures after pose graph callback ");
    *found_loop = true;
  }

  nfg_to_load_graph_ = pgo_solver_->getFactorsUnsafe();
  values_to_load_graph_ = pgo_solver_->calculateEstimate();

  // publish posegraph
  has_changed_ = true;
  PublishPoseGraph();

  if (*found_loop){
    ROS_INFO("Found loop in poseGraphBaseHandler, publishing artifacts");
    PublishArtifacts();
  }
  ROS_INFO("Exiting poseGraphBaseHandler");
}

size_t LaserLoopClosure::GetNumberStampsKeyed() const {
  return stamps_keyed_.size();
}

ros::Time LaserLoopClosure::GetTimeAtLastKey() {
  if (keyed_stamps_.count(key_-1) != 0){
    return keyed_stamps_[key_-1];
  } else {
    return ros::Time::now();
  }
}

