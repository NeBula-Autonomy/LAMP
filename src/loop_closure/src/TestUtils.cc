/*
PointCloudUtils.cc
Author: Yun Chang
Some utility functions for wokring with Point Clouds
*/

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "loop_closure/TestUtils.h"
#include <utils/CommonFunctions.h>
#include <utils/PrefixHandling.h>

namespace test_utils {

bool LoadExistingTestData(const std::string& file_path, TestData* data) {
  // First read loop candidate
  std::string lc_candidate_file = file_path + "/candidates.csv";
  if (!LoadLoopCandidateFromCsv(lc_candidate_file, &(data->real_candidates_))) {
    std::cout << "Failed to load loop candidates csv file. \n";
    return false;
  }

  std::string fake_lc_candidate_file = file_path + "/false_candidates.csv";
  if (!LoadLoopCandidateFromCsv(fake_lc_candidate_file,
                                &(data->fake_candidates_))) {
    std::cout << "Failed to load fake loop candidates csv file. \n";
    return false;
  }

  // Then read the keyed poses and scans
  std::string keyed_poses_csv = file_path + "/keyed_poses.csv";
  if (!LoadKeydPosesAndScansFromFile(keyed_poses_csv,
                                     file_path,
                                     &(data->odom_keyed_poses_),
                                     &(data->keyed_scans_),
                                     &(data->labels_))) {
    std::cout << "Failed to load keyed poses and scans. \n";
    return false;
  }

  std::string gt_keyed_poses_csv = file_path + "/gt_keyed_poses.csv";
  if (!LoadKeydPosesFromFile(gt_keyed_poses_csv, &(data->gt_keyed_poses_))) {
    std::cout << "Failed to load keyed poses and scans. \n";
    return false;
  }

  return true;
}

bool LoadLoopCandidateFromCsv(const std::string& candidate_file,
                              pose_graph_msgs::LoopCandidateArray* candidates) {
  // Create an input filestream
  std::ifstream csv_file(candidate_file);

  // Make sure the file is open
  if (!csv_file.is_open()) {
    std::cout << "Unable to open loop candidate csv file. \n";
    return false;
  }

  // Read candidates line by line
  std::string line;
  while (std::getline(csv_file, line)) {
    std::stringstream ss(line);
    // Keep track of the current column index
    int idx = 0;
    pose_graph_msgs::LoopCandidate lc;
    // Extract each integer
    std::stringstream linestream(line);
    std::string entry;
    while (std::getline(linestream, entry, ',')) {
      int val = std::stoi(entry);
      if (idx == 0) {
        lc.key_from = val;
      } else if (idx == 1) {
        lc.key_to = val;
      }
      idx++;
    }
    candidates->candidates.push_back(lc);
  }

  // Close file
  csv_file.close();
  return true;
}

bool LoadKeydPosesAndScansFromFile(
    const std::string& csv_file_path,
    const std::string& ptcld_dir_path,
    std::vector<gtsam::Pose3>* keyed_poses,
    std::vector<pose_graph_msgs::KeyedScan>* keyed_scans,
    std::vector<std::string>* keyed_labels) {
  // Make sure keyed poses and scans are empty
  keyed_poses->clear();
  keyed_scans->clear();
  // Create an input filestream
  std::ifstream csv_file(csv_file_path);

  // Make sure the file is open
  if (!csv_file.is_open()) {
    std::cout << "Unable to open keyed poses csv file \n";
    return false;
  }

  // Read keyed poses line by line
  std::string line;
  int line_num = 0;
  while (std::getline(csv_file, line)) {
    std::stringstream ss(line);
    // Keep track of the current column index
    int idx = 0;
    // Extract each integer
    std::string entry;
    std::vector<double> transrot; // x y z qx qy qz qw
    size_t key;
    std::string label;
    std::stringstream linestream(line);
    while (std::getline(linestream, entry, ',')) {
      if (idx == 8) {
        label = entry;
        continue;
      }
      double val = std::stod(entry);
      if (idx == 0) {
        key = static_cast<size_t>(val);
        if (key != line_num) {
          std::cout
              << "Key index does not match line number. Check file format. \n";
          return false;
        }
      } else if (idx > 0 && idx < 8) {
        transrot.push_back(val);
      }
      idx++;
    }

    // Create pose
    keyed_poses->push_back(gtsam::Pose3(
        gtsam::Rot3(transrot[6], transrot[3], transrot[4], transrot[5]),
        gtsam::Point3(transrot[0], transrot[1], transrot[2])));

    // Label
    keyed_labels->push_back(label);

    // Read point cloud
    std::string pcd_file = ptcld_dir_path + "/" + std::to_string(key) + ".pcd";
    PointCloud::Ptr cloud(new PointCloud);
    pose_graph_msgs::KeyedScan keyed_scan;
    if (pcl::io::loadPCDFile<Point>(pcd_file, *cloud) == -1) {
      std::cout << "Error reading " << pcd_file << std::endl;
      return false;
    }
    pcl::toROSMsg(*cloud, keyed_scan.scan);
    keyed_scan.key = key;
    keyed_scans->push_back(keyed_scan);

    line_num++;
  }
  return true;
}

bool LoadKeydPosesFromFile(const std::string& csv_file_path,
                           std::vector<gtsam::Pose3>* keyed_poses) {
  // Make sure keyed poses and scans are empty
  keyed_poses->clear();
  // Create an input filestream
  std::ifstream csv_file(csv_file_path);

  // Make sure the file is open
  if (!csv_file.is_open()) {
    std::cout << "Unable to open keyed poses csv file \n";
    return false;
  }

  // Read keyed poses line by line
  std::string line;
  int line_num = 0;
  while (std::getline(csv_file, line)) {
    std::stringstream ss(line);
    // Keep track of the current column index
    int idx = 0;
    // Extract each integer
    std::string entry;
    std::vector<double> transrot; // x y z qx qy qz qw
    size_t key;
    std::string label;
    std::stringstream linestream(line);
    while (std::getline(linestream, entry, ',')) {
      if (idx == 8) {
        label = entry;
        continue;
      }
      double val = std::stod(entry);
      if (idx == 0) {
        key = static_cast<size_t>(val);
        if (key != line_num) {
          std::cout
              << "Key index does not match line number. Check file format. \n";
          return false;
        }
      } else if (idx > 0 && idx < 8) {
        transrot.push_back(val);
      }
      idx++;
    }

    // Create pose
    keyed_poses->push_back(gtsam::Pose3(
        gtsam::Rot3(transrot[6], transrot[3], transrot[4], transrot[5]),
        gtsam::Point3(transrot[0], transrot[1], transrot[2])));

    line_num++;
  }
  return true;
}

bool WriteLoopCandidatesToFile(
    const pose_graph_msgs::LoopCandidateArray& candidates,
    const std::string& output_file) {
  std::ofstream csv_file(output_file);

  for (const auto& ct : candidates.candidates) {
    csv_file << ct.key_from << "," << ct.key_to << "\n";
  }
  // Close the file
  csv_file.close();
  return true;
}

bool WriteKeyedPosesToFile(const std::vector<gtsam::Pose3>& keyed_poses,
                           const std::vector<std::string>& keyed_labels,
                           const std::string& output_file) {
  std::ofstream csv_file(output_file);

  if (keyed_poses.size() != keyed_labels.size()) {
    std::cout << "Number of labels should match the number of poses. \n";
    return false;
  }

  for (size_t i = 0; i < keyed_poses.size(); i++) {
    gtsam::Pose3 pose = keyed_poses.at(i);
    gtsam::Point3 t = pose.translation();
    gtsam::Quaternion q = pose.rotation().toQuaternion();
    // x y z qx qy qz qw
    csv_file << i << "," << t.x() << "," << t.y() << "," << t.z() << ","
             << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << ","
             << keyed_labels.at(i) << "\n";
  }

  // Close the file
  csv_file.close();
  return true;
}

bool WriteKeyedScansToFile(
    const std::vector<pose_graph_msgs::KeyedScan>& keyed_scans,
    const std::string& output_folder) {
  for (const auto& ks : keyed_scans) {
    pcl::PointCloud<Point>::Ptr scan(new pcl::PointCloud<Point>);
    pcl::fromROSMsg(ks.scan, *scan);
    std::string pcd_file =
        output_folder + "/" + std::to_string(ks.key) + ".pcd";
    pcl::io::savePCDFileBinary(pcd_file, *scan);
  }

  return true;
}

bool WriteTestDataToFile(const TestData& data, const std::string& output_dir) {
  std::string candidate_file = output_dir + "/candidates.csv";
  if (!WriteLoopCandidatesToFile(data.real_candidates_, candidate_file)) {
    std::cout << "Failed to write loop candidates to csv. \n";
    return false;
  }

  std::string false_candidate_file = output_dir + "/false_candidates.csv";
  if (!WriteLoopCandidatesToFile(data.fake_candidates_, false_candidate_file)) {
    std::cout << "Failed to write false loop candidates to csv. \n";
    return false;
  }

  if (data.gt_keyed_poses_.size() != data.keyed_scans_.size()) {
    std::cout << "Number of keyed poses should correspond to the number of "
                 "keyed scans. \n";
    return false;
  }

  std::string gt_poses_file = output_dir + "/gt_keyed_poses.csv";
  if (!WriteKeyedPosesToFile(
          data.gt_keyed_poses_, data.labels_, gt_poses_file)) {
    std::cout << "Failed to save gt keyed poses. \n";
    return false;
  }

  std::string poses_file = output_dir + "/keyed_poses.csv";
  if (!WriteKeyedPosesToFile(
          data.odom_keyed_poses_, data.labels_, poses_file)) {
    std::cout << "Failed to save keyed poses. \n";
    return false;
  }

  if (!WriteKeyedScansToFile(data.keyed_scans_, output_dir)) {
    std::cout << "Failed to save keyed scans to file. \n";
    return false;
  }

  return true;
}

bool GetPoseAtTime(const std::map<ros::Time, gtsam::Pose3>& pose_stamped,
                   const ros::Time& query_stamp,
                   gtsam::Pose3* result_pose,
                   const double& t_diff) {
  std::map<ros::Time, gtsam::Pose3>::const_iterator low, prev;
  low = pose_stamped.lower_bound(query_stamp);
  ros::Time result_stamp;
  if (low == pose_stamped.end()) {
    *result_pose = pose_stamped.rbegin()->second;
    result_stamp = pose_stamped.rbegin()->first;
  } else if (low == pose_stamped.begin()) {
    *result_pose = low->second;
    result_stamp = low->first;
  } else {
    prev = std::prev(low);
    if ((query_stamp - prev->first) < (low->first - query_stamp)) {
      *result_pose = prev->second;
      result_stamp = prev->first;
    } else {
      *result_pose = low->second;
      result_stamp = low->first;
    }
  }

  if (abs((result_stamp - query_stamp).toSec()) > t_diff) {
    std::cout << "Exceed tolerence in GetPoseAtTime: "
              << abs((result_stamp - query_stamp).toSec()) << std::endl;
    return false;
  }
  return true;
}

bool ReadOdometryBagFile(const std::string& bag_file,
                         const std::string& topic_name,
                         std::map<ros::Time, gtsam::Pose3>* pose_stamped) {
  rosbag::Bag bag;
  bag.open(bag_file);

  std::vector<std::string> topics;
  topics.push_back(topic_name);

  for (rosbag::MessageInstance const m :
       rosbag::View(bag, rosbag::TopicQuery(topics))) {
    nav_msgs::Odometry::ConstPtr i = m.instantiate<nav_msgs::Odometry>();
    if (i != nullptr)
      pose_stamped->insert(std::pair<ros::Time, gtsam::Pose3>{
          i->header.stamp, utils::ToGtsam(i->pose.pose)});
  }
  bag.close();
  if (pose_stamped->size() == 0) {
    std::cout << "Read 0 odometry poses. \n";
    return false;
  }
  return true;
}

bool ReadKeyedScansAndPosesFromBagFile(
    const std::string& bag_file,
    const std::string& robot_name,
    std::unordered_map<gtsam::Key, ros::Time>* keyed_stamps,
    std::map<gtsam::Key, gtsam::Pose3>* keyed_poses,
    std::unordered_map<gtsam::Key, pose_graph_msgs::KeyedScan>* keyed_scans) {
  rosbag::Bag bag;
  bag.open(bag_file);

  std::vector<std::string> topics;
  topics.push_back("/" + robot_name + "/lamp/keyed_scans");
  topics.push_back("/" + robot_name + "/lamp/pose_graph_incremental");

  for (rosbag::MessageInstance const m :
       rosbag::View(bag, rosbag::TopicQuery(topics))) {
    pose_graph_msgs::KeyedScan::ConstPtr i =
        m.instantiate<pose_graph_msgs::KeyedScan>();
    if (i != nullptr)
      keyed_scans->insert(
          std::pair<gtsam::Key, pose_graph_msgs::KeyedScan>{i->key, *i});
    pose_graph_msgs::PoseGraph::ConstPtr p =
        m.instantiate<pose_graph_msgs::PoseGraph>();
    if (p != nullptr)
      for (const auto& n : p->nodes) {
        keyed_poses->insert(
            std::pair<gtsam::Key, gtsam::Pose3>{n.key, utils::ToGtsam(n.pose)});
        keyed_stamps->insert(
            std::pair<gtsam::Key, ros::Time>{n.key, n.header.stamp});
      }
  }
  bag.close();

  if (keyed_stamps->size() != keyed_scans->size()) {
    std::cout << "Size of keyed stamps and keyed scans do not match. "
              << keyed_stamps->size() << " vs. " << keyed_scans->size() << "\n";
    // Delete
    if (keyed_stamps->size() > keyed_scans->size()) {
      for (const auto& ks : *keyed_stamps) {
        if (keyed_scans->find(ks.first) == keyed_scans->end()) {
          keyed_poses->erase(ks.first);
          keyed_stamps->erase(ks.first);
        }
      }
    } else {
      for (const auto& ks : *keyed_scans) {
        if (keyed_stamps->find(ks.first) == keyed_stamps->end()) {
          keyed_scans->erase(ks.first);
        }
      }
    }
  }

  if (keyed_stamps->size() == 0) {
    std::cout << "Read 0 keyed scans afrom bag file. \n";
    return false;
  }
  return true;
}

void FindLoopCandidateFromGt(
    const std::map<char, std::map<ros::Time, gtsam::Pose3>>& gt_pose_stamped,
    const std::unordered_map<gtsam::Key, ros::Time>& keyed_stamps,
    const double& radius,
    const size_t& key_dist,
    pose_graph_msgs::LoopCandidateArray* candidates,
    std::map<gtsam::Key, gtsam::Pose3>* gt_keyed_poses) {
  // First create gt keyed poses
  for (const auto& k : keyed_stamps) {
    gtsam::Pose3 kp;
    char prefix = gtsam::Symbol(k.first).chr();
    GetPoseAtTime(gt_pose_stamped.at(prefix), k.second, &kp);
    gt_keyed_poses->insert(std::pair<gtsam::Key, gtsam::Pose3>{k.first, kp});
  }

  // Now find loop closures
  for (const auto& m : *gt_keyed_poses) {
    for (const auto& n : *gt_keyed_poses) {
      if (m.first - n.first > key_dist) {
        if ((m.second.translation() - n.second.translation()).norm() < radius) {
          // Create loop closure
          pose_graph_msgs::LoopCandidate cand;
          cand.key_from = m.first;
          cand.key_to = n.first;
          candidates->candidates.push_back(cand);
        }
      }

      if (n.first > (m.first - key_dist))
        break;
    }
  }
}

void GenerateFalseLoopCandidateFromGt(
    const std::map<char, std::map<ros::Time, gtsam::Pose3>>& gt_pose_stamped,
    const std::unordered_map<gtsam::Key, ros::Time>& keyed_stamps,
    pose_graph_msgs::LoopCandidateArray* false_candidates) {
  size_t n = 500;
  while (false_candidates->candidates.size() < n) {
    auto it = keyed_stamps.begin();
    std::advance(it, rand() % keyed_stamps.size());
    gtsam::Key key_1 = it->first;
    it = keyed_stamps.begin();
    std::advance(it, rand() % keyed_stamps.size());
    gtsam::Key key_2 = it->first;
    gtsam::Pose3 p1, p2;
    char c1 = gtsam::Symbol(key_1).chr();
    char c2 = gtsam::Symbol(key_2).chr();
    GetPoseAtTime(gt_pose_stamped.at(c1), keyed_stamps.at(key_1), &p1);
    GetPoseAtTime(gt_pose_stamped.at(c2), keyed_stamps.at(key_2), &p2);
    if ((p1.translation() - p2.translation()).norm() > 10) {
      pose_graph_msgs::LoopCandidate cand;
      cand.key_from = key_1;
      cand.key_to = key_2;
      false_candidates->candidates.push_back(cand);
    }
  }
}

bool AppendNewCandidates(
    const pose_graph_msgs::LoopCandidateArray& candidates,
    const pose_graph_msgs::LoopCandidateArray& other_candidates,
    const std::map<gtsam::Key, gtsam::Pose3>& candidate_keyed_poses,
    const std::map<gtsam::Key, gtsam::Pose3>& gt_keyed_poses,
    const std::unordered_map<gtsam::Key, pose_graph_msgs::KeyedScan>&
        candidate_keyed_scans,
    const std::string& label,
    TestData* data) {
  if (candidate_keyed_scans.size() != candidate_keyed_poses.size()) {
    std::cout << "Number of candidate scans should match the number of "
                 "candidate keys. "
              << candidate_keyed_scans.size() << " vs. "
              << candidate_keyed_poses.size() << std::endl;
    return false;
  }

  std::unordered_map<gtsam::Key, size_t> reindexing;

  for (const auto& k : candidate_keyed_poses) {
    reindexing[k.first] = data->gt_keyed_poses_.size();
    data->odom_keyed_poses_.push_back(k.second);
    data->gt_keyed_poses_.push_back(gt_keyed_poses.at(k.first));
    data->labels_.push_back(label);
  }

  for (const auto& k : candidate_keyed_scans) {
    pose_graph_msgs::KeyedScan new_scan = k.second;
    new_scan.key = reindexing[k.first];
    data->keyed_scans_.push_back(new_scan);
  }

  if (data->gt_keyed_poses_.size() != data->keyed_scans_.size()) {
    std::cout
        << "Number of keyed poses does not match the number of keyed scans. \n";
    return false;
  }

  for (const auto& c : candidates.candidates) {
    pose_graph_msgs::LoopCandidate new_c = c;
    new_c.key_from = reindexing[c.key_from];
    new_c.key_to = reindexing[c.key_to];
    data->real_candidates_.candidates.push_back(new_c);
  }

  for (const auto& c : other_candidates.candidates) {
    pose_graph_msgs::LoopCandidate new_c = c;
    new_c.key_from = reindexing[c.key_from];
    new_c.key_to = reindexing[c.key_to];
    data->fake_candidates_.candidates.push_back(new_c);
  }
  return true;
}

void OutputTestSummary(
    const TestData& data,
    const std::vector<pose_graph_msgs::PoseGraphEdge>& results,
    const std::vector<pose_graph_msgs::PoseGraphEdge>& false_results,
    const std::string& output_dir,
    const std::string& test_name) {
  // First find number of candidates by label
  std::unordered_map<std::string, size_t> expected_num_lc;
  for (const auto& c : data.real_candidates_.candidates) {
    std::string label = data.labels_.at(c.key_from);
    if (expected_num_lc.find(label) == expected_num_lc.end()) {
      expected_num_lc[label] = 1;
    } else {
      expected_num_lc[label]++;
    }
  }

  std::unordered_map<std::string, size_t> false_num_lc;
  for (const auto& c : data.fake_candidates_.candidates) {
    std::string label = data.labels_.at(c.key_from);
    if (false_num_lc.find(label) == false_num_lc.end()) {
      false_num_lc[label] = 1;
    } else {
      false_num_lc[label]++;
    }
  }

  // Now evaluate
  std::unordered_map<std::string, size_t> num_lc;
  std::unordered_map<std::string, size_t> num_false_lc;
  std::unordered_map<std::string, double> total_trans_error;
  std::unordered_map<std::string, double> total_rot_error;
  std::unordered_map<std::string, double> min_trans_error;
  std::unordered_map<std::string, double> max_trans_error;
  std::unordered_map<std::string, double> min_rot_error;
  std::unordered_map<std::string, double> max_rot_error;
  std::unordered_map<std::string, std::vector<double>> trans_error_raw;
  std::unordered_map<std::string, std::vector<double>> rot_error_raw;
  std::vector<std::pair<double, double>> fit_to_trans_err;
  std::vector<std::pair<double, double>> fit_to_rot_err;
  std::vector<std::pair<double, double>> fit_to_trans_err_fp;
  std::vector<std::pair<double, double>> fit_to_rot_err_fp;

  for (const auto& enl : expected_num_lc) {
    num_lc[enl.first] = 0;
    total_trans_error[enl.first] = 0;
    total_rot_error[enl.first] = 0;
    min_trans_error[enl.first] = std::numeric_limits<double>::max();
    max_trans_error[enl.first] = 0;
    min_rot_error[enl.first] = std::numeric_limits<double>::max();
    max_rot_error[enl.first] = 0;
    trans_error_raw[enl.first] = std::vector<double>();
    rot_error_raw[enl.first] = std::vector<double>();
  }

  for (const auto& edge : results) {
    std::string label = data.labels_.at(edge.key_from);
    num_lc[label]++;

    gtsam::Pose3 transform = utils::ToGtsam(edge.pose);
    gtsam::Pose3 gt_transform =
        data.gt_keyed_poses_.at(edge.key_from)
            .between(data.gt_keyed_poses_.at(edge.key_to));
    gtsam::Pose3 error_pose3 = transform.between(gt_transform);
    gtsam::Vector error_log = gtsam::Pose3::Logmap(error_pose3);

    double trans_error =
        std::sqrt(error_log.tail(3).transpose() * error_log.tail(3));
    double rot_error =
        std::sqrt(error_log.head(3).transpose() * error_log.head(3));

    // Populate stats
    fit_to_trans_err.push_back(
        std::pair<double, double>{edge.range_error, trans_error});
    fit_to_rot_err.push_back(
        std::pair<double, double>{edge.range_error, rot_error});
    total_trans_error[label] += trans_error;
    total_rot_error[label] += rot_error;
    trans_error_raw[label].push_back(trans_error);
    rot_error_raw[label].push_back(rot_error);

    if (trans_error < min_trans_error[label])
      min_trans_error[label] = trans_error;
    if (trans_error > max_trans_error[label])
      max_trans_error[label] = trans_error;
    if (rot_error < min_rot_error[label])
      min_rot_error[label] = rot_error;
    if (rot_error > max_rot_error[label])
      max_rot_error[label] = rot_error;
  }

  for (const auto& edge : false_results) {
    std::string label = data.labels_.at(edge.key_from);
    num_false_lc[label]++;

    gtsam::Pose3 transform = utils::ToGtsam(edge.pose);
    gtsam::Pose3 gt_transform =
        data.gt_keyed_poses_.at(edge.key_from)
            .between(data.gt_keyed_poses_.at(edge.key_to));
    gtsam::Pose3 error_pose3 = transform.between(gt_transform);
    gtsam::Vector error_log = gtsam::Pose3::Logmap(error_pose3);

    double trans_error =
        std::sqrt(error_log.tail(3).transpose() * error_log.tail(3));
    double rot_error =
        std::sqrt(error_log.head(3).transpose() * error_log.head(3));

    // Track fitness and false positives
    fit_to_trans_err_fp.push_back(
        std::pair<double, double>{edge.range_error, trans_error});
    fit_to_rot_err_fp.push_back(
        std::pair<double, double>{edge.range_error, rot_error});
  }

  // Write to file
  std::ofstream outfile;
  std::string summary_file = output_dir + "/" + test_name + "_summary.csv";
  outfile.open(summary_file);
  if (!outfile.is_open()) {
    std::cout << "Unable to open output result summary file. \n";
    return;
  }
  outfile << "label,expected,detected,recall,false-positive-rate,mean-trans-"
             "error(m),min-"
             "trans-error(m),max-trans-error(m),mean-rot-error(deg),min-rot-"
             "error(deg),max-rot-error(deg)\n";
  for (const auto& entry : expected_num_lc) {
    outfile << entry.first << "," << entry.second << "," << num_lc[entry.first]
            << ","
            << static_cast<double>(num_lc[entry.first]) /
            static_cast<double>(entry.second)
            << ","
            << static_cast<double>(num_false_lc[entry.first]) /
            static_cast<double>(false_num_lc[entry.first])
            << "," << total_trans_error[entry.first] / num_lc[entry.first]
            << "," << min_trans_error[entry.first] << ","
            << max_trans_error[entry.first] << ","
            << (total_rot_error[entry.first] / num_lc[entry.first]) * 180 /
            3.1416
            << "," << (min_rot_error[entry.first]) * 180 / 3.1416 << ","
            << (max_rot_error[entry.first]) * 180 / 3.1416 << "\n";
  }
  outfile.close();

  // Write fitness score and error to file
  std::ofstream statfile;
  std::string stat_file = output_dir + "/" + test_name + "_fitness.csv";
  statfile.open(stat_file);
  if (!statfile.is_open()) {
    std::cout << "Unable to open output fitness error file. \n";
    return;
  }
  statfile << "fitness,trans-error(m),rot-error(deg)\n";
  for (size_t i = 0; i < fit_to_trans_err.size(); i++) {
    statfile << fit_to_trans_err[i].first << "," << fit_to_trans_err[i].second
             << "," << fit_to_rot_err[i].second * 180 / 3.1416 << "\n";
  }
  statfile.close();

  // Write fitness score and error to file for the false positives
  std::ofstream fp_statfile;
  std::string fp_stat_file = output_dir + "/" + test_name + "_fp_fitness.csv";
  fp_statfile.open(fp_stat_file);
  if (!fp_statfile.is_open()) {
    std::cout << "Unable to open output false positive fitness error file. \n";
    return;
  }
  fp_statfile << "fitness,trans-error(m),rot-error(deg)\n";
  for (size_t i = 0; i < fit_to_trans_err_fp.size(); i++) {
    fp_statfile << fit_to_trans_err_fp[i].first << ","
                << fit_to_trans_err_fp[i].second << ","
                << fit_to_rot_err_fp[i].second * 180 / 3.1416 << "\n";
  }
  fp_statfile.close();
  // Write translation and rotation error to file
  std::ofstream errorfile;
  std::string error_file = output_dir + "/" + test_name + "_error.csv";
  errorfile.open(error_file);
  if (!errorfile.is_open()) {
    std::cout << "Unable to open output false positive fitness error file. \n";
    return;
  }
  errorfile << "label,trans-error(m),rot-error(deg)\n";
  for (const auto& label_error : trans_error_raw) {
    for (size_t i = 0; i < label_error.second.size(); i++) {
      errorfile << label_error.first << ","
                << trans_error_raw[label_error.first][i] << ","
                << rot_error_raw[label_error.first][i] * 180 / 3.1416 << "\n";
    }
  }
  errorfile.close();
}

} // namespace test_utils