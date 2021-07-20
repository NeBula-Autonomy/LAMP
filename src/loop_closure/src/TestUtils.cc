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

namespace test_utils {

bool LoadExistingTestData(const std::string& file_path, TestData* data) {
  // First read loop candidate
  std::string lc_candidate_file = file_path + "/candidates.csv";
  if (!LoadLoopCandidateFromCsv(lc_candidate_file, &(data->test_candidates_))) {
    std::cout << "Failed to load loop candidates csv file. \n";
    return false;
  }

  // Then read the keyed poses and scans
  std::string keyed_poses_csv = file_path + "/keyed_poses.csv";
  if (!LoadKeydPosesAndScansFromFile(keyed_poses_csv,
                                     file_path,
                                     &(data->gt_keyed_poses_),
                                     &(data->keyed_scans_))) {
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
    std::vector<pose_graph_msgs::KeyedScan>* keyed_scans) {
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
    std::stringstream linestream(line);
    while (std::getline(linestream, entry, ',')) {
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
        gtsam::Rot3(transrot[7], transrot[4], transrot[5], transrot[6]),
        gtsam::Point3(transrot[1], transrot[2], transrot[3])));

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

bool WriteLoopCandidatesToFile(
    const pose_graph_msgs::LoopCandidateArray& candidates,
    const std::string& output_file) {
  std::ofstream csv_file(output_file);

  for (auto ct : candidates.candidates) {
    csv_file << ct.key_from << "," << ct.key_to << "\n";
  }
  // Close the file
  csv_file.close();
  return true;
}

bool WriteKeyedPosesToFile(const std::vector<gtsam::Pose3>& keyed_poses,
                           const std::string& output_file) {
  std::ofstream csv_file(output_file);

  for (size_t i = 0; i < keyed_poses.size(); i++) {
    gtsam::Pose3 pose = keyed_poses.at(i);
    gtsam::Point3 t = pose.translation();
    gtsam::Quaternion q = pose.rotation().toQuaternion();
    // x y z qx qy qz qw
    csv_file << i << "," << t.x() << "," << t.y() << "," << t.z() << ","
             << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << "\n";
  }

  // Close the file
  csv_file.close();
  return true;
}

bool WriteKeyedScansToFile(
    const std::vector<pose_graph_msgs::KeyedScan>& keyed_scans,
    const std::string& output_folder) {
  for (auto ks : keyed_scans) {
    pcl::PointCloud<Point>::Ptr scan(new pcl::PointCloud<Point>);
    pcl::fromROSMsg(ks.scan, *scan);
    std::string pcd_file =
        output_folder + "/" + std::to_string(ks.key) + ".pcd";
    pcl::io::savePCDFileASCII(pcd_file, *scan);
  }

  return true;
}

bool WriteTestDataToFile(const TestData& data, const std::string& output_dir) {
  std::string candidate_file = output_dir + "/candidates.csv";
  if (!WriteLoopCandidatesToFile(data.test_candidates_, candidate_file)) {
    std::cout << "Failed to write loop candidates to csv. \n";
    return false;
  }

  if (data.gt_keyed_poses_.size() != data.keyed_scans_.size()) {
    std::cout << "Number of keyed poses should correspond to the number of "
                 "keyed scans. \n";
    return false;
  }

  std::string poses_file = output_dir + "/keyed_poses.csv";
  if (!WriteKeyedPosesToFile(data.gt_keyed_poses_, poses_file)) {
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

bool ReadKeyedScansFromBagFile(
    const std::string& bag_file,
    const std::string& robot_name,
    std::map<gtsam::Key, ros::Time>* keyed_stamps,
    std::map<gtsam::Key, pose_graph_msgs::KeyedScan>* keyed_scans) {
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
      for (auto n : p->nodes)
        keyed_stamps->insert(
            std::pair<gtsam::Key, ros::Time>{n.key, n.header.stamp});
  }
  bag.close();

  if (keyed_stamps->size() != keyed_scans->size()) {
    std::cout << "Size of keyed stamps and keyed scans do not match. \n";
    return false;
  }

  if (keyed_stamps->size() == 0) {
    std::cout << "Read 0 keyed scans afrom bag file. \n";
    return false;
  }
  return true;
}

void FindLoopCandidateFromGt(
    const std::map<ros::Time, gtsam::Pose3>& gt_pose_stamped,
    const std::map<gtsam::Key, ros::Time>& keyed_stamps,
    const std::map<gtsam::Key, pose_graph_msgs::KeyedScan>& keyed_scans,
    const double& radius,
    const size_t& key_dist,
    pose_graph_msgs::LoopCandidateArray* candidates,
    std::map<gtsam::Key, gtsam::Pose3>* keyed_poses) {
  // First create gt keyed poses
  std::map<gtsam::Key, gtsam::Pose3> gt_keyed_poses;
  for (auto k : keyed_stamps) {
    gtsam::Pose3 kp;
    GetPoseAtTime(gt_pose_stamped, k.second, &kp);
    gt_keyed_poses[k.first] = kp;
  }
  *keyed_poses = gt_keyed_poses; // (TODO: should we use gt here or estimated?)

  // Now find loop closures
  for (auto m : gt_keyed_poses) {
    for (auto n : gt_keyed_poses) {
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

bool AppendNewCandidates(
    const pose_graph_msgs::LoopCandidateArray& candidates,
    const std::map<gtsam::Key, gtsam::Pose3>& candidate_keyed_poses,
    const std::map<gtsam::Key, pose_graph_msgs::KeyedScan>&
        candidate_keyed_scans,
    TestData* data) {
  if (candidate_keyed_scans.size() != candidate_keyed_poses.size()) {
    std::cout << "Number of candidate scans should match the number of "
                 "candidate keys. \n";
    return false;
  }

  std::map<gtsam::Key, size_t> reindexing;

  for (auto k : candidate_keyed_poses) {
    reindexing[k.first] = data->gt_keyed_poses_.size();
    data->gt_keyed_poses_.push_back(k.second);
  }

  for (auto k : candidate_keyed_scans) {
    pose_graph_msgs::KeyedScan new_scan = k.second;
    new_scan.key = reindexing[k.first];
    data->keyed_scans_.push_back(new_scan);
  }

  if (data->gt_keyed_poses_.size() != data->keyed_scans_.size()) {
    std::cout
        << "Number of keyed poses does not match the number of keyed scans. \n";
    return false;
  }

  for (auto c : candidates.candidates) {
    pose_graph_msgs::LoopCandidate new_c = c;
    new_c.key_from = reindexing[c.key_from];
    new_c.key_to = reindexing[c.key_to];
    data->test_candidates_.candidates.push_back(new_c);
  }
  return true;
}

} // namespace test_utils