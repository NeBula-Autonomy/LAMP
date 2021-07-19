/*
PointCloudUtils.cc
Author: Yun Chang
Some utility functions for wokring with Point Clouds
*/

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>

#include "loop_closure/TestUtils.h"

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
    int val;
    while (ss >> val) {
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
    pose_graph_msgs::LoopCandidate lc;
    // Extract each integer
    double val;
    std::vector<double> transrot; // x y z qx qy qz qw
    size_t key;
    while (ss >> val) {
      if (idx == 0) {
        key = static_cast<size_t>(val);
        if (key != line_num) {
          std::cout
              << "Key index does not match line number. Check file format. \n";
          return false;
        }
      } else if (idx > 1 && idx < 8) {
        transrot.push_back(val);
      }
      idx++;
    }

    // Create pose
    keyed_poses->push_back(gtsam::Pose3(
        gtsam::Rot3(transrot[6], transrot[3], transrot[4], transrot[5]),
        gtsam::Point3(transrot[0], transrot[1], transrot[2])));

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

  for (auto pose : keyed_poses) {
    gtsam::Point3 t = pose.translation();
    gtsam::Quaternion q = pose.rotation().toQuaternion();
    // x y z qx qy qz qw
    csv_file << t.x() << "," << t.y() << "," << t.z() << "," << q.x() << ","
             << q.y() << "," << q.z() << "," << q.w() << "\n";
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

} // namespace test_utils