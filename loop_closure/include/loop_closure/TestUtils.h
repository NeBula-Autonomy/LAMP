/*
TestUtils.h
Author: Yun Chang
Some utility functions for offline testing
*/

#ifndef TEST_UTILS_H_
#define TEST_UTILS_H_

#include <string>
#include <unordered_map>

#include <pose_graph_msgs/KeyedScan.h>
#include <pose_graph_msgs/LoopCandidate.h>
#include <pose_graph_msgs/LoopCandidateArray.h>
#include <pose_graph_msgs/PoseGraph.h>
#include <pose_graph_msgs/PoseGraphEdge.h>
#include <lamp_utils/CommonStructs.h>

namespace test_utils {

struct TestData {
  pose_graph_msgs::LoopCandidateArray real_candidates_;
  pose_graph_msgs::LoopCandidateArray fake_candidates_;
  std::vector<gtsam::Pose3> gt_keyed_poses_;
  std::vector<gtsam::Pose3> odom_keyed_poses_;
  std::vector<pose_graph_msgs::KeyedScan> keyed_scans_;
  std::vector<std::string> labels_;
};

bool LoadExistingTestData(const std::string& file_path, TestData* data);

bool LoadLoopCandidateFromCsv(const std::string& candidate_file,
                              pose_graph_msgs::LoopCandidateArray* candidates);

bool LoadKeydPosesAndScansFromFile(
    const std::string& csv_file_path,
    const std::string& ptcld_dir_path,
    std::vector<gtsam::Pose3>* keyed_poses,
    std::vector<pose_graph_msgs::KeyedScan>* keyed_scans,
    std::vector<std::string>* keyed_labels);

bool LoadKeydPosesFromFile(const std::string& csv_file_path,
                           std::vector<gtsam::Pose3>* keyed_poses);

bool WriteLoopCandidatesToFile(
    const pose_graph_msgs::LoopCandidateArray& candidates,
    const std::string& output_file);

bool WriteKeyedPosesToFile(const std::vector<gtsam::Pose3>& keyed_poses,
                           const std::vector<std::string>& keyed_labels,
                           const std::string& output_file);

bool WriteKeyedScansToFile(
    const std::vector<pose_graph_msgs::KeyedScan>& keyed_scans,
    const std::string& output_folder);

bool WriteTestDataToFile(const TestData& data, const std::string& output_dir);

bool GetPoseAtTime(const std::map<ros::Time, gtsam::Pose3>& pose_stamped,
                   const ros::Time& query_stamp,
                   gtsam::Pose3* result_pose,
                   const double& t_diff = 1e-2);

bool ReadOdometryBagFile(const std::string& bag_file,
                         const std::string& topic_name,
                         std::map<ros::Time, gtsam::Pose3>* pose_stamped);

bool ReadKeyedScansAndPosesFromBagFile(
    const std::string& bag_file,
    const std::string& robot_name,
    std::unordered_map<gtsam::Key, ros::Time>* keyed_stamps,
    std::map<gtsam::Key, gtsam::Pose3>* keyed_poses,
    std::unordered_map<gtsam::Key, pose_graph_msgs::KeyedScan>* keyed_scans);

void FindLoopCandidateFromGt(
    const std::map<char, std::map<ros::Time, gtsam::Pose3>>& gt_pose_stamped,
    const std::unordered_map<gtsam::Key, ros::Time>& keyed_stamps,
    const double& min_radius,
    const double& max_radius,
    const size_t& key_dist,
    pose_graph_msgs::LoopCandidateArray* candidates,
    std::map<gtsam::Key, gtsam::Pose3>* gt_keyed_poses);

void GenerateFalseLoopCandidateFromGt(
    const std::map<char, std::map<ros::Time, gtsam::Pose3>>& gt_pose_stamped,
    const std::unordered_map<gtsam::Key, ros::Time>& keyed_stamps,
    pose_graph_msgs::LoopCandidateArray* false_candidates);

bool AppendNewCandidates(
    const pose_graph_msgs::LoopCandidateArray& candidates,
    const pose_graph_msgs::LoopCandidateArray& other_candidates,
    const std::map<gtsam::Key, gtsam::Pose3>& candidate_keyed_poses,
    const std::map<gtsam::Key, gtsam::Pose3>& gt_keyed_poses,
    const std::unordered_map<gtsam::Key, pose_graph_msgs::KeyedScan>&
        candidate_keyed_scans,
    const std::string& label,
    TestData* data);

void OutputTestSummary(
    const TestData& data,
    const std::vector<pose_graph_msgs::PoseGraphEdge>& results,
    const std::vector<pose_graph_msgs::PoseGraphEdge>& false_results,
    const std::string& output_dir,
    const std::string& test_name);

} // namespace test_utils
#endif
