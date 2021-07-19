/*
TestUtils.h
Author: Yun Chang
Some utility functions for offline testing
*/

#ifndef TEST_UTILS_H_
#define TEST_UTILS_H_

#include <string>

#include <pose_graph_msgs/KeyedScan.h>
#include <pose_graph_msgs/LoopCandidate.h>
#include <pose_graph_msgs/LoopCandidateArray.h>
#include <pose_graph_msgs/PoseGraph.h>
#include <pose_graph_msgs/PoseGraphEdge.h>
#include <utils/CommonStructs.h>

namespace test_utils {

struct TestData {
  pose_graph_msgs::LoopCandidateArray test_candidates_;
  std::vector<gtsam::Pose3> gt_keyed_poses_;
  std::vector<pose_graph_msgs::KeyedScan> keyed_scans_;
};

bool LoadExistingTestData(const std::string& file_path, TestData* data);

bool LoadLoopCandidateFromCsv(const std::string& candidate_file,
                              pose_graph_msgs::LoopCandidateArray* candidates);

bool LoadKeydPosesAndScansFromFile(
    const std::string& csv_file_path,
    const std::string& ptcld_dir_path,
    std::vector<gtsam::Pose3>* keyed_poses,
    std::vector<pose_graph_msgs::KeyedScan>* keyed_scans);

bool WriteLoopCandidatesToFile(
    const pose_graph_msgs::LoopCandidateArray& candidates,
    const std::string& output_file);

bool WriteKeyedPosesToFile(const std::vector<gtsam::Pose3>& keyed_poses,
                           const std::string& output_file);

bool WriteKeyedScansToFile(
    const std::vector<pose_graph_msgs::KeyedScan>& keyed_scans,
    const std::string& output_folder);

bool WriteTestDataToFile(const TestData& data, const std::string& output_dir);

} // namespace test_utils
#endif
