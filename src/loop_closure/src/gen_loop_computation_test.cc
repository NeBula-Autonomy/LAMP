/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */

#include <loop_closure/TestUtils.h>
#include <ros/ros.h>

namespace tu = test_utils;

int main(int argc, char** argv) {
  ros::init(argc, argv, "gen_loop_computation_test");
  ros::NodeHandle n("~");

  std::string dataset_path, gt_odom_bag, gt_odom_topic, lamp_bag, robot_name;
  n.getParam("dataset_path", dataset_path);
  n.getParam("gt_odom_bag", gt_odom_bag);
  n.getParam("gt_odom_topic", gt_odom_topic);
  n.getParam("lamp_bag", lamp_bag);
  n.getParam("robot_name",
             robot_name); // This restrict us to get only single robot loop
                          // closures to be in the dataset (TODO fix later)

  double radius_tol;
  n.getParam("radius_tol", radius_tol);

  tu::TestData test_data;
  // First try load existing (if exists)
  ROS_INFO("Checking for existing dataset...");
  if (LoadExistingTestData(dataset_path, &test_data))
    ROS_INFO("Loaded dataset with %d candidates. Adding new data on top of "
             "existing...",
             test_data.test_candidates_.candidates.size());

  // Then read the bags
  std::map<ros::Time, gtsam::Pose3> gt_pose_stamped;
  std::map<gtsam::Key, ros::Time> pg_keyed_stamps;
  std::map<gtsam::Key, pose_graph_msgs::KeyedScan> pg_keyed_scans;

  ROS_INFO("Reading ground truth trajectory from %s with topic %s",
           gt_odom_bag.c_str(),
           gt_odom_topic.c_str());

  if (!tu::ReadOdometryBagFile(gt_odom_bag, gt_odom_topic, &gt_pose_stamped)) {
    ROS_ERROR("Failed to read ground truth odometry. ");
    return EXIT_FAILURE;
  }

  ROS_INFO("Reading keyed scans and poses from %s", lamp_bag.c_str());

  if (!tu::ReadKeyedScansFromBagFile(
          lamp_bag, robot_name, &pg_keyed_stamps, &pg_keyed_scans)) {
    ROS_ERROR("Failed to read keyed scans and pose graph. ");
    return EXIT_FAILURE;
  }

  pose_graph_msgs::LoopCandidateArray new_candidates;
  std::map<gtsam::Key, gtsam::Pose3> pg_keyed_poses;
  tu::FindLoopCandidateFromGt(gt_pose_stamped,
                              pg_keyed_stamps,
                              pg_keyed_scans,
                              radius_tol,
                              10,
                              &new_candidates,
                              &pg_keyed_poses);

  // Add these newly found candidates
  if (!tu::AppendNewCandidates(
          new_candidates, pg_keyed_poses, pg_keyed_scans, &test_data)) {
    ROS_ERROR("Failed to append new candidates. ");
    return EXIT_FAILURE;
  }

  ROS_INFO("Writing %d candidates to dataset. ",
           test_data.test_candidates_.candidates.size());
  // Write to file
  if (!tu::WriteTestDataToFile(test_data, dataset_path)) {
    ROS_ERROR("Failed to write data to file. ");
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
