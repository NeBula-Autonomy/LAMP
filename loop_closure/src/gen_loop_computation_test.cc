/*
 * Copyright Notes
 *
 * Authors: Yun Chang (yunchang@mit.edu)
 */
#include <string>

#include <loop_closure/TestUtils.h>
#include <ros/ros.h>
#include <utils/PrefixHandling.h>

namespace tu = test_utils;

using PoseStamped = std::map<ros::Time, gtsam::Pose3>;

int main(int argc, char** argv) {
  ros::init(argc, argv, "gen_loop_computation_test");
  ros::NodeHandle n("~");

  std::string dataset_path, gt_path, lamp_bag_path, label;
  n.getParam("dataset_path", dataset_path);
  n.getParam("gt_path", gt_path);
  n.getParam("lamp_bag_path", lamp_bag_path);
  n.getParam("label", label);

  std::vector<std::string> robot_names;
  n.getParam("robots", robot_names);
  if (robot_names.size() == 0) {
    ROS_ERROR("Must specify array of robot names in param 'robots'");
    return EXIT_FAILURE;
  }

  double radius_tol, min_radius;
  n.getParam("radius_tol", radius_tol);
  n.getParam("min_radius", min_radius);

  tu::TestData test_data;
  // First try load existing (if exists)
  ROS_INFO("Checking for existing dataset...");
  if (LoadExistingTestData(dataset_path, &test_data))
    ROS_INFO("Loaded dataset with %d candidates. Adding new data on top of "
             "existing...",
             test_data.real_candidates_.candidates.size());

  // Then read the bags
  std::map<char, PoseStamped> gt_pose_stamped;
  std::map<gtsam::Key, gtsam::Pose3> pg_keyed_poses;
  std::unordered_map<gtsam::Key, ros::Time> pg_keyed_stamps;
  std::unordered_map<gtsam::Key, pose_graph_msgs::KeyedScan> pg_keyed_scans;

  for (const auto& robot : robot_names) {
    std::string gt_odom_bag = gt_path + "/" + robot + "_odom.bag";
    std::string gt_odom_topic = "/" + robot + "/lo_frontend/odometry";
    ROS_INFO("Reading ground truth trajectory from %s with topic %s",
             gt_odom_bag.c_str(),
             gt_odom_topic.c_str());

    char robot_prefix = utils::GetRobotPrefix(robot);
    gt_pose_stamped.insert({robot_prefix, PoseStamped()});

    if (!tu::ReadOdometryBagFile(
            gt_odom_bag, gt_odom_topic, &gt_pose_stamped[robot_prefix])) {
      ROS_ERROR("Failed to read ground truth odometry. ");
      return EXIT_FAILURE;
    }

    std::string lamp_bag = lamp_bag_path + "/" + robot + ".bag";
    ROS_INFO("Reading keyed scans and poses from %s", lamp_bag.c_str());

    if (!tu::ReadKeyedScansAndPosesFromBagFile(lamp_bag,
                                               robot,
                                               &pg_keyed_stamps,
                                               &pg_keyed_poses,
                                               &pg_keyed_scans)) {
      ROS_ERROR("Failed to read keyed scans and pose graph. ");
      return EXIT_FAILURE;
    }
  }

  pose_graph_msgs::LoopCandidateArray new_candidates, new_false_candidates;
  std::map<gtsam::Key, gtsam::Pose3> gt_keyed_poses;
  tu::FindLoopCandidateFromGt(gt_pose_stamped,
                              pg_keyed_stamps,
                              min_radius,
                              radius_tol,
                              10,
                              &new_candidates,
                              &gt_keyed_poses);

  tu::GenerateFalseLoopCandidateFromGt(
      gt_pose_stamped, pg_keyed_stamps, &new_false_candidates);

  // Add these newly found candidates
  if (!tu::AppendNewCandidates(new_candidates,
                               new_false_candidates,
                               pg_keyed_poses,
                               gt_keyed_poses,
                               pg_keyed_scans,
                               label,
                               &test_data)) {
    ROS_ERROR("Failed to append new candidates. ");
    return EXIT_FAILURE;
  }

  ROS_INFO("Writing %d candidates to dataset. ",
           test_data.real_candidates_.candidates.size());
  // Write to file
  if (!tu::WriteTestDataToFile(test_data, dataset_path)) {
    ROS_ERROR("Failed to write data to file. ");
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
